#!/usr/bin/env python3
"""ROS-free EtherCAT setup and heavy_v1 enable test console."""

from __future__ import annotations

import argparse
import ctypes
import gc
import math
import mmap
import os
from pathlib import Path
import pwd
import re

# Generated single source of truth for the CiA 402 status-word table (see
# generate_lift_console_states.py, generated from cia402.cpp).  This console
# used to keep its own copy of the mapping, which had drifted: it lacked 0x28
# (the LD3M word 0x0638 reported while a drive fault is latched) and used
# abbreviated state names that no longer matched the runtime parser.
from lift_console_states import CIA402_STATE_MASK, CIA402_STATE_NAMES
import select
import signal
import shutil
import subprocess
import sys
import termios
import threading
import time
from typing import Any
import tty
import unicodedata

try:
    import yaml
except ImportError as exc:  # pragma: no cover - deployment dependency check
    raise SystemExit("PyYAML is required: sudo apt install python3-yaml") from exc


SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_PATH = SCRIPT_DIR / "hardware_io.yaml"
ARM_AXIS_NAMES = tuple(
    [f"ljoint{index}" for index in range(1, 8)]
    + [f"rjoint{index}" for index in range(1, 8)]
)


def allocate_driver_log(configured_path: str | Path) -> Path:
    """Create a unique per-run driver log path and update a latest pointer."""
    base = Path(str(configured_path))
    log_root = os.environ.get("JOINT_CONTROLLER_LOG_ROOT", "").strip()
    dynamic_session = os.environ.get("JUNIOR_RUNTIME_LOG_DYNAMIC", "") == "1"
    if log_root and not dynamic_session:
        root = Path(log_root)
        base = root / base.name
    elif dynamic_session or not base.is_absolute() or str(base).startswith("/tmp/"):
        # Keep standalone invocations in the same persistent log hierarchy as
        # the managed server.  The shell entry points set
        # JOINT_CONTROLLER_LOG_ROOT; this fallback also covers direct calls to
        # ethercat_hardware_test.py and older configs that still say /tmp.
        run_user = (
            os.environ.get("JUNIOR_RUNTIME_LOG_USER", "").strip()
            or os.environ.get("SUDO_USER", "").strip()
        )
        if not run_user:
            run_user = pwd.getpwuid(os.getuid()).pw_name
            if run_user == "root":
                try:
                    workspace_owner = pwd.getpwuid(SCRIPT_DIR.parent.parent.stat().st_uid).pw_name
                except (KeyError, OSError):
                    workspace_owner = ""
                if workspace_owner and workspace_owner != "root":
                    run_user = workspace_owner
                else:
                    try:
                        pwd.getpwnam("user")
                    except KeyError:
                        pass
                    else:
                        run_user = "user"
        try:
            run_home = Path(pwd.getpwnam(run_user).pw_dir)
        except KeyError:
            run_home = Path.home()
        workspace_root = SCRIPT_DIR.parent.parent
        marker = Path(
            os.environ.get(
                "JOINT_CONTROLLER_SESSION_NAME_FILE",
                str(workspace_root / ".junior_runtime_session_name"),
            )
        )
        session_name = "standalone-hardware"
        try:
            candidate = marker.read_text(encoding="utf-8").strip()
        except OSError:
            candidate = ""
        if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", candidate):
            session_name = candidate
        log_root = os.environ.get("JUNIOR_LOG_ROOT", "").strip()
        if log_root:
            root = Path(log_root)
        else:
            root = run_home / ".ros" / "log"
        base = root / session_name / "runtime" / base.name
    stamp = time.strftime("%Y%m%d-%H%M%S")
    unique = base.with_name(f"{base.stem}-{stamp}-{os.getpid()}{base.suffix}")
    unique.parent.mkdir(parents=True, exist_ok=True)
    latest = base.with_name(f"{base.stem}.latest{base.suffix}")
    try:
        latest.unlink(missing_ok=True)
        latest.symlink_to(unique)
    except OSError:
        pass
    return unique


class AxisCommand(ctypes.Structure):
    _fields_ = [
        ("ec_mode", ctypes.c_uint8),
        ("ec_ctrword", ctypes.c_uint8),
        ("axis_position", ctypes.c_int32),
        ("axis_velocity", ctypes.c_int32),
        ("axis_effort", ctypes.c_int32),
        ("axis_counter", ctypes.c_uint8),
        ("axis_checksum", ctypes.c_uint8),
    ]


class AxisFeedback(ctypes.Structure):
    _fields_ = [
        ("ec_modestate", ctypes.c_uint8),
        ("ec_ctrstate", ctypes.c_uint8),
        ("axis_position", ctypes.c_int32),
        ("axis_velocity", ctypes.c_int32),
        ("axis_effort", ctypes.c_int32),
        ("axis_encoder_0", ctypes.c_int32),
        ("axis_encoder_1", ctypes.c_int32),
        ("axis_error_code", ctypes.c_uint16),
        ("axis_counter", ctypes.c_uint8),
        ("axis_checksum", ctypes.c_uint8),
    ]


class DesireRegion(ctypes.Structure):
    _fields_ = [
        ("ec_poweron", ctypes.c_uint8),
        ("axis_ctr", AxisCommand * 23),
        ("ec_axis_io", ctypes.c_uint8 * 16),
    ]


class RealRegion(ctypes.Structure):
    _fields_ = [
        ("ec_powerstate", ctypes.c_uint8),
        ("axis_state", AxisFeedback * 23),
    ]


class RawTerminal:
    def __init__(self) -> None:
        self.fd = -1
        self.settings: list[Any] | None = None

    def __enter__(self) -> "RawTerminal":
        if not sys.stdin.isatty():
            raise RuntimeError("arm keyboard console requires an interactive terminal")
        self.fd = sys.stdin.fileno()
        self.settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, _exc_type: Any, _exc_value: Any, _traceback: Any) -> None:
        if self.fd >= 0 and self.settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)

    def read_key(self, timeout_s: float | None = None) -> str:
        if self.fd < 0 or not select.select([self.fd], [], [], timeout_s)[0]:
            return ""
        first = os.read(self.fd, 1)
        if first != b"\x1b":
            if first in {b"\r", b"\n"}:
                return "enter"
            return first.decode("ascii", errors="ignore").lower()
        sequence = first
        for _ in range(2):
            if not select.select([self.fd], [], [], 0.02)[0]:
                break
            sequence += os.read(self.fd, 1)
        return {
            b"\x1b[A": "up",
            b"\x1b[B": "down",
        }.get(sequence, "")


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if not isinstance(config, dict):
        raise ValueError(f"configuration root must be a mapping: {path}")
    return config


def mapping(config: dict[str, Any]) -> dict[str, Any]:
    ethercat = config.get("ethercat")
    masters = ethercat.get("masters") if isinstance(ethercat, dict) else None
    if not isinstance(masters, dict) or not masters:
        raise ValueError("ethercat.masters must be a non-empty mapping")
    required = {
        "left_arm": (0, "enp3s0"),
        "right_arm": (1, "enp4s0"),
        "lift": (2, "enp5s0"),
    }
    for name, (index, interface) in required.items():
        value = masters.get(name)
        if not isinstance(value, dict):
            raise ValueError(f"missing ethercat.masters.{name}")
        if int(value.get("index", -1)) != index:
            raise ValueError(f"{name} must use Master{index}")
        if str(value.get("interface", "")) != interface:
            raise ValueError(f"{name} must use {interface}")
    return ethercat


def set_config_value(text: str, key: str, value: str) -> str:
    replacement = f'{key}="{value}"'
    pattern = re.compile(rf"^[ \t]*#?[ \t]*{re.escape(key)}=.*$", re.MULTILINE)
    if pattern.search(text):
        return pattern.sub(replacement, text, count=1)
    suffix = "" if text.endswith("\n") else "\n"
    return f"{text}{suffix}{replacement}\n"


def run(command: list[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, check=check, text=True, capture_output=True)


def ethercat_slave_counts() -> dict[int, int]:
    """Return the currently discovered slave count for each EtherLab master."""
    result = run(["ethercat", "master"], check=False)
    if result.returncode != 0:
        return {}
    counts: dict[int, int] = {}
    current_master: int | None = None
    for line in result.stdout.splitlines():
        header = re.match(r"^\s*Master(\d+)\s*$", line, re.IGNORECASE)
        if header:
            current_master = int(header.group(1))
            continue
        inline = re.search(r"\bMaster(\d+)\b.*\bSlaves:\s*(\d+)", line, re.IGNORECASE)
        if inline:
            counts[int(inline.group(1))] = int(inline.group(2))
            current_master = None
            continue
        slaves = re.match(r"^\s*Slaves:\s*(\d+)\s*$", line, re.IGNORECASE)
        if slaves and current_master is not None:
            counts[current_master] = int(slaves.group(1))
            current_master = None
    return counts


def interface_state(interface: str) -> str:
    state_path = Path("/sys/class/net") / interface / "operstate"
    try:
        return state_path.read_text(encoding="ascii").strip()
    except OSError as exc:
        raise RuntimeError(f"EtherCAT interface does not exist: {interface}") from exc


def print_mapping(config: dict[str, Any]) -> None:
    ethercat = mapping(config)
    print("EtherCAT mapping:")
    for name, value in ethercat["masters"].items():
        print(f"  Master{int(value['index'])}: {value['interface']} ({name})")


def check_setup(config: dict[str, Any]) -> int:
    ethercat = mapping(config)
    print_mapping(config)
    missing = False
    for value in ethercat["masters"].values():
        interface = str(value["interface"])
        try:
            print(f"  {interface}: {interface_state(interface)}")
        except RuntimeError as exc:
            print(f"  ERROR: {exc}", file=sys.stderr)
            missing = True
    for key in ("sysconfig_file", "conf_file", "init_script"):
        path = Path(str(ethercat[key]))
        print(f"  {key}: {path} ({'present' if path.exists() else 'missing'})")
        if not path.exists():
            missing = True
    return 1 if missing else 0


def configure_ethercat(config: dict[str, Any]) -> int:
    if os.geteuid() != 0:
        print("EtherCAT configuration requires root; use sudo.", file=sys.stderr)
        return 1
    ethercat = mapping(config)
    interfaces = [str(value["interface"]) for value in ethercat["masters"].values()]
    if len(set(interfaces)) != len(interfaces):
        raise ValueError("EtherCAT master interfaces must be distinct")
    for interface in interfaces:
        state = interface_state(interface)
        print(f"{interface}: {state}")

    config_paths = [Path(str(ethercat["sysconfig_file"])), Path(str(ethercat["conf_file"]))]
    if any(not path.is_file() for path in config_paths):
        missing = ", ".join(str(path) for path in config_paths if not path.is_file())
        raise RuntimeError(f"EtherLab configuration file is missing: {missing}")
    init_script = Path(str(ethercat["init_script"]))
    if not init_script.is_file():
        raise RuntimeError(f"EtherLab init script is missing: {init_script}")

    timestamp = time.strftime("%Y%m%d%H%M%S")
    backups: list[tuple[Path, Path]] = []
    for path in config_paths:
        backup = Path(f"{path}.heavy_v1.bak.{timestamp}")
        backup.write_bytes(path.read_bytes())
        backups.append((path, backup))

    for path in config_paths:
        text = path.read_text(encoding="utf-8")
        for index, interface in enumerate(interfaces):
            text = set_config_value(text, f"MASTER{index}_DEVICE", interface)
        text = set_config_value(text, "DEVICE_MODULES", str(ethercat.get("device_modules", "generic")))
        text = set_config_value(text, "UPDOWN_INTERFACES", " ".join(interfaces))
        path.write_text(text, encoding="utf-8")

    unmanaged = Path(str(ethercat["network_manager_unmanaged_file"]))
    unmanaged.parent.mkdir(parents=True, exist_ok=True)
    unmanaged.write_text(
        "[keyfile]\n"
        f"unmanaged-devices={';'.join('interface-name:' + item for item in interfaces)}\n",
        encoding="utf-8",
    )
    if shutil_available("nmcli"):
        for interface in interfaces:
            run(["nmcli", "device", "set", interface, "managed", "no"], check=False)
    for interface in interfaces:
        run(["ip", "link", "set", interface, "up"])

    run([str(init_script), "stop"], check=False)
    started = run([str(init_script), "start"], check=False)
    if started.returncode != 0:
        restore_configs(backups, init_script)
        print(started.stderr, file=sys.stderr, end="")
        return 2
    devices = [Path(f"/dev/EtherCAT{index}") for index in range(len(interfaces))]
    if any(not device.exists() for device in devices):
        print("EtherLab did not create all configured masters.", file=sys.stderr)
        restore_configs(backups, init_script)
        return 2

    # EtherLab creates the master device immediately, but discovers slaves
    # asynchronously as links come up. Do not report success until every
    # configured master has discovered its expected slave count.
    master_expectations = {
        int(value["index"]): int(value["expected_slaves"])
        for value in ethercat["masters"].values()
        if "expected_slaves" in value
    }
    master_indices = [int(value["index"]) for value in ethercat["masters"].values()]
    deadline = time.monotonic() + 15.0
    counts: dict[int, int] = {}
    while time.monotonic() < deadline:
        counts = ethercat_slave_counts()
        if all(
            counts.get(index, 0) > 0
            and (index not in master_expectations or counts[index] == master_expectations[index])
            for index in master_indices
        ):
            break
        time.sleep(0.25)
    else:
        formatted = ", ".join(
            f"Master{index}={counts.get(index, 0)}"
            + (f" (expected {master_expectations[index]})" if index in master_expectations else "")
            for index in master_indices
        )
        print(
            "EtherLab started but no complete slave discovery after 15 seconds "
            f"({formatted}); check EtherCAT links, power, and cabling.",
            file=sys.stderr,
        )
        restore_configs(backups, init_script)
        return 2

    print("EtherLab configured and started:")
    print_mapping(config)
    print("Backups:")
    for _, backup in backups:
        print(f"  {backup}")
    return 0


def shutil_available(command: str) -> bool:
    return shutil.which(command) is not None


def restore_configs(backups: list[tuple[Path, Path]], init_script: Path) -> None:
    run([str(init_script), "stop"], check=False)
    for path, backup in backups:
        path.write_bytes(backup.read_bytes())


class SharedMemory:
    # Mappings which still had live ctypes views at close() time.  Keeping a
    # reference stops Python from retrying a failing close during garbage
    # collection; the kernel releases the mapping at process exit.
    deferred_close: list[mmap.mmap] = []

    def __init__(self, name: str, structure: type[ctypes.Structure]) -> None:
        self.path = Path("/dev/shm") / name
        self.structure = structure
        self.fd = -1
        self.file_map: mmap.mmap | None = None
        self.value: ctypes.Structure | None = None

    def open(self) -> "SharedMemory":
        try:
            self.fd = os.open(self.path, os.O_RDWR)
        except OSError as exc:
            raise RuntimeError(f"shared memory is not available: {self.path}") from exc
        size = ctypes.sizeof(self.structure)
        try:
            self.file_map = mmap.mmap(self.fd, size, access=mmap.ACCESS_WRITE)
            self.value = self.structure.from_buffer(self.file_map)
        except (OSError, ValueError) as exc:
            self.close()
            raise RuntimeError(f"cannot map {self.path}: {exc}") from exc
        return self

    def close(self) -> None:
        """Release the mapping without ever raising on the console exit path.

        A structure created with ``from_buffer()`` keeps an exported pointer
        into the mapping, and so does every nested view of it
        (``real.axis_state[slot]``, ``desire.axis_ctr[slot]`` and friends).
        ``mmap.close()`` raises ``BufferError`` while any of those views is
        still alive, which used to abort the exit path before the IGH driver
        was stopped and left ``/dev/EtherCAT*`` open for the next run.  A
        mapping that cannot be closed yet is kept referenced so the kernel
        releases it when the process ends instead of raising again.
        """
        self.value = None
        if self.file_map is not None:
            file_map = self.file_map
            self.file_map = None
            try:
                file_map.close()
            except BufferError:
                gc.collect()
                try:
                    file_map.close()
                except BufferError:
                    SharedMemory.deferred_close.append(file_map)
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


def open_driver_shared_memory(
    desire_block: SharedMemory,
    real_block: SharedMemory,
    driver_process: Any,
    log_path: Path,
    timeout_s: float = 10.0,
) -> None:
    """Wait for the IGH driver's shared-memory segments, then map both of them.

    The mapping itself is retried instead of being gated on ``path.exists()``.
    A POSIX segment exists as soon as it is created but is only grown to its
    final size by a later ``truncate()``, so an existence check can still observe
    a segment that is too small to map ("mmap length is greater than file
    size").  The same happens on a cold start whenever another consumer left the
    segment shrunk: the ROS ``erobot_hw`` interface truncates
    ``Ethercat_axis_desire`` to its own 464-byte struct, while the IGH driver and
    this tool both expect the 480-byte ABI.  Retrying lets the driver grow the
    segment instead of failing the run.
    """
    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise ValueError("shared-memory wait timeout must be positive and finite")
    deadline = time.monotonic() + timeout_s
    last_map_error = ""
    while time.monotonic() < deadline:
        if driver_process.poll() is not None:
            raise RuntimeError(
                f"igh_driver exited with code {driver_process.returncode}; see {log_path}"
            )
        try:
            desire_block.open()
            real_block.open()
            return
        except RuntimeError as exc:
            last_map_error = str(exc)
            desire_block.close()
            real_block.close()
            time.sleep(0.1)
    detail = f" ({last_map_error})" if last_map_error else ""
    raise RuntimeError(f"driver did not create shared memory; see {log_path}{detail}")


def process_pids(binary: Path) -> list[int]:
    pids: list[int] = []
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            executable = proc / "exe"
            if os.path.samefile(executable, binary):
                pids.append(int(proc.name))
        except OSError:
            continue
    return pids


def resolve_driver(config: dict[str, Any], config_path: Path) -> Path:
    arm = config.get("arm", {})
    configured = Path(str(arm.get("driver_binary", "")))
    candidates = []
    if configured:
        candidates.append((config_path.parent / configured).resolve())
    repo_root = config_path.parent.parent.parent
    candidates.extend([
        repo_root / "build/erobot_igh_driver/igh_driver",
        repo_root / "src/erobot_igh_driver/build/igh_driver",
        Path("/home/user/joint_controller/build/erobot_igh_driver/igh_driver"),
        Path("/home/user/joint_controller/src/erobot_igh_driver/build/igh_driver"),
    ])
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    raise RuntimeError("igh_driver binary was not found; build erobot_igh_driver first")


def check_driver(config: dict[str, Any], config_path: Path) -> int:
    driver = resolve_driver(config, config_path)
    pids = process_pids(driver)
    if not pids:
        print(f"OK: no running driver: {driver}")
        return 0
    print(f"ERROR: driver already running: {driver}", file=sys.stderr)
    print(f"  PID(s): {', '.join(str(pid) for pid in pids)}", file=sys.stderr)
    print("Stop the owning ROS/IGH service first, or stop a confirmed standalone process with:", file=sys.stderr)
    print(f"  sudo kill -INT {' '.join(str(pid) for pid in pids)}", file=sys.stderr)
    return 2


def state_name(status: int) -> str:
    """CiA 402 state name for a raw 6041h status word.

    Decoded from the generated table so this console cannot drift from the
    runtime parser again; unknown words still print as hex.
    """
    decoded = CIA402_STATE_NAMES.get(status & CIA402_STATE_MASK)
    return decoded if decoded is not None else f"0x{status:04X}"


# CiA402 object 0x603F ("Error code") as reported by the arm drives.  The IGH
# driver reads it over SDO and republishes it as axis_state[].axis_error_code,
# and both the driver fault check and arm_status() treat any non-zero value as
# a fault, so 0x0000 is the only value that means "healthy".
#
# The named entries below are the CiA 301 emergency codes that the DS402 drive
# profiles reuse for 0x603F.  They are a reading aid, not a complete table: any
# value that is missing here is a drive-vendor code, and the authoritative list
# for these joints is the ZeroErr (eRunner) drive manual -- for example ZeroErr
# documents the vendor-specific joint code 0x730F, which is not a CiA 301 code.
AXIS_ERROR_NAMES = {
    0x0000: "无错误",
    0x2310: "过流",
    0x3110: "主电源过压",
    0x3120: "主电源欠压",
    0x3210: "母线过压",
    0x3220: "母线欠压",
    0x4210: "过温",
    0x7300: "编码器故障",
}

AXIS_ERROR_UNKNOWN = "厂商专有码,查ZeroErr手册"


def axis_error_name(code: int) -> str:
    return AXIS_ERROR_NAMES.get(code, AXIS_ERROR_UNKNOWN)


# ---------------------------------------------------------------------------
# Shared page vocabulary.
#
# This console and the upper-machine remote console
# (junior_ws/tools/hardware/remote/remote_console.py) print the same two arm
# pages, so the columns, the Chinese status words and the key footer below are
# kept identical between the two implementations.  The raw hex code stays next
# to the Chinese wording so logs and the drive manual can still be matched.
# ---------------------------------------------------------------------------
ARM_ENABLE_LABELS = {True: "使能", False: "失能"}

UNKNOWN_ERROR_NAME = "未知故障码"

# CiA 402 status word (masked with 0x006F) -> page label.  The same uppercase
# names are used by the lower hardware server and by the upper remote console.
ARM_STATE_LABELS = {
    0x08: "FAULT",
    0x21: "READY_TO_SWITCH_ON",
    0x23: "SWITCHED_ON",
    0x27: "OPERATION_ENABLED",
    0x40: "SWITCH_ON_DISABLED",
}


def display_width(text: str) -> int:
    """Return the terminal cell width of text (CJK glyphs occupy two cells)."""
    return sum(2 if unicodedata.east_asian_width(char) in "WF" else 1 for char in text)


def pad_display(text: str, width: int) -> str:
    return text + " " * max(0, width - display_width(text))


def arm_error_text(code: int) -> str:
    value = int(code) & 0xFFFF
    return f"0x{value:04X} ({AXIS_ERROR_NAMES.get(value, UNKNOWN_ERROR_NAME)})"


# A drive that reports 0x603F only transiently (the ZeroLegacy profiles clear it
# again after the driver's fault reset) makes a single frame show whichever axes
# happened to be non-zero at that instant, so the page looks like the fault is
# hopping between arms.  Keep the last non-zero code of every joint for this
# many seconds and show its age, so one page lists every joint that is currently
# in trouble.  C clears the memory immediately.  The upper-machine remote
# console (junior_ws/tools/hardware/remote/remote_console.py) carries the same
# constant, class, column width and footer.
ARM_ERROR_LATCH_S = 5.0


class ArmErrorLatch:
    """Remember the most recent non-zero 0x603F code per joint."""

    def __init__(self, window_s: float = ARM_ERROR_LATCH_S) -> None:
        self.window_s = float(window_s)
        self.latched: dict[str, tuple[int, float]] = {}

    def update(self, joint: str, code: Any, now: float) -> None:
        value = int(code) & 0xFFFF
        if value:
            self.latched[joint] = (value, now)

    def cell(self, joint: str, code: Any, now: float) -> str:
        """Return the Error column text: live code, aged latched code, or 无错误."""
        value = int(code) & 0xFFFF
        if value:
            return arm_error_text(value)
        entry = self.latched.get(joint)
        if entry is not None:
            latched_code, seen_at = entry
            age = now - seen_at
            if age <= self.window_s:
                return f"{arm_error_text(latched_code)} {age:.1f}s前"
            del self.latched[joint]
        return arm_error_text(0)

    def clear(self) -> int:
        count = len(self.latched)
        self.latched.clear()
        return count


# ---------------------------------------------------------------------------
# Read-only bus voltage reporting.
#
# Every drive publishes its own DC link voltage and its own undervoltage limit
# over SDO, and `ethercat upload` needs no sudo and keeps working while the IGH
# driver owns the masters, so the console can show both without touching the
# driver, the PDO mapping or the shared-memory ABI.  Values are millivolts:
#
#   EYOU axes     0x6079 voltage, 0x202D:01 undervoltage, 0x202D:03 overvoltage
#   ZeroErr axes  0x6079 voltage, 0x3B6F minimum bus,  0x3B6E maximum bus
#
# On this machine the six ZeroErr axes require >= 44.0 V while the bus sits near
# 39 V, so they report 0x3220 (母线欠压) instead of reaching Operation Enabled.
# The nominal bus voltage is deliberately not printed: it is still being
# confirmed against the supply.
# ---------------------------------------------------------------------------
BUS_VOLTAGE_INDEX = 0x6079
ZEROERR_MIN_BUS_INDEX = 0x3B6F
ZEROERR_MAX_BUS_INDEX = 0x3B6E
EYOU_LIMIT_INDEX = 0x202D
EYOU_UNDERVOLTAGE_SUBINDEX = 0x01
EYOU_OVERVOLTAGE_SUBINDEX = 0x03
ZEROERR_FAMILY = "ZeroErr"
EYOU_FAMILY = "EYOU"
BUS_VOLTAGE_REFRESH_S = 2.0
ETHERCAT_UPLOAD_TIMEOUT_S = 1.0


def parse_ethercat_upload(text: str) -> int | None:
    """Return the integer value printed by `ethercat upload`, or None."""
    for token in reversed(text.split()):
        try:
            return int(token, 0)
        except ValueError:
            continue
    return None


def read_ethercat_object(
    master: int,
    position: int,
    index: int,
    subindex: int = 0x00,
    timeout_s: float = ETHERCAT_UPLOAD_TIMEOUT_S,
) -> int | None:
    """Read one SDO object with the read-only `ethercat` CLI.

    Returns None when the tool is missing, the object does not exist on this
    drive, or the request times out, so callers never have to distinguish those
    cases: an absent value simply is not displayed.
    """
    command = [
        "ethercat",
        "upload",
        "-m",
        str(int(master)),
        "-p",
        str(int(position)),
        f"0x{int(index):04X}",
        f"0x{int(subindex):02X}",
    ]
    try:
        completed = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout_s, check=False
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None
    return parse_ethercat_upload(completed.stdout)


def arm_axis_master_positions(config: dict[str, Any]) -> dict[str, tuple[int, int]]:
    """Map every arm joint name to its (master index, slave position)."""
    masters = mapping(config).get("masters", {})
    positions: dict[str, tuple[int, int]] = {}
    slots = config.get("arm", {}).get("axis_slots", {})
    for group, master_key in (("left_arm", "left_arm"), ("right_arm", "right_arm")):
        master = masters.get(master_key)
        if not isinstance(master, dict) or "index" not in master:
            continue
        for local, slot in enumerate(slots.get(group, [])):
            positions[arm_axis_name(int(slot))] = (int(master["index"]), local)
    return positions


class BusVoltageMonitor:
    """Poll the drives' DC link voltage and undervoltage limits off the loop.

    The reads run on a background thread so the 50 Hz motion tick and the page
    rendering never block on a subprocess.  ``snapshot()`` returns a plain dict
    that is also the payload the hardware server forwards to the upper console.
    """

    def __init__(
        self,
        positions: dict[str, tuple[int, int]],
        interval_s: float = BUS_VOLTAGE_REFRESH_S,
    ) -> None:
        self.positions = dict(positions)
        self.interval_s = float(interval_s)
        self.voltage_mv: dict[str, int] = {}
        self.limits: dict[str, dict[str, Any]] = {}
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def start(self) -> None:
        if not self.positions or shutil.which("ethercat") is None:
            return
        self._thread = threading.Thread(target=self._run, name="bus-voltage", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def snapshot(self) -> dict[str, Any]:
        return {
            "voltage_mv": dict(self.voltage_mv),
            "limits": {joint: dict(limit) for joint, limit in self.limits.items()},
        }

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.sample_once()
            except Exception:  # noqa: BLE001 - a read-only probe must never kill the console
                pass
            self._stop_event.wait(self.interval_s)

    def sample_once(self) -> None:
        """Refresh the voltage of every axis and probe each axis' limits once."""
        voltage = dict(self.voltage_mv)
        limits = dict(self.limits)
        changed = False
        for joint, (master, position) in self.positions.items():
            if self._stop_event.is_set():
                break
            value = read_ethercat_object(master, position, BUS_VOLTAGE_INDEX)
            if value is not None:
                voltage[joint] = value
                changed = True
            if joint not in limits:
                limit = probe_bus_voltage_limits(master, position)
                if limit is not None:
                    limits[joint] = limit
                    changed = True
        if changed:
            self.voltage_mv = voltage
            self.limits = limits


def probe_bus_voltage_limits(master: int, position: int) -> dict[str, Any] | None:
    """Return the drive's own bus voltage window, or None if it exposes none."""
    zero_min = read_ethercat_object(master, position, ZEROERR_MIN_BUS_INDEX)
    if zero_min is not None:
        return {
            "family": ZEROERR_FAMILY,
            "min_mv": zero_min,
            "max_mv": read_ethercat_object(master, position, ZEROERR_MAX_BUS_INDEX),
        }
    eyou_min = read_ethercat_object(
        master, position, EYOU_LIMIT_INDEX, EYOU_UNDERVOLTAGE_SUBINDEX
    )
    if eyou_min is not None:
        return {
            "family": EYOU_FAMILY,
            "min_mv": eyou_min,
            "max_mv": read_ethercat_object(
                master, position, EYOU_LIMIT_INDEX, EYOU_OVERVOLTAGE_SUBINDEX
            ),
        }
    return None


def bus_status_line(bus: Any) -> str | None:
    """Format the shared voltage line: spread, per family count and limit."""
    if not isinstance(bus, dict):
        return None
    voltage = bus.get("voltage_mv")
    if not isinstance(voltage, dict) or not voltage:
        return None
    limits = bus.get("limits")
    limits = limits if isinstance(limits, dict) else {}

    readings: dict[str, int] = {}
    for joint, value in voltage.items():
        if isinstance(value, (int, float)):
            readings[str(joint)] = int(value)
    if not readings:
        return None

    family_of: dict[str, str] = {}
    limit_of: dict[str, int] = {}
    for joint in readings:
        limit = limits.get(joint)
        if not isinstance(limit, dict):
            continue
        min_mv = limit.get("min_mv")
        if not isinstance(min_mv, (int, float)):
            continue
        family_of[joint] = str(limit.get("family", "?"))
        limit_of[joint] = int(min_mv)

    parts = [f"Bus {min(readings.values()) / 1000.0:.2f}-{max(readings.values()) / 1000.0:.2f}V"]
    for family in sorted(set(family_of.values())):
        joints = [joint for joint in family_of if family_of[joint] == family]
        threshold = "/".join(
            f"{mv / 1000.0:.1f}V" for mv in sorted({limit_of[joint] for joint in joints})
        )
        low = any(readings[joint] < limit_of[joint] for joint in joints)
        parts.append(f"{family} {len(joints)}x min {threshold}{' LOW' if low else ''}")
    return " | ".join(parts)


def arm_state_label(axis: Any) -> str:
    return ARM_STATE_LABELS.get(int(axis.ec_ctrstate) & 0x006F, "UNKNOWN")


def arm_axis_enabled(axis: Any) -> bool:
    return int(axis.ec_ctrstate) & 0x006F == 0x27


def arm_ramp_target(current_units: int, requested_units: int, max_delta_units: float) -> int:
    """Step an encoder target toward its request by at most one speed-limited step."""
    difference = requested_units - current_units
    if difference == 0:
        return current_units
    step = int(max_delta_units)
    if step < 1:
        step = 1
    if abs(difference) <= step:
        return requested_units
    return current_units + (step if difference > 0 else -step)


def arm_status(real: RealRegion, slots: list[int]) -> tuple[int, int, int, int]:
    enabled = 0
    faults = 0
    modes_ok = 0
    for slot in slots:
        axis = real.axis_state[slot]
        if axis.ec_ctrstate == 0x27:
            enabled += 1
        if axis.ec_ctrstate & 0x08 or axis.axis_error_code:
            faults += 1
        if axis.ec_modestate == 0x08:
            modes_ok += 1
    return enabled, faults, modes_ok, len(slots)


def arm_feedback_ready(real: RealRegion, slots: list[int], csp_mode: int) -> bool:
    if real.ec_powerstate == 0:
        return False
    return all(
        real.axis_state[slot].ec_modestate == csp_mode
        and real.axis_state[slot].ec_ctrstate != 0
        for slot in slots
    )


def print_arm_status(
    real: RealRegion, slots: list[int], units_per_rad: float = 83443.02680376362
) -> None:
    enabled, faults, modes_ok, total = arm_status(real, slots)
    print(
        f"powerstate={real.ec_powerstate} enabled={enabled}/{total} "
        f"csp_mode={modes_ok}/{total} faults={faults}"
    )
    for slot in slots:
        axis = real.axis_state[slot]
        print(
            f"  axis {slot:02d}: {state_name(axis.ec_ctrstate):18s} "
            f"mode=0x{axis.ec_modestate:02X} pos={axis.axis_position} "
            f"({axis.axis_position / units_per_rad:.5f} rad) "
            f"vel={axis.axis_velocity} error=0x{axis.axis_error_code:04X}"
        )


def arm_units_per_rad(config: dict[str, Any]) -> float:
    value = float(config["arm"].get("position_units_per_rad", 83443.02680376362))
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("arm.position_units_per_rad must be positive and finite")
    return value


def arm_rad_to_units(position_rad: float, units_per_rad: float) -> int:
    if not math.isfinite(position_rad):
        raise ValueError("arm position must be finite")
    value = int(round(position_rad * units_per_rad))
    if not -(2**31) <= value <= 2**31 - 1:
        raise ValueError("arm position is outside int32 range")
    return value


def arm_logical_position(raw_units: int, zero_units: int, units_per_rad: float) -> float:
    return (raw_units - zero_units) / units_per_rad


def arm_axis_name(slot: int) -> str:
    if slot < 0 or slot >= len(ARM_AXIS_NAMES):
        raise ValueError(f"invalid arm axis slot: {slot}")
    return ARM_AXIS_NAMES[slot]


def arm_zero_offset_path(config: dict[str, Any], config_path: Path) -> Path:
    value = str(config.get("arm", {}).get("zero_offset_file", "")).strip()
    if not value:
        raise ValueError("arm.zero_offset_file must not be empty")
    path = Path(value).expanduser()
    return path if path.is_absolute() else (config_path.parent / path).resolve()


def load_arm_zero_offsets(
    config: dict[str, Any], config_path: Path, slots: list[int]
) -> dict[int, int]:
    offsets = {slot: 0 for slot in slots}
    path = arm_zero_offset_path(config, config_path)
    if not path.exists():
        return offsets
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    values = data.get("zero_offsets", {}) if isinstance(data, dict) else {}
    if not isinstance(values, dict):
        raise ValueError(f"zero_offsets must be a mapping: {path}")
    for slot in slots:
        value = values.get(arm_axis_name(slot), 0)
        try:
            offset = int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid zero offset for {arm_axis_name(slot)}: {value}") from exc
        if not -(2**31) <= offset <= 2**31 - 1:
            raise ValueError(f"zero offset for {arm_axis_name(slot)} is outside int32 range")
        offsets[slot] = offset
    return offsets


def save_arm_zero_offsets(
    config: dict[str, Any], config_path: Path, offsets: dict[int, int]
) -> None:
    """Write the arm logical-zero table.

    The interactive joint page is W/S/D/E/H/Q on both machines (the upper
    remote console has no zero-mark key), so nothing binds this writer to a key
    any more.  It is kept as the single authoritative writer for
    `arm.zero_offset_file` so re-zeroing stays a supported, auditable operation
    instead of a hand-edited YAML change.
    """
    path = arm_zero_offset_path(config, config_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": 1,
        "zero_offsets": {
            arm_axis_name(slot): int(offsets[slot])
            for slot in sorted(offsets)
        },
    }
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    try:
        temporary.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    owner_name = os.environ.get("SUDO_USER")
    if owner_name:
        try:
            owner = pwd.getpwnam(owner_name)
            os.chown(path, owner.pw_uid, owner.pw_gid)
        except KeyError:
            pass


def render_arm_selection(
    real: RealRegion,
    slots: list[int],
    selected_slot: int,
    units_per_rad: float,
    zero_offsets: dict[int, int],
    message: str,
    error_latch: ArmErrorLatch | None = None,
    bus: Any = None,
) -> None:
    """Print the arm summary page (总表).

    The columns, the Chinese status words, the all-axis keys (E/D/H), the latch
    clear (C) and the footer are identical to the upper-machine remote console
    (junior_ws/tools/hardware/remote/remote_console.py::render_arm_state).  The
    Error column keeps the last non-zero code of every joint for
    ARM_ERROR_LATCH_S seconds and shows its age, because the ZeroLegacy drives
    report 0x603F only in ~100 ms bursts and one frame would otherwise show a
    rotating subset of the joints that are in trouble.  The summary page has no
    Space binding: Space was a no-op while nothing was moving and D already
    aborts a move and disables all drives.
    """
    print("\033[2J\033[H", end="")
    now = time.monotonic()
    enabled, faults, modes_ok, total = arm_status(real, slots)
    selected_joint = arm_axis_name(selected_slot) if selected_slot in slots else "?"
    print(f"ARM | SELECTED {slots.index(selected_slot) + 1}/{len(slots)}: {selected_joint}")
    print("Select axis with Up/Down, Enter to control, Q to back")
    print(f"Power: {real.ec_powerstate}  Enabled: {enabled}/{total}")
    bus_line = bus_status_line(bus)
    if bus_line:
        print(bus_line)
    print(
        "No.  "
        + pad_display("Joint", 10)
        + pad_display("Enable", 8)
        + pad_display("Status", 22)
        + pad_display("Error", 26)
        + "Position(rad)"
    )
    for index, slot in enumerate(slots):
        axis = real.axis_state[slot]
        marker = ">>" if slot == selected_slot else "  "
        joint = arm_axis_name(slot)
        error = axis.axis_error_code
        if error_latch is None:
            error_cell = arm_error_text(error)
        else:
            error_latch.update(joint, error, now)
            error_cell = error_latch.cell(joint, error, now)
        print(
            f"{marker}{index + 1:02d} "
            + pad_display(joint, 10)
            + pad_display(ARM_ENABLE_LABELS[arm_axis_enabled(axis)], 8)
            + pad_display(arm_state_label(axis), 22)
            + pad_display(error_cell, 26)
            + f"{arm_logical_position(axis.axis_position, zero_offsets[slot], units_per_rad): .5f}"
        )
    if message:
        print(f"\n{message}")
    print(
        "\nUp/Down SELECT  Enter CONTROL  E ENABLE ALL  D DISABLE ALL  "
        "H HOME ALL  C CLEAR ERR  Q BACK"
    )
    # The launcher redirects stdout into a pipe (tee), so stdout is block
    # buffered and one 1 KiB frame would stay invisible until the buffer fills
    # or the process exits.  Flush every frame so the menu appears immediately.
    sys.stdout.flush()


def render_arm_control(
    real: RealRegion,
    desire: DesireRegion,
    slot: int,
    units_per_rad: float,
    zero_units: int,
    target_units: int,
    max_speed_rad_s: float,
    message: str,
) -> None:
    """Print the single-joint page.

    The key set is exactly W/S/D/E/H/Q on both machines: W and S jog, D
    disables, E enables, H returns the joint to its logical zero.
    """
    axis = real.axis_state[slot]
    print("\033[2J\033[H", end="")
    print(f"ARM | CONTROL {arm_axis_name(slot)}")
    print(f"Power: {bool(desire.ec_poweron)}  Enabled: {ARM_ENABLE_LABELS[arm_axis_enabled(axis)]}")
    print(f"Status: {arm_state_label(axis)}  Mode: 0x{axis.ec_modestate:02X}  Error: {arm_error_text(axis.axis_error_code)}")
    print(f"Position(rad): {arm_logical_position(axis.axis_position, zero_units, units_per_rad): .5f}")
    print(f"Target(rad)  : {arm_logical_position(target_units, zero_units, units_per_rad): .5f}")
    print(f"Zero offset  : {zero_units} units")
    print(f"Max speed    : {max_speed_rad_s:.5f} rad/s")
    print("\nW +STEP  S -STEP  D DISABLE  E ENABLE  H HOME  Q BACK")
    print(message)
    # Same reason as render_arm_selection: keep this frame visible while the
    # console blocks on the next keypress.
    sys.stdout.flush()


def run_arm_test(config: dict[str, Any], config_path: Path) -> int:
    if os.geteuid() != 0:
        print("Arm EtherCAT test requires root; use sudo.", file=sys.stderr)
        return 1
    arm = config["arm"]
    units_per_rad = arm_units_per_rad(config)
    max_step_rad = float(arm.get("max_step_rad", 0.1))
    if not math.isfinite(max_step_rad) or max_step_rad <= 0.0:
        raise ValueError("arm.max_step_rad must be positive and finite")
    driver = resolve_driver(config, config_path)
    pids = process_pids(driver)
    if pids:
        raise RuntimeError(
            f"driver already running: {driver} (PID(s): {', '.join(str(pid) for pid in pids)}); "
            "stop ROS/IGH before this test"
        )
    log_path = allocate_driver_log(arm.get("driver_log", "heavy_v1_igh_driver.log"))
    log_file = log_path.open("wb")
    env = os.environ.copy()
    library_dir = str(config.get("ethercat", {}).get("library_dir", "/usr/local/etherlab/lib"))
    env["LD_LIBRARY_PATH"] = library_dir + ":" + env.get("LD_LIBRARY_PATH", "")
    driver_process = subprocess.Popen([str(driver)], stdout=log_file, stderr=subprocess.STDOUT, env=env)
    desire_block = SharedMemory(str(arm["shared_memory"]["desire"]), DesireRegion)
    real_block = SharedMemory(str(arm["shared_memory"]["real"]), RealRegion)
    real: RealRegion | None = None
    desire: DesireRegion | None = None
    bus_monitor: BusVoltageMonitor | None = None
    try:
        # Retry the mapping itself instead of gating it on path.exists(): see
        # open_driver_shared_memory() for why an existing segment can still be
        # too small to map, which used to make every first run of the console
        # fail with "mmap length is greater than file size".
        open_driver_shared_memory(desire_block, real_block, driver_process, log_path)

        slots = list(arm["axis_slots"]["left_arm"]) + list(arm["axis_slots"]["right_arm"])
        # Read-only bus voltage / undervoltage limit display; the reads run on a
        # background thread so the motion tick never waits for a subprocess.
        bus_monitor = BusVoltageMonitor(arm_axis_master_positions(config))
        bus_monitor.start()
        zero_offsets = load_arm_zero_offsets(config, config_path, slots)
        csp_mode = int(str(arm.get("csp_mode", "0x08")), 0)
        jog_step_rad = float(arm.get("jog_step_rad", 0.005))
        if not math.isfinite(jog_step_rad) or jog_step_rad <= 0.0 or jog_step_rad > max_step_rad:
            raise ValueError("arm.jog_step_rad must be positive and no larger than max_step_rad")
        jog_step_units = arm_rad_to_units(jog_step_rad, units_per_rad)
        max_speed_rad_s = float(arm.get("max_speed_rad_s", 0.05))
        if not math.isfinite(max_speed_rad_s) or max_speed_rad_s <= 0.0:
            raise ValueError("arm.max_speed_rad_s must be positive and finite")
        max_speed_units_s = max_speed_rad_s * units_per_rad
        feedback_timeout_s = float(arm.get("feedback_ready_timeout_s", 35.0))
        if not math.isfinite(feedback_timeout_s) or feedback_timeout_s <= 0.0:
            raise ValueError("arm.feedback_ready_timeout_s must be positive and finite")

        feedback_deadline = time.monotonic() + feedback_timeout_s
        while time.monotonic() < feedback_deadline:
            real = real_block.value
            if real is not None and arm_feedback_ready(real, slots, csp_mode):
                break
            if driver_process.poll() is not None:
                raise RuntimeError(f"igh_driver exited with code {driver_process.returncode}; see {log_path}")
            time.sleep(0.02)
        else:
            raise RuntimeError(
                "driver feedback is not ready; refusing to enable drives "
                f"(see {log_path})"
            )

        selected_index = 0

        def hold_all_axes() -> None:
            real = real_block.value
            desire = desire_block.value
            assert real is not None and desire is not None
            for slot in slots:
                desire.axis_ctr[slot].ec_mode = csp_mode
                desire.axis_ctr[slot].axis_position = real.axis_state[slot].axis_position
                desire.axis_ctr[slot].axis_velocity = 0
                desire.axis_ctr[slot].axis_effort = 0

        def all_axes_enabled() -> bool:
            real = real_block.value
            if real is None:
                return False
            enabled, faults, modes_ok, total = arm_status(real, slots)
            return enabled == total and faults == 0 and modes_ok == total

        def synchronize_targets_to_feedback() -> bool:
            real = real_block.value
            desire = desire_block.value
            if real is None or desire is None or not arm_feedback_ready(real, slots, csp_mode):
                return False
            for slot in slots:
                desire.axis_ctr[slot].ec_mode = csp_mode
                desire.axis_ctr[slot].axis_position = real.axis_state[slot].axis_position
                desire.axis_ctr[slot].axis_velocity = 0
                desire.axis_ctr[slot].axis_effort = 0
            desire.ec_poweron = 0
            return True

        def disable_all_axes() -> None:
            hold_all_axes()
            desire = desire_block.value
            assert desire is not None
            desire.ec_poweron = 0

        def control_selected_joint(terminal: RawTerminal, slot: int) -> str:
            real = real_block.value
            desire = desire_block.value
            assert real is not None and desire is not None
            hold_all_axes()
            target_units = real.axis_state[slot].axis_position
            requested_target_units = target_units
            message = "Press E to enable, then W/S to jog. Press Q to return."
            limit_stall_timeout_s = float(arm.get("limit_stall_timeout_s", 0.5))
            limit_tolerance_units = arm_rad_to_units(
                float(arm.get("limit_position_tolerance_rad", 0.001)), units_per_rad
            )
            if not math.isfinite(limit_stall_timeout_s) or limit_stall_timeout_s <= 0.0:
                raise ValueError("arm.limit_stall_timeout_s must be positive and finite")
            if limit_tolerance_units <= 0:
                raise ValueError("arm.limit_position_tolerance_rad must be positive")
            last_target_update = time.monotonic()
            motion_direction = 0
            motion_watch_position = target_units
            motion_watch_started = last_target_update
            blocked_direction = 0

            def update_target_trajectory() -> None:
                nonlocal target_units, last_target_update
                if not desire.ec_poweron or not all_axes_enabled():
                    last_target_update = time.monotonic()
                    return
                now = time.monotonic()
                elapsed = min(max(now - last_target_update, 0.0), 0.1)
                max_delta = max_speed_units_s * elapsed
                difference = requested_target_units - target_units
                if abs(difference) <= max_delta:
                    target_units = requested_target_units
                elif difference > 0:
                    target_units += int(max_delta)
                else:
                    target_units -= int(max_delta)
                desire.axis_ctr[slot].ec_mode = csp_mode
                desire.axis_ctr[slot].axis_position = target_units
                last_target_update = now

            def observe_motion_limit() -> None:
                nonlocal target_units, requested_target_units
                nonlocal motion_watch_position, motion_watch_started
                nonlocal motion_direction, blocked_direction
                nonlocal redraw
                if motion_direction == 0 or not desire.ec_poweron:
                    return
                actual = real.axis_state[slot].axis_position
                progress = (actual - motion_watch_position) * motion_direction
                if progress >= limit_tolerance_units:
                    motion_watch_position = actual
                    motion_watch_started = time.monotonic()
                    return
                target_error = (requested_target_units - actual) * motion_direction
                velocity_limit = max(1, arm_rad_to_units(0.002, units_per_rad))
                if (
                    time.monotonic() - motion_watch_started >= limit_stall_timeout_s
                    and target_error >= max(1, jog_step_units // 2)
                    and abs(real.axis_state[slot].axis_velocity) <= velocity_limit
                ):
                    target_units = actual
                    requested_target_units = actual
                    blocked_direction = motion_direction
                    motion_direction = 0
                    desire.axis_ctr[slot].axis_position = actual
                    message_text = (
                        f"{arm_axis_name(slot)} limit detected; target clamped at "
                        f"{actual / units_per_rad:.5f} rad. Press opposite direction to leave."
                    )
                    set_message(message_text)
                    redraw = True

            def set_message(value: str) -> None:
                nonlocal message
                message = value

            redraw = True
            while True:
                real = real_block.value
                desire = desire_block.value
                assert real is not None and desire is not None
                update_target_trajectory()
                observe_motion_limit()
                if redraw:
                    render_arm_control(
                        real,
                        desire,
                        slot,
                        units_per_rad,
                        zero_offsets[slot],
                        target_units,
                        max_speed_rad_s,
                        message,
                    )
                    redraw = False
                key = terminal.read_key(0.02)
                if not key:
                    continue
                if key == "q":
                    disable_all_axes()
                    return f"Returned from {arm_axis_name(slot)}; arm drives disabled."
                if key == "e":
                    real = real_block.value
                    desire = desire_block.value
                    assert real is not None and desire is not None
                    if not synchronize_targets_to_feedback():
                        message = "Enable refused: waiting for valid feedback from all 14 drives."
                        continue
                    real = real_block.value
                    desire = desire_block.value
                    assert real is not None and desire is not None
                    target_units = real.axis_state[slot].axis_position
                    requested_target_units = target_units
                    last_target_update = time.monotonic()
                    desire.ec_poweron = 1
                    message = "Enable requested after synchronizing all 14 actual positions."
                    redraw = True
                    continue
                if key == "d":
                    # D on the joint page does what D does on the summary page
                    # and what the upper remote console's ARM_DISABLE does:
                    # abort the move and disable all 14 drives.
                    disable_all_axes()
                    target_units = real.axis_state[slot].axis_position
                    requested_target_units = target_units
                    motion_direction = 0
                    blocked_direction = 0
                    message = "All 14 drives disabled; any move was aborted and targets hold."
                    redraw = True
                    continue
                if key == "h":
                    if not all_axes_enabled():
                        message = "Home refused: press E and wait until all 14 drives are enabled."
                        redraw = True
                        continue
                    requested_target_units = zero_offsets[slot]
                    motion_direction = 0
                    blocked_direction = 0
                    last_target_update = time.monotonic()
                    desire.axis_ctr[slot].ec_mode = csp_mode
                    message = (
                        f"Returning {arm_axis_name(slot)} to logical zero at "
                        f"{max_speed_rad_s:.5f} rad/s."
                    )
                    redraw = True
                    continue
                if key not in {"w", "s"}:
                    continue
                if not all_axes_enabled():
                    message = "Move refused: press E and wait until all 14 drives are enabled."
                    redraw = True
                    continue
                direction = 1 if key == "w" else -1
                if blocked_direction == direction:
                    message = "Limit is active in that direction; press the opposite key first."
                    redraw = True
                    continue
                if blocked_direction == -direction:
                    blocked_direction = 0
                    requested_target_units = real.axis_state[slot].axis_position
                    target_units = requested_target_units
                elif motion_direction != 0 and motion_direction != direction:
                    requested_target_units = real.axis_state[slot].axis_position
                    target_units = requested_target_units
                requested_target_units += direction * jog_step_units
                if motion_direction != direction:
                    motion_direction = direction
                    motion_watch_position = real.axis_state[slot].axis_position
                    motion_watch_started = time.monotonic()
                desire.axis_ctr[slot].ec_mode = csp_mode
                update_target_trajectory()
                message = (
                    f"Target {requested_target_units / units_per_rad:.5f} rad; "
                    f"W=+{jog_step_rad:.5f}, S=-{jog_step_rad:.5f}"
                )
                redraw = True

        # All-axis console thresholds.  H on the main page reuses the same limits
        # and the same stall semantics as H inside a single joint, so both paths
        # behave identically; the only difference is that all 14 axes ramp at once.
        all_limit_stall_timeout_s = float(arm.get("limit_stall_timeout_s", 0.5))
        if not math.isfinite(all_limit_stall_timeout_s) or all_limit_stall_timeout_s <= 0.0:
            raise ValueError("arm.limit_stall_timeout_s must be positive and finite")
        all_limit_tolerance_units = arm_rad_to_units(
            float(arm.get("limit_position_tolerance_rad", 0.001)), units_per_rad
        )
        if all_limit_tolerance_units <= 0:
            raise ValueError("arm.limit_position_tolerance_rad must be positive")
        all_stall_velocity_limit = max(1, arm_rad_to_units(0.002, units_per_rad))
        all_enable_timeout_s = float(arm.get("enable_timeout_s", 8.0))
        if not math.isfinite(all_enable_timeout_s) or all_enable_timeout_s <= 0.0:
            raise ValueError("arm.enable_timeout_s must be positive and finite")

        with RawTerminal() as terminal:
            message = (
                "E enables all 14 drives, then H returns every joint to its logical zero.\n"
                "D disables all 14 drives and aborts any move; there is no Space key.\n"
                "C clears the remembered error codes shown with an age suffix."
            )
            task = "idle"
            task_deadline = 0.0
            last_tick = 0.0
            next_render = 0.0
            error_latch = ArmErrorLatch()
            ramp_target = {slot: 0 for slot in slots}
            ramp_requested = {slot: 0 for slot in slots}
            ramp_direction = {slot: 0 for slot in slots}
            stall_blocked = {slot: 0 for slot in slots}
            watch_position = {slot: 0 for slot in slots}
            watch_started = {slot: 0.0 for slot in slots}
            while True:
                real = real_block.value
                desire = desire_block.value
                assert real is not None and desire is not None
                selected_slot = slots[selected_index]
                now = time.monotonic()
                if task == "enabling":
                    # Glue every target to its measured position while the drives
                    # come up, so enabling itself can never command a movement.
                    hold_all_axes()
                    if all_axes_enabled():
                        task = "idle"
                        message = (
                            "All 14 drives are enabled. Press H to send every joint to its logical "
                            "zero, or Enter to control one joint."
                        )
                        next_render = 0.0
                    elif now >= task_deadline:
                        enabled, faults, modes_ok, total = arm_status(real, slots)
                        task = "idle"
                        message = (
                            f"Enable timed out after {all_enable_timeout_s:.1f} s: {enabled}/{total} enabled, "
                            f"{faults} faulted, {modes_ok}/{total} in CSP mode. Press D to disable all."
                        )
                        next_render = 0.0
                elif task == "homing":
                    enabled, faults, modes_ok, total = arm_status(real, slots)
                    if enabled != total or faults != 0:
                        hold_all_axes()
                        task = "idle"
                        message = (
                            f"Homing aborted: {enabled}/{total} enabled, {faults} faulted. Every target "
                            "was frozen at its measured position."
                        )
                        next_render = 0.0
                    else:
                        elapsed = min(max(now - last_tick, 0.0), 0.1)
                        max_delta_units = max_speed_units_s * elapsed
                        for slot in slots:
                            axis = real.axis_state[slot]
                            if stall_blocked[slot] != 0:
                                continue
                            ramp_target[slot] = arm_ramp_target(
                                ramp_target[slot], ramp_requested[slot], max_delta_units
                            )
                            desire.axis_ctr[slot].ec_mode = csp_mode
                            desire.axis_ctr[slot].axis_position = ramp_target[slot]
                            desire.axis_ctr[slot].axis_velocity = 0
                            desire.axis_ctr[slot].axis_effort = 0
                            direction = ramp_direction[slot]
                            if direction == 0:
                                continue
                            if (
                                (axis.axis_position - watch_position[slot]) * direction
                                >= all_limit_tolerance_units
                            ):
                                watch_position[slot] = axis.axis_position
                                watch_started[slot] = now
                                continue
                            target_error = (ramp_requested[slot] - axis.axis_position) * direction
                            if (
                                now - watch_started[slot] >= all_limit_stall_timeout_s
                                and target_error >= max(1, jog_step_units // 2)
                                and abs(axis.axis_velocity) <= all_stall_velocity_limit
                            ):
                                ramp_target[slot] = axis.axis_position
                                ramp_requested[slot] = axis.axis_position
                                stall_blocked[slot] = direction
                                desire.axis_ctr[slot].axis_position = axis.axis_position
                                message = (
                                    f"{arm_axis_name(slot)} stalled at "
                                    f"{arm_logical_position(axis.axis_position, zero_offsets[slot], units_per_rad): .5f} rad "
                                    "and is now held; the other joints keep homing. Press D to disable all."
                                )
                                next_render = 0.0
                        last_tick = now
                        settled = all(
                            ramp_target[slot] == ramp_requested[slot]
                            and abs(real.axis_state[slot].axis_position - ramp_requested[slot])
                            <= all_limit_tolerance_units
                            for slot in slots
                        )
                        if settled:
                            stalled = [
                                arm_axis_name(slot) for slot in slots if stall_blocked[slot] != 0
                            ]
                            task = "idle"
                            message = (
                                "All 14 joints are back at their logical zero."
                                if not stalled
                                else "Homing stopped with these joints short of zero: " + ", ".join(stalled)
                            )
                            next_render = 0.0
                        elif now >= task_deadline:
                            hold_all_axes()
                            for slot in slots:
                                ramp_target[slot] = real.axis_state[slot].axis_position
                                ramp_requested[slot] = ramp_target[slot]
                            task = "idle"
                            message = (
                                "Homing timed out; every target was frozen at its measured position. "
                                "Press D to disable all."
                            )
                            next_render = 0.0
                if task == "homing":
                    at_zero = sum(
                        1
                        for slot in slots
                        if abs(real.axis_state[slot].axis_position - zero_offsets[slot])
                        <= all_limit_tolerance_units
                    )
                    furthest_units = max(
                        abs(zero_offsets[slot] - real.axis_state[slot].axis_position) for slot in slots
                    )
                    shown_message = (
                        f"{message}\n{at_zero}/{len(slots)} joints at zero; furthest joint is still "
                        f"{furthest_units / units_per_rad:.5f} rad away."
                    )
                elif task == "enabling":
                    enabled, _faults, _modes, total = arm_status(real, slots)
                    shown_message = f"{message}\n{enabled}/{total} drives enabled so far."
                else:
                    shown_message = message
                if task == "idle":
                    # Nothing is moving, so a blocking read keeps the console idle.
                    render_arm_selection(real, slots, selected_slot, units_per_rad, zero_offsets, shown_message, error_latch, bus_monitor.snapshot())
                    key = terminal.read_key()
                else:
                    # A task is running: tick the trajectory at 50 Hz and redraw at 10 Hz.
                    if now >= next_render:
                        render_arm_selection(real, slots, selected_slot, units_per_rad, zero_offsets, shown_message, error_latch, bus_monitor.snapshot())
                        next_render = now + 0.1
                    key = terminal.read_key(0.02)
                if key == "up":
                    selected_index = (selected_index - 1) % len(slots)
                elif key == "down":
                    selected_index = (selected_index + 1) % len(slots)
                elif key == "c":
                    message = f"Latched error memory cleared ({error_latch.clear()} joint(s))."
                elif key == "enter":
                    if task == "idle":
                        message = control_selected_joint(terminal, selected_slot)
                    else:
                        message = "Busy: press D to disable all 14 drives and abort the current task."
                elif key == "e":
                    if task != "idle":
                        message = "Busy: press D to disable all 14 drives and abort the current task."
                    elif synchronize_targets_to_feedback():
                        desire = desire_block.value
                        assert desire is not None
                        desire.ec_poweron = 1
                        task = "enabling"
                        task_deadline = now + all_enable_timeout_s
                        message = (
                            "Enabling all 14 drives; every target stays at its measured position until "
                            "they are all enabled."
                        )
                    else:
                        message = "Enable refused: waiting for valid feedback from all 14 drives."
                elif key == "d":
                    task = "idle"
                    disable_all_axes()
                    message = "All 14 drives are disabled; any move was aborted and targets hold."
                elif key == "h":
                    if task != "idle":
                        message = "Busy: press D to disable all 14 drives and abort the current task."
                    elif not all_axes_enabled():
                        message = "Home refused: press E and wait until all 14 drives are enabled."
                    else:
                        for slot in slots:
                            ramp_target[slot] = real.axis_state[slot].axis_position
                            ramp_requested[slot] = zero_offsets[slot]
                            direction = 0
                            if ramp_requested[slot] > ramp_target[slot]:
                                direction = 1
                            elif ramp_requested[slot] < ramp_target[slot]:
                                direction = -1
                            ramp_direction[slot] = direction
                            stall_blocked[slot] = 0
                            watch_position[slot] = ramp_target[slot]
                            watch_started[slot] = now
                        longest_rad = max(
                            abs(zero_offsets[slot] - ramp_target[slot]) for slot in slots
                        ) / units_per_rad
                        task = "homing"
                        last_tick = now
                        task_deadline = now + max(30.0, longest_rad / max_speed_rad_s * 1.5 + 10.0)
                        message = (
                            f"Homing all 14 joints to their logical zero at {max_speed_rad_s:.5f} rad/s "
                            f"(longest travel {longest_rad:.5f} rad). D aborts and disables all."
                        )
                elif key == "q":
                    task = "idle"
                    disable_all_axes()
                    break
        return 0
    finally:
        # Drop the ctypes views this frame still references (the homing branch
        # keeps `axis` alive) so the mappings can really be closed, then hand
        # every remaining release step to a helper that cannot raise.
        axis = None
        desire = None
        real = None
        if bus_monitor is not None:
            bus_monitor.stop()
        for warning in release_arm_test_resources(
            desire_block=desire_block,
            real_block=real_block,
            driver_process=driver_process,
            log_file=log_file,
            disable_settle_s=float(arm.get("disable_settle_s", 1.0)),
        ):
            print(f"WARNING: {warning}", file=sys.stderr)


def release_arm_test_resources(
    *,
    desire_block: "SharedMemory",
    real_block: "SharedMemory",
    driver_process: subprocess.Popen[bytes],
    log_file: Any,
    disable_settle_s: float,
) -> list[str]:
    """Release everything run_arm_test owns; never raise.

    Order matters and each step is independently guarded.  A failure while
    closing the shared-memory mappings must never skip stopping the IGH
    driver: a leftover driver keeps ``/dev/EtherCAT*`` open, which blocks the
    non-ROS TCP server ("hardware is already active") and the next console run.
    Returns the warnings to print; the caller reports them after cleanup.
    """
    warnings: list[str] = []
    try:
        if desire_block.value is not None:
            desire_block.value.ec_poweron = 0
        time.sleep(max(0.0, float(disable_settle_s)))
    except Exception as exc:  # noqa: BLE001 - cleanup must continue
        warnings.append(f"power-off request failed: {exc}")
    for name, block in (("desire", desire_block), ("real", real_block)):
        try:
            block.close()
        except Exception as exc:  # noqa: BLE001 - cleanup must continue
            warnings.append(f"{name} shared-memory close failed: {exc}")
    if driver_process.poll() is None:
        driver_process.send_signal(signal.SIGINT)
        try:
            driver_process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            warnings.append("igh_driver did not stop within 3 s of SIGINT; killing it")
            driver_process.kill()
            try:
                driver_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                warnings.append("igh_driver survived SIGKILL; it may still hold /dev/EtherCAT*")
    try:
        log_file.close()
    except OSError as exc:
        warnings.append(f"driver log close failed: {exc}")
    return warnings


def run_lift_test(config: dict[str, Any], config_path: Path) -> int:
    """Run the unified lift console (W/S/D/E/H/Z/Q) over the EtherCAT CLI.

    The console keeps the control logic in one script on both machines and
    drives `lift_ethercat_cli --command-mode`; the CLI stays the single owner
    of EtherCAT and of the CiA 402 state machine.  See lift_console.py.
    """
    from lift_console import build_parser, run_console

    args = build_parser().parse_args(["--config", str(config_path)])
    return run_console(args)


def self_test(config_path: Path) -> int:
    config = load_config(config_path)
    mapping(config)
    expected = {
        "AxisCommand": 20,
        "AxisFeedback": 28,
        "DesireRegion": 480,
        "RealRegion": 648,
    }
    actual = {
        "AxisCommand": ctypes.sizeof(AxisCommand),
        "AxisFeedback": ctypes.sizeof(AxisFeedback),
        "DesireRegion": ctypes.sizeof(DesireRegion),
        "RealRegion": ctypes.sizeof(RealRegion),
    }
    if actual != expected:
        raise AssertionError(f"shared-memory ABI mismatch: expected {expected}, got {actual}")
    slots = config["arm"]["axis_slots"]
    combined = list(slots["left_arm"]) + list(slots["right_arm"])
    if combined != list(range(14)):
        raise AssertionError(f"arm axis slots must cover 0..13: {combined}")
    print("PASS: EtherCAT mapping and shared-memory ABI checks")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG_PATH)
    parser.add_argument(
        "action", choices=("check", "check-driver", "configure", "arm", "lift", "self-test")
    )
    args = parser.parse_args()
    config = load_config(args.config)
    if args.action == "check":
        return check_setup(config)
    if args.action == "check-driver":
        return check_driver(config, args.config)
    if args.action == "configure":
        return configure_ethercat(config)
    if args.action == "self-test":
        return self_test(args.config)
    if args.action == "arm":
        return run_arm_test(config, args.config)
    return run_lift_test(config, args.config)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
