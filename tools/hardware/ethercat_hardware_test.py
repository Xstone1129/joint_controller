#!/usr/bin/env python3
"""ROS-free EtherCAT setup and heavy_v1 enable test console."""

from __future__ import annotations

import argparse
import ctypes
import math
import mmap
import os
from pathlib import Path
import pwd
import re
import select
import signal
import shutil
import subprocess
import sys
import termios
import time
from typing import Any
import tty

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
    if log_root:
        root = Path(log_root)
        base = root / base.name
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
        self.value = None
        if self.file_map is not None:
            self.file_map.close()
            self.file_map = None
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


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
    return {
        0x08: "fault",
        0x21: "ready",
        0x23: "switched_on",
        0x27: "operation_enabled",
        0x40: "switch_on_disabled",
    }.get(status & 0x006F, f"0x{status:04X}")


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
) -> None:
    print("\033[2J\033[H", end="")
    print("Arm EtherCAT - select joint (Up/Down, Enter, Q)")
    print("No.  Joint      Position(rad)  State                 Error")
    for slot in slots:
        axis = real.axis_state[slot]
        line = (
            f"{slot + 1:02d}.  {arm_axis_name(slot):8s}  "
            f"{arm_logical_position(axis.axis_position, zero_offsets[slot], units_per_rad): .5f}       "
            f"{state_name(axis.ec_ctrstate):18s}  0x{axis.axis_error_code:04X}"
        )
        if slot == selected_slot:
            line = f"\033[7m> {line}\033[0m"
        else:
            line = f"  {line}"
        print(line)
    print(f"\n{message}")


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
    axis = real.axis_state[slot]
    print("\033[2J\033[H", end="")
    print(f"Arm EtherCAT - control {arm_axis_name(slot)} (slot {slot})")
    print(f"Actual position : {arm_logical_position(axis.axis_position, zero_units, units_per_rad): .5f} rad")
    print(f"Target position : {arm_logical_position(target_units, zero_units, units_per_rad): .5f} rad")
    print(f"Zero offset     : {zero_units} units")
    print(f"Drive state     : {state_name(axis.ec_ctrstate)} / mode 0x{axis.ec_modestate:02X}")
    print(f"Power request   : {'ON' if desire.ec_poweron else 'OFF'}")
    print(f"Max speed      : {max_speed_rad_s:.5f} rad/s")
    print("\nE enable | W +step | S -step | H return zero | Z mark zero")
    print("Space stop | Q back")
    print(message)


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
    log_path = allocate_driver_log(arm.get("driver_log", "/tmp/heavy_v1_igh_driver.log"))
    log_file = log_path.open("wb")
    env = os.environ.copy()
    library_dir = str(config.get("ethercat", {}).get("library_dir", "/usr/local/etherlab/lib"))
    env["LD_LIBRARY_PATH"] = library_dir + ":" + env.get("LD_LIBRARY_PATH", "")
    driver_process = subprocess.Popen([str(driver)], stdout=log_file, stderr=subprocess.STDOUT, env=env)
    desire_block = SharedMemory(str(arm["shared_memory"]["desire"]), DesireRegion)
    real_block = SharedMemory(str(arm["shared_memory"]["real"]), RealRegion)
    real: RealRegion | None = None
    desire: DesireRegion | None = None
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if driver_process.poll() is not None:
                raise RuntimeError(f"igh_driver exited with code {driver_process.returncode}; see {log_path}")
            if desire_block.path.exists() and real_block.path.exists():
                desire_block.open()
                real_block.open()
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(f"driver did not create shared memory; see {log_path}")

        slots = list(arm["axis_slots"]["left_arm"]) + list(arm["axis_slots"]["right_arm"])
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
                if key == " ":
                    hold_all_axes()
                    target_units = real.axis_state[slot].axis_position
                    requested_target_units = target_units
                    motion_direction = 0
                    blocked_direction = 0
                    message = "Stopped and holding measured position."
                    redraw = True
                    continue
                if key == "z":
                    motion_direction = 0
                    hold_all_axes()
                    real = real_block.value
                    desire = desire_block.value
                    assert real is not None and desire is not None
                    zero_offsets[slot] = real.axis_state[slot].axis_position
                    target_units = zero_offsets[slot]
                    try:
                        save_arm_zero_offsets(config, config_path, zero_offsets)
                    except (OSError, ValueError) as exc:
                        message = f"Zero not saved: {exc}"
                    else:
                        message = f"{arm_axis_name(slot)} zero saved at current position."
                    target_units = real.axis_state[slot].axis_position
                    requested_target_units = target_units
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

        with RawTerminal() as terminal:
            message = "Use Up/Down to select one of 14 joints, Enter to control, Q to quit."
            while True:
                real = real_block.value
                desire = desire_block.value
                assert real is not None and desire is not None
                selected_slot = slots[selected_index]
                render_arm_selection(real, slots, selected_slot, units_per_rad, zero_offsets, message)
                key = terminal.read_key()
                if key == "up":
                    selected_index = (selected_index - 1) % len(slots)
                elif key == "down":
                    selected_index = (selected_index + 1) % len(slots)
                elif key == "enter":
                    message = control_selected_joint(terminal, selected_slot)
                elif key == "q":
                    disable_all_axes()
                    break
        return 0
    finally:
        if desire_block.value is not None:
            desire_block.value.ec_poweron = 0
        time.sleep(float(arm.get("disable_settle_s", 1.0)))
        desire = None
        real = None
        desire_block.close()
        real_block.close()
        if driver_process.poll() is None:
            driver_process.send_signal(signal.SIGINT)
            try:
                driver_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                driver_process.kill()
        log_file.close()


def run_lift_test(config: dict[str, Any], config_path: Path) -> int:
    lift = config["lift"]
    binary = (config_path.parent / str(lift["cli_binary"])).resolve()
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise RuntimeError(f"lift CLI binary was not found or is not executable: {binary}")

    owner_name = os.environ.get("SUDO_USER") or os.environ.get("USER")
    try:
        owner = pwd.getpwnam(owner_name) if owner_name else pwd.getpwuid(os.getuid())
    except KeyError:
        owner = pwd.getpwuid(os.getuid())
    zero_offset_file = (
        Path(owner.pw_dir) / ".local/state/joint_controller/lift_zero_offset.cfg"
    )
    command = [
        str(binary),
        "--interface", str(lift["interface"]),
        "--master", str(lift["master"]),
        "--alias", str(lift["slave_alias"]),
        "--position", str(lift["slave_position"]),
        "--min-position", str(lift["min_position_m"]),
        "--max-position", str(lift["max_position_m"]),
        "--zero-offset-file", str(zero_offset_file),
        "--zero-offset-uid", str(owner.pw_uid),
        "--zero-offset-gid", str(owner.pw_gid),
    ]
    env = os.environ.copy()
    library_dir = str(config.get("ethercat", {}).get("library_dir", "/usr/local/etherlab/lib"))
    env["LD_LIBRARY_PATH"] = library_dir + ":" + env.get("LD_LIBRARY_PATH", "")
    print("Lift test uses the standalone CSV EtherCAT CLI. It is ROS-free.")
    return subprocess.run(
        command,
        cwd=config_path.parent.parent.parent,
        check=False,
        env=env,
    ).returncode


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
