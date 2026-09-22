#!/usr/bin/env python3
"""Interactive lift console for the standalone EtherCAT lift test.

This is the interactive front end of `tools/hardware/run_hardware_console.sh
lift`.  It keeps the control logic people actually use -- W/S step, D disable,
E enable, H home, Z zero, Q back -- in one script and drives the existing
`tools/lift_ethercat_cli` binary through its `--command-mode` line protocol.
The C++ binary stays the single owner of EtherCAT, the 100 Hz PDO loop and the
CiA 402 state machine; this console never opens the bus itself.

The page layout, the key set and the wording are copied from the upper-machine
remote console (junior_ws/tools/hardware/remote/remote_console.py) so that the
lift page is identical on both machines: both print
`Backend/Enabled/Height/Velocity/Drive/Error` and the footer
`E ENABLE  D DISABLE  H HOME  Z ZERO  W UP  S DOWN  Q BACK`.

The previous interactive UI of `lift_ethercat_cli` used `u/j` for hold-to-jog
and the arrow keys for a +/-100 mm position task, which gave two different
motion meanings to two similar key groups.  That UI is no longer used by the
launcher; this console is the only interactive lift page.
"""

from __future__ import annotations

import argparse
import os
import pwd
import select
import subprocess
import sys
import termios
import time
import tty
import unicodedata
from pathlib import Path
from typing import Any

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR / "hardware_io.yaml"

# The CLI needs its EtherLab shared library; run_hardware_console.sh already
# exports this for the whole session, and the fallback keeps a direct call
# working as well.
DEFAULT_ETHERLAB_LIB = "/usr/local/etherlab/lib"

READY_TIMEOUT_S = 30.0
FIRST_SAMPLE_TIMEOUT_S = 12.0
STATE_PERIOD_S = 0.3
RENDER_PERIOD_S = 0.1
KEY_TIMEOUT_S = 0.1
STOP_VELOCITY_MPS = 0.001


# ---------------------------------------------------------------------------
# Shared page vocabulary -- keep byte-identical with the upper remote console.
# ---------------------------------------------------------------------------
LIFT_ERROR_NAMES = {
    0x0000: "无错误",
    0x2310: "过流",
    0x3110: "主电源过压",
    0x3120: "主电源欠压",
    0x3210: "母线过压",
    0x3220: "母线欠压",
    0x4210: "过温",
    0x7300: "编码器故障",
}

UNKNOWN_ERROR_NAME = "未知故障码"

# CiA 402 status words come from one generated table so the console, the
# runtime (cia402.cpp) and the lift_ethercat_cli binary cannot drift apart.
# Regenerate with: python3 tools/hardware/generate_lift_console_states.py
from lift_console_states import CIA402_STATE_MASK, CIA402_STATE_NAMES

LIFT_KEY_FOOTER = "E ENABLE  D DISABLE  H HOME  Z ZERO  W UP  S DOWN  Q BACK"


def display_width(text: str) -> int:
    """Return the terminal cell width of text (CJK glyphs occupy two cells)."""
    return sum(2 if unicodedata.east_asian_width(char) in "WF" else 1 for char in text)


def lift_error_text(error: Any) -> str:
    code = int(error) & 0xFFFF
    return f"0x{code:04X} ({LIFT_ERROR_NAMES.get(code, UNKNOWN_ERROR_NAME)})"


def cia402_state_name(status_word: int) -> str:
    return CIA402_STATE_NAMES.get(int(status_word) & CIA402_STATE_MASK, "unknown")


def render_lift_state(
    state: dict[str, Any],
    enabled: bool | None,
    message: str = "",
    backend: str = "real",
) -> None:
    """Print the unified lift page.  Mirrors the upper remote console."""
    print("\033[2J\033[H", end="")
    print("LIFT")
    print(f"Backend : {backend}")
    print(f"Enabled : {bool(enabled)}")
    position = state.get("position_m")
    velocity = state.get("velocity_mps")
    feedback_available = state.get("feedback_available") is True
    pdo_fresh = state.get("pdo_fresh") is True
    if isinstance(position, (int, float)) and feedback_available and pdo_fresh:
        print(f"Height  : {float(position): .6f} m (measured)")
    elif isinstance(position, (int, float)):
        print(f"Height  : {float(position): .6f} m (stale feedback)")
    else:
        print("Height  : unavailable (waiting for fresh PDO feedback)")
    if isinstance(velocity, (int, float)):
        print(f"Velocity: {float(velocity): .6f} m/s")
    status_word = state.get("status_word")
    if isinstance(status_word, int):
        link_state = str(state.get("link_state", "unknown"))
        print(f"Drive   : {cia402_state_name(status_word)}  PDO: {'fresh' if pdo_fresh else 'stale'}  link: {link_state}")
    error_code = state.get("error_code")
    if isinstance(error_code, int):
        print(f"Error   : {lift_error_text(error_code)}")
    print(f"\n{LIFT_KEY_FOOTER}")
    if message:
        print(message)
    sys.stdout.flush()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"configuration must be a mapping: {path}")
    return data


def resolve_cli_binary(config_path: Path, lift_config: dict[str, Any]) -> Path:
    value = str(lift_config.get("cli_binary", "")).strip()
    if not value:
        raise ValueError("lift.cli_binary must not be empty")
    path = Path(value).expanduser()
    return path if path.is_absolute() else (config_path.parent / path).resolve()


def resolve_zero_offset_file(lift_config: dict[str, Any]) -> Path:
    value = str(lift_config.get("zero_offset_file", "")).strip()
    if value:
        return Path(value).expanduser()
    owner = _invoking_user()
    return Path(owner.pw_dir) / ".local/state/joint_controller/lift_zero_offset.cfg"


def _invoking_user() -> pwd.struct_passwd:
    """Return the account that will own files written by the root console."""
    for name in (os.environ.get("SUDO_USER"), os.environ.get("USER")):
        if name:
            try:
                return pwd.getpwnam(name)
            except KeyError:
                continue
    return pwd.getpwuid(os.getuid())


def build_cli_command(
    binary: Path,
    config_path: Path,
    lift_config: dict[str, Any],
    min_position_m: float,
    max_position_m: float,
    speed_mps: float,
) -> list[str]:
    owner = _invoking_user()
    return [
        str(binary),
        "--command-mode",
        "--interface", str(lift_config["interface"]),
        "--master", str(lift_config["master"]),
        "--alias", str(lift_config["slave_alias"]),
        "--position", str(lift_config["slave_position"]),
        "--min-position", str(min_position_m),
        "--max-position", str(max_position_m),
        "--speed", str(speed_mps),
        "--accel", str(lift_config.get("acceleration_mps2", 0.033333333)),
        "--zero-offset-file", str(resolve_zero_offset_file(lift_config)),
        "--zero-offset-uid", str(owner.pw_uid),
        "--zero-offset-gid", str(owner.pw_gid),
    ]


class RawTerminal:
    """cbreak terminal that returns normalised key names.

    The arrow keys are accepted as an alias of W/S so a habit of pressing them
    still works, but they no longer carry a separate meaning: the documented
    and displayed logic is W/S/D/E/H/Z/Q only.
    """

    def __init__(self) -> None:
        self.fd = -1
        self.settings: list[Any] | None = None

    def __enter__(self) -> "RawTerminal":
        if not sys.stdin.isatty():
            raise ValueError("interactive terminal required")
        self.fd = sys.stdin.fileno()
        self.settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_args: object) -> None:
        if self.fd >= 0 and self.settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)

    def read_key(self, timeout_s: float) -> str:
        if not select.select([self.fd], [], [], timeout_s)[0]:
            return ""
        first = os.read(self.fd, 1).decode("utf-8", errors="replace")
        if first == "\x1b":
            sequence = bytearray(first.encode())
            deadline = time.monotonic() + 0.12
            while len(sequence) < 8 and time.monotonic() < deadline:
                remaining = max(0.0, deadline - time.monotonic())
                if not select.select([self.fd], [], [], remaining)[0]:
                    break
                sequence.extend(os.read(self.fd, 1))
                if sequence[-1:] in b"ABCD~":
                    break
            return {
                b"\x1b[A": "w",
                b"\x1bOA": "w",
                b"\x1b[B": "s",
                b"\x1bOB": "s",
            }.get(bytes(sequence), "")
        if first in {"\r", "\n"}:
            return "enter"
        return first.lower()


class LiftConsole:
    def __init__(self, args: argparse.Namespace) -> None:
        self.config_path = Path(args.config).resolve()
        if not self.config_path.is_file():
            raise ValueError(f"configuration not found: {self.config_path}")
        config = load_yaml(self.config_path)
        lift_config = config.get("lift")
        if not isinstance(lift_config, dict):
            raise ValueError(f"configuration must contain a lift mapping: {self.config_path}")
        self.lift_config = lift_config
        self.etherlab_lib = str(
            config.get("ethercat", {}).get("library_dir", DEFAULT_ETHERLAB_LIB)
        )
        self.binary = resolve_cli_binary(self.config_path, lift_config)

        self.min_position_m = _finite_float(
            args.min_position_m
            if args.min_position_m is not None
            else lift_config.get("min_position_m", -1.0),
            "lift min position",
        )
        self.max_position_m = _finite_float(
            args.max_position_m
            if args.max_position_m is not None
            else lift_config.get("max_position_m", 1.0),
            "lift max position",
        )
        if self.min_position_m >= self.max_position_m:
            raise ValueError("lift min position must be less than the max position")
        self.speed_mps = _finite_float(
            args.speed_mps
            if args.speed_mps is not None
            else lift_config.get("max_speed_mps", 0.015),
            "lift speed",
        )
        if self.speed_mps <= 0.0:
            raise ValueError("lift speed must be positive")
        self.jog_step_m = _finite_float(
            args.jog_step_m
            if args.jog_step_m is not None
            else lift_config.get("jog_step_m", 0.01),
            "lift jog step",
        )
        self.max_step_m = _finite_float(lift_config.get("max_step_m", 0.1), "lift max step")
        if not 0.0 < self.jog_step_m <= self.max_step_m:
            raise ValueError("lift jog step must be positive and no larger than max_step_m")

        self.proc: subprocess.Popen[bytes] | None = None
        self.pending = ""
        self.state: dict[str, Any] = {"feedback_available": False}
        self.enabled = False
        self.message = ""
        self.fault_latched = False

    # -- process plumbing ---------------------------------------------------
    def start(self) -> None:
        if not self.binary.is_file() or not os.access(self.binary, os.X_OK):
            raise RuntimeError(f"lift CLI binary was not found or is not executable: {self.binary}")
        command = build_cli_command(
            self.binary,
            self.config_path,
            self.lift_config,
            self.min_position_m,
            self.max_position_m,
            self.speed_mps,
        )
        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = self.etherlab_lib + ":" + env.get("LD_LIBRARY_PATH", "")
        print("正在启动 lift EtherCAT CLI（command mode），等待主站与从站就绪...")
        self.proc = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
        )
        if self.proc.stdout is None:
            raise RuntimeError("lift CLI stdout is unavailable")
        os.set_blocking(self.proc.stdout.fileno(), False)
        self._await_ready()

    def _read_line(self) -> str | None:
        """Return the next complete CLI stdout line, or None when none is ready."""
        if self.proc is None or self.proc.stdout is None:
            return None
        while "\n" not in self.pending:
            try:
                raw = self.proc.stdout.readline()
            except BlockingIOError:
                return None
            if not raw:
                return None
            self.pending += raw.decode("utf-8", errors="replace")
        line, self.pending = self.pending.split("\n", 1)
        return line.strip()

    def _await_ready(self) -> None:
        assert self.proc is not None
        deadline = time.monotonic() + READY_TIMEOUT_S
        startup_lines: list[str] = []
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                detail = "; ".join(item for item in startup_lines if item)
                raise RuntimeError(
                    f"lift CLI exited with code {self.proc.returncode}"
                    + (f": {detail}" if detail else "")
                )
            line = self._read_line()
            if line is None:
                time.sleep(0.02)
                continue
            startup_lines.append(line)
            del startup_lines[:-8]
            if "COMMAND_MODE_READY" in line:
                break
        else:
            detail = "; ".join(item for item in startup_lines if item)
            raise RuntimeError(
                f"lift CLI did not become ready within {READY_TIMEOUT_S:.0f} seconds"
                + (f": {detail}" if detail else "")
            )
        # Readiness gate: keep polling STATE until the drive reports its first
        # usable sample instead of sleeping a fixed time and pretending.
        first_sample_deadline = time.monotonic() + FIRST_SAMPLE_TIMEOUT_S
        while time.monotonic() < first_sample_deadline:
            self.send("STATE")
            deadline_sample = time.monotonic() + 0.5
            while time.monotonic() < deadline_sample:
                if self.drain_lines():
                    break
                if self.state.get("feedback_available"):
                    return
                time.sleep(0.01)
            if self.state.get("feedback_available"):
                return
        self.message = "No fresh PDO sample yet; the page will keep waiting for feedback."

    def send(self, command: str) -> None:
        if self.proc is None or self.proc.stdin is None:
            return
        try:
            self.proc.stdin.write((command + "\n").encode("ascii"))
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            self.message = f"CLI write failed: {exc}"

    def drain_lines(self) -> int:
        """Consume every available CLI line; return how many were consumed."""
        handled = 0
        while True:
            line = self._read_line()
            if line is None:
                return handled
            handled += 1
            if line.startswith("LIFT_STATE "):
                self._apply_state_line(line)
            elif line:
                self.message = f"CLI: {line}"

    def _apply_state_line(self, line: str) -> None:
        fields: dict[str, str] = {}
        for token in line.split()[1:]:
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value
        if fields.get("available") != "1":
            self.state = {"feedback_available": False}
            return
        try:
            status_word = int(fields.get("status_word", "0"), 0)
            error_code = int(fields.get("error_code", "0"), 0)
            self.state = {
                "feedback_available": True,
                "position_m": float(fields["position_m"]),
                "velocity_mps": float(fields["velocity_mps"]),
                "pdo_fresh": fields.get("pdo_fresh") == "1",
                "link_state": fields.get("link_state", "unknown"),
                "status_word": status_word,
                "error_code": error_code,
            }
        except (KeyError, ValueError):
            self.state = {"feedback_available": False, "feedback_error": "invalid lift state line"}
            return
        if error_code != 0 and not self.fault_latched:
            self.fault_latched = True
            self.enabled = False
            self.message = (
                f"Drive fault {lift_error_text(error_code)} latched; motion is blocked. "
                "Press E to retry after inspection."
            )

    # -- actions -----------------------------------------------------------
    def _health_reason(self) -> str | None:
        if not self.state.get("feedback_available"):
            return "waiting for a fresh PDO sample"
        if self.state.get("pdo_fresh") is not True:
            return "PDO feedback is stale"
        if str(self.state.get("link_state", "unknown")) != "operational":
            return f"EtherCAT link is {self.state.get('link_state', 'unknown')}"
        error_code = self.state.get("error_code")
        if isinstance(error_code, int) and error_code != 0:
            return f"drive fault {lift_error_text(error_code)}"
        return None

    def enable(self) -> None:
        self.send("ENABLE")
        self.fault_latched = False
        self.enabled = True
        self.message = "Enable requested; holding the measured position."

    def disable(self) -> None:
        self.send("DISABLE")
        self.enabled = False
        self.message = "Disable requested; the lift holds at its measured position."

    def home(self) -> None:
        reason = self._health_reason()
        if reason is not None:
            self.message = f"Home refused: {reason}."
            return
        self.send("HOME")
        self.enabled = True
        self.message = "Homing to logical zero at the configured speed."

    def zero(self) -> None:
        reason = self._health_reason()
        if reason is not None:
            self.message = f"Zero refused: {reason}."
            return
        velocity = self.state.get("velocity_mps")
        if isinstance(velocity, (int, float)) and abs(float(velocity)) > STOP_VELOCITY_MPS:
            self.message = (
                f"Zero refused: the lift must be stationary (velocity={float(velocity): .6f} m/s)."
            )
            return
        self.send("ZERO")
        self.enabled = False
        self.message = "Zero requested: controlled disable, then the current position is saved as zero."

    def jog(self, direction: int) -> None:
        reason = self._health_reason()
        if reason is not None:
            self.message = f"Jog refused: {reason}."
            return
        if not self.enabled:
            self.message = "Enable the lift before jogging."
            return
        self.send(f"MOVE_STEP {direction * self.jog_step_m:.9f}")
        self.message = (
            f"Jog {'up' if direction > 0 else 'down'} {self.jog_step_m:.4f} m "
            f"(W=+{self.jog_step_m:.4f}, S=-{self.jog_step_m:.4f})"
        )

    def handle_key(self, key: str) -> bool:
        """Handle one key; return False when the console should exit."""
        if key in {"q", "\x03"}:
            return False
        if key == "e":
            self.enable()
        elif key == "d":
            self.disable()
        elif key == "h":
            self.home()
        elif key == "z":
            self.zero()
        elif key == "w":
            self.jog(1)
        elif key == "s":
            self.jog(-1)
        return True

    # -- main loop ---------------------------------------------------------
    def run(self) -> int:
        self.start()
        next_state_poll = 0.0
        next_render = 0.0
        try:
            with RawTerminal() as terminal:
                while True:
                    now = time.monotonic()
                    self.drain_lines()
                    if self.proc is not None and self.proc.poll() is not None:
                        self.message = f"CLI exited with code {self.proc.returncode}."
                        return 1
                    if now >= next_state_poll:
                        self.send("STATE")
                        next_state_poll = now + STATE_PERIOD_S
                    if now >= next_render:
                        render_lift_state(self.state, self.enabled, self.message)
                        next_render = now + RENDER_PERIOD_S
                    key = terminal.read_key(KEY_TIMEOUT_S)
                    if not key:
                        continue
                    if not self.handle_key(key):
                        return 0
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        proc = self.proc
        self.proc = None
        if proc is None:
            return
        try:
            if proc.stdin is not None:
                proc.stdin.write(b"HOLD\nQUIT\n")
                proc.stdin.flush()
        except OSError:
            pass
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        for stream in (proc.stdin, proc.stdout):
            if stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass


def _finite_float(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a number: {value!r}") from exc
    if result != result or result in (float("inf"), float("-inf")):
        raise ValueError(f"{label} must be finite: {value!r}")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--jog-step-m", type=float, help="W/S step in m (default tools config)")
    parser.add_argument("--speed-mps", type=float, help="motion speed limit in m/s")
    parser.add_argument("--min-position-m", type=float, help="lower travel limit in m")
    parser.add_argument("--max-position-m", type=float, help="upper travel limit in m")
    return parser


def run_console(args: argparse.Namespace, console_type: type[LiftConsole] = LiftConsole) -> int:
    if os.geteuid() != 0:
        print("Lift EtherCAT console requires root; use sudo.", file=sys.stderr)
        return 1
    try:
        console = console_type(args)
    except (OSError, ValueError) as exc:
        print(f"lift console configuration error: {exc}", file=sys.stderr)
        return 2
    try:
        return console.run()
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Lift console failed: {exc}", file=sys.stderr)
        return 1
    finally:
        print("\033[0m\nLift console exited; the lift held its position and the CLI stopped.")


def main(argv: list[str] | None = None) -> int:
    return run_console(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
