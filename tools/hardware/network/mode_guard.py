"""Cross-process ROS/Direct hardware ownership guard."""

from __future__ import annotations

import fcntl
import os
from pathlib import Path
import subprocess
from typing import Iterable


class HardwareBusy(RuntimeError):
    pass


class HardwareActivityProbe:
    """Detect hardware users which may predate the shared ownership lock."""

    def __init__(
        self,
        *,
        service_name: str = "joint-controller-stack-real.service",
        pid_file: Path = Path("/var/run/igh_driver.pid"),
        proc_root: Path = Path("/proc"),
        device_root: Path = Path("/dev"),
    ) -> None:
        self.service_name = service_name
        self.pid_file = pid_file
        self.proc_root = proc_root
        self.device_root = device_root

    @staticmethod
    def _pid_alive(pid: int) -> bool:
        if pid <= 1:
            return False
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _service_active(self) -> bool:
        try:
            result = subprocess.run(
                ["systemctl", "is-active", "--quiet", self.service_name],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
                timeout=1.0,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False
        return result.returncode == 0

    def _hardware_tool_pids(self) -> set[int]:
        pids: set[int] = set()
        try:
            pid = int(self.pid_file.read_text(encoding="ascii").strip())
            if self._pid_alive(pid):
                pids.add(pid)
        except (OSError, ValueError):
            pass

        try:
            proc_entries: Iterable[Path] = self.proc_root.iterdir()
        except OSError:
            return pids
        for entry in proc_entries:
            if not entry.name.isdigit():
                continue
            try:
                comm = (entry / "comm").read_text(encoding="utf-8").strip()
                raw_cmdline = (entry / "cmdline").read_bytes()
                cmdline = os.fsdecode(raw_cmdline).replace("\0", " ")
            except OSError:
                continue
            executable = Path(cmdline.split(" ", 1)[0]).name if cmdline else ""
            if (
                comm == "igh_driver"
                or executable == "igh_driver"
                or "lift_ethercat_cli" in cmdline
                or "ethercat_hardware_test.py" in cmdline
            ):
                pids.add(int(entry.name))
        return pids

    def _device_owners(self) -> list[tuple[int, str]]:
        device_paths = {str(path.resolve()) for path in self.device_root.glob("EtherCAT*")}
        if not device_paths:
            return []
        owners: list[tuple[int, str]] = []
        try:
            proc_entries = self.proc_root.iterdir()
        except OSError:
            return owners
        for entry in proc_entries:
            if not entry.name.isdigit():
                continue
            fd_dir = entry / "fd"
            try:
                descriptors = list(fd_dir.iterdir())
            except OSError:
                continue
            for descriptor in descriptors:
                try:
                    target = os.path.realpath(descriptor)
                except OSError:
                    continue
                if target in device_paths:
                    owners.append((int(entry.name), target))
        return owners

    def active_reasons(self) -> list[str]:
        reasons: list[str] = []
        pids = sorted(self._hardware_tool_pids())
        device_owners = self._device_owners()
        # Only report the service as a blocker if there is actually a hardware
        # tool process or an open EtherCAT device.  systemctl may still report
        # the service as "active" during the TimeoutStopSec window after the
        # IGH driver has already exited; without this guard the TCP console
        # cannot enter Direct mode until systemd marks the service inactive.
        if self._service_active() and (pids or device_owners):
            reasons.append(f"{self.service_name} is active")
        if pids:
            reasons.append("standalone EtherCAT hardware tool is running (pid " + ", ".join(map(str, pids)) + ")")
        for pid, device in device_owners:
            reasons.append(f"pid {pid} has {device} open")
        return reasons

    def assert_available(self) -> None:
        reasons = self.active_reasons()
        if reasons:
            raise HardwareBusy("hardware is already active: " + "; ".join(reasons))


class HardwareOwner:
    def __init__(self, path: Path = Path("/run/lock/junior-hardware-owner.lock")) -> None:
        self.path = path
        self.fd: int | None = None

    def acquire(self) -> None:
        if self.fd is not None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o660)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.close(fd)
            raise HardwareBusy("hardware is owned by ROS or another Direct worker") from exc
        self.fd = fd

    def release(self) -> None:
        if self.fd is None:
            return
        try:
            fcntl.flock(self.fd, fcntl.LOCK_UN)
        finally:
            os.close(self.fd)
            self.fd = None

    def __enter__(self) -> "HardwareOwner":
        self.acquire()
        return self

    def __exit__(self, *_args: object) -> None:
        self.release()
