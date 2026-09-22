"""Real non-ROS worker adapters used by hardware_server.

The adapters keep all EtherCAT access on the lower machine.  Network handlers
only submit bounded, high-level commands to these workers.
"""

from __future__ import annotations

import ctypes
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from typing import Any

HARDWARE_DIR = Path(__file__).resolve().parents[1]
import sys
if str(HARDWARE_DIR) not in sys.path:
    sys.path.insert(0, str(HARDWARE_DIR))

from ethercat_hardware_test import (  # noqa: E402
    AxisCommand,
    AxisFeedback,
    BusVoltageMonitor,
    DesireRegion,
    RealRegion,
    SharedMemory,
    arm_axis_master_positions,
    arm_axis_name,
    arm_feedback_ready,
    arm_rad_to_units,
    arm_status,
    allocate_driver_log,
    load_arm_zero_offsets,
    resolve_driver,
)


class WorkerError(RuntimeError):
    pass


class HardwareWorkers:
    def __init__(self, config: dict[str, Any], config_path: Path) -> None:
        self.config = config
        self.config_path = config_path
        self.arm: ArmWorker | None = None
        self.lift: LiftWorker | None = None
        self.ethercat_started = False
        self._lifecycle_lock = threading.RLock()
        self._startup_threads: dict[str, threading.Thread] = {}
        self._startup_workers: dict[str, Any] = {}
        self._startup_errors: dict[str, str] = {}
        self._stopping = False

    def start(self) -> None:
        """Compatibility helper for callers that explicitly want both workers."""
        started: list[str] = []
        try:
            self._ensure_ethercat()
            self._start_worker("arm")
            started.append("arm")
            self._start_worker("lift")
            started.append("lift")
        except Exception:
            for device in reversed(started):
                worker = getattr(self, device)
                if worker is not None:
                    worker.stop()
                    setattr(self, device, None)
            if self.ethercat_started:
                self._stop_ethercat()
            raise

    def _configured_devices_ready(self) -> bool:
        masters = self.config.get("ethercat", {}).get("masters", {})
        return bool(masters) and all(
            Path(f"/dev/EtherCAT{int(master['index'])}").exists()
            for master in masters.values()
        )

    def _ensure_ethercat(self) -> None:
        with self._lifecycle_lock:
            # run_hardware_server.sh normally starts EtherLab before this
            # process. Do not claim or stop an already-existing master.
            if self._configured_devices_ready():
                return
            init_script = Path(
                str(self.config.get("ethercat", {}).get("init_script", "/etc/init.d/ethercat"))
            )
            result = subprocess.run(
                [str(init_script), "start"],
                check=False,
                text=True,
                capture_output=True,
                timeout=20.0,
            )
            if result.returncode != 0:
                raise WorkerError(f"failed to start EtherCAT master: {result.stderr.strip()}")
            self.ethercat_started = True
            if not self._configured_devices_ready():
                raise WorkerError("required EtherCAT master devices are missing")

    def _start_worker(self, device: str) -> None:
        with self._lifecycle_lock:
            if self._stopping:
                raise WorkerError("hardware workers are stopping")
            if getattr(self, device) is not None:
                return
        worker: Any = ArmWorker(self.config, self.config_path) if device == "arm" else LiftWorker(
            self.config, self.config_path
        )
        with self._lifecycle_lock:
            self._startup_workers[device] = worker
        failed = False
        try:
            self._ensure_ethercat()
            worker.start()
            with self._lifecycle_lock:
                if self._stopping:
                    worker.stop()
                    return
                setattr(self, device, worker)
        except Exception:
            failed = True
            raise
        finally:
            with self._lifecycle_lock:
                self._startup_workers.pop(device, None)
                stop_owned_ethercat = (
                    failed
                    and self.ethercat_started
                    and self.arm is None
                    and self.lift is None
                    and not self._startup_workers
                )
            if stop_owned_ethercat:
                self._stop_ethercat()

    def _start_worker_background(self, device: str) -> None:
        try:
            self._start_worker(device)
        except Exception as exc:
            with self._lifecycle_lock:
                self._startup_errors[device] = str(exc)
        finally:
            with self._lifecycle_lock:
                self._startup_threads.pop(device, None)

    def ensure_worker(self, device: str) -> None:
        if device not in {"arm", "lift"}:
            raise WorkerError(f"unsupported worker: {device}")
        with self._lifecycle_lock:
            if self._stopping:
                raise WorkerError("hardware workers are stopping")
            if getattr(self, device) is not None:
                return
            thread = self._startup_threads.get(device)
            if thread is not None and thread.is_alive():
                return
            self._startup_errors.pop(device, None)
            thread = threading.Thread(
                target=self._start_worker_background,
                args=(device,),
                name=f"{device}-worker-start",
                daemon=True,
            )
            self._startup_threads[device] = thread
            thread.start()

    def startup_state(self, device: str) -> tuple[str, str]:
        with self._lifecycle_lock:
            if getattr(self, device) is not None:
                return "ready", ""
            error = self._startup_errors.get(device)
            if error:
                return "error", error
            thread = self._startup_threads.get(device)
            if thread is not None and thread.is_alive():
                return "starting", ""
            return "idle", ""

    def stop(self, *, stop_ethercat: bool = False) -> None:
        with self._lifecycle_lock:
            self._stopping = True
            startup_workers = list(self._startup_workers.values())
        for worker in startup_workers:
            worker.stop()
        for thread in list(self._startup_threads.values()):
            thread.join(timeout=1.0)
        for worker in (self.lift, self.arm):
            if worker is not None:
                worker.stop()
        self.lift = None
        self.arm = None
        # Keep EtherLab masters alive between Direct sessions. A client leave
        # must stop workers, but stopping the masters here makes the next
        # ENTER_DIRECT skip the full setup and lose slave discovery.
        if stop_ethercat and self.ethercat_started:
            self._stop_ethercat()

    def _stop_ethercat(self) -> None:
        if not self.ethercat_started:
            return
        self.ethercat_started = False
        init_script = Path(str(self.config.get("ethercat", {}).get("init_script", "/etc/init.d/ethercat")))
        subprocess.run(
            [str(init_script), "stop"], check=False, stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20.0,
        )

    def command(self, message_type: str, payload: dict[str, Any]) -> dict[str, Any]:
        if message_type.startswith("ARM_"):
            if self.arm is None:
                raise WorkerError("arm worker is not running")
            return self.arm.command(message_type[4:], payload)
        if message_type.startswith("LIFT_"):
            if self.lift is None:
                raise WorkerError("lift worker is not running")
            return self.lift.command(message_type[5:], payload)
        if message_type == "HOLD_ALL":
            result = {}
            if self.arm is not None:
                result["arm"] = self.arm.command("HOLD", {})
            if self.lift is not None:
                result["lift"] = self.lift.command("HOLD", {})
            return result
        raise WorkerError(f"unsupported worker command: {message_type}")


class ArmWorker:
    def __init__(self, config: dict[str, Any], config_path: Path) -> None:
        self.config = config
        self.config_path = config_path
        self.arm_config = config.get("arm", {})
        self.driver: subprocess.Popen[bytes] | None = None
        self.desire_block: SharedMemory | None = None
        self.real_block: SharedMemory | None = None
        self.desire: DesireRegion | None = None
        self.real: RealRegion | None = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.lock = threading.RLock()
        self.enabled = False
        self.targets: dict[int, int] = {}
        # Position requested by the client and position currently being sent
        # to the CSP driver are separate.  The latter must advance from the
        # previous command; recomputing it from feedback every cycle limits
        # motion to roughly one control-period of progress.
        self.command_positions: dict[int, int] = {}
        self.limits: dict[int, tuple[int, int]] = {}
        self.zero_offsets: dict[int, int] = {slot: 0 for slot in range(14)}
        self.motion_direction: dict[int, int] = {}
        self.blocked_direction: dict[int, int] = {}
        self.motion_watch_position: dict[int, int] = {}
        self.motion_watch_started: dict[int, float] = {}
        self.bus_monitor: BusVoltageMonitor | None = None

    def _raise_if_start_cancelled(self) -> None:
        if self.stop_event.is_set():
            raise WorkerError("arm worker startup cancelled")

    def _load_limits(self, units: float) -> None:
        urdf = Path(str(self.arm_config.get(
            "limits_urdf", self.config_path.parent.parent.parent / "src/robot_arm_description/urdf/right_left_arm.urdf"
        )))
        if not urdf.is_absolute():
            urdf = (self.config_path.parent / urdf).resolve()
        if not urdf.exists():
            raise WorkerError(f"arm limits URDF is missing: {urdf}")
        root = ET.parse(urdf).getroot()
        offsets = load_arm_zero_offsets(self.config, self.config_path, list(range(14)))
        self.zero_offsets = offsets
        for slot in range(14):
            joint = root.find(f".//joint[@name='{arm_axis_name(slot)}']")
            limit = joint.find("limit") if joint is not None else None
            if limit is None:
                raise WorkerError(f"missing absolute limit for {arm_axis_name(slot)}")
            lower, upper = float(limit.attrib["lower"]), float(limit.attrib["upper"])
            self.limits[slot] = (
                offsets[slot] + arm_rad_to_units(lower, units),
                offsets[slot] + arm_rad_to_units(upper, units),
            )

    def start(self) -> None:
        try:
            self._raise_if_start_cancelled()
            if os.geteuid() != 0:
                raise WorkerError("real arm worker requires root")
            driver_path = resolve_driver(self.config, self.config_path)
            log_path = allocate_driver_log(
                self.arm_config.get("driver_log", "heavy_v1_igh_driver.log")
            )
            log_file = log_path.open("wb")
            env = os.environ.copy()
            library_dir = str(self.config.get("ethercat", {}).get("library_dir", "/usr/local/etherlab/lib"))
            env["LD_LIBRARY_PATH"] = library_dir + ":" + env.get("LD_LIBRARY_PATH", "")
            self.driver = subprocess.Popen([str(driver_path)], stdout=log_file, stderr=subprocess.STDOUT, env=env)
            log_file.close()
            self._raise_if_start_cancelled()
            deadline = time.monotonic() + float(self.arm_config.get("feedback_ready_timeout_s", 35.0))
            self.desire_block = SharedMemory(str(self.arm_config["shared_memory"]["desire"]), DesireRegion)
            self.real_block = SharedMemory(str(self.arm_config["shared_memory"]["real"]), RealRegion)
            while time.monotonic() < deadline:
                self._raise_if_start_cancelled()
                if self.driver.poll() is not None:
                    raise WorkerError(
                        f"IGH driver exited with code {self.driver.returncode}; log={log_path}"
                    )
                try:
                    self.desire_block.open()
                    self.real_block.open()
                    break
                except RuntimeError:
                    self.desire_block.close(); self.real_block.close(); time.sleep(0.05)
            else:
                raise WorkerError("IGH driver did not create shared memory")
            self.desire, self.real = self.desire_block.value, self.real_block.value
            units = float(self.arm_config.get("position_units_per_rad", 83443.02680376362))
            self._load_limits(units)
            if self.desire is None or self.real is None:
                raise WorkerError("arm shared memory mapping failed")
            mode = int(str(self.arm_config.get("csp_mode", "0x08")), 0)
            feedback_deadline = time.monotonic() + float(self.arm_config.get("feedback_ready_timeout_s", 35.0))
            while not arm_feedback_ready(self.real, list(range(14)), mode):
                self._raise_if_start_cancelled()
                if time.monotonic() >= feedback_deadline:
                    statuses = ",".join(
                        f"{slot}:state=0x{int(self.real.axis_state[slot].ec_ctrstate):04x},"
                        f"mode=0x{int(self.real.axis_state[slot].ec_modestate):02x},"
                        f"error=0x{int(self.real.axis_state[slot].axis_error_code):04x}"
                        for slot in range(14)
                    )
                    raise WorkerError(
                        "arm feedback did not become ready; "
                        f"ec_powerstate={int(self.real.ec_powerstate)}; axes=[{statuses}]; log={log_path}"
                    )
                if self.driver.poll() is not None:
                    raise WorkerError(
                        f"IGH driver exited with code {self.driver.returncode}; log={log_path}"
                    )
                time.sleep(0.02)
            self._raise_if_start_cancelled()
            self._hold_locked()
            # Read-only bus voltage / undervoltage limit reporting; forwarded to
            # the upper console through GET_ARM_STATE.
            self.bus_monitor = BusVoltageMonitor(
                arm_axis_master_positions(self.config)
            )
            self.bus_monitor.start()
            self.thread = threading.Thread(target=self._loop, name="arm-worker", daemon=True)
            self.thread.start()
        except Exception:
            self.stop()
            raise

    def _hold_locked(self) -> None:
        assert self.desire is not None and self.real is not None
        mode = int(str(self.arm_config.get("csp_mode", "0x08")), 0)
        for slot in range(14):
            self.desire.axis_ctr[slot].ec_mode = mode
            self.desire.axis_ctr[slot].axis_position = self.real.axis_state[slot].axis_position
            self.desire.axis_ctr[slot].axis_velocity = 0
            self.desire.axis_ctr[slot].axis_effort = 0
            self.targets[slot] = self.real.axis_state[slot].axis_position
            self.command_positions[slot] = self.real.axis_state[slot].axis_position
            # A HOLD/DISABLE starts a fresh motion command.  Do not carry a
            # previous stall direction into the next control session, or the
            # first jog after re-enable would be rejected indefinitely.
            self.motion_direction[slot] = 0
            self.blocked_direction[slot] = 0
            self.motion_watch_position[slot] = self.real.axis_state[slot].axis_position
            self.motion_watch_started[slot] = time.monotonic()

    def _loop(self) -> None:
        while not self.stop_event.wait(0.01):
            with self.lock:
                if self.desire is None or self.real is None:
                    return
                if self.driver is None or self.driver.poll() is not None:
                    self.enabled = False
                _, faults, _, _ = arm_status(self.real, list(range(14)))
                if faults:
                    self.enabled = False
                if self.enabled:
                    self.desire.ec_poweron = 1
                    max_speed = float(self.arm_config.get("max_speed_rad_s", 0.1))
                    units = float(self.arm_config.get("position_units_per_rad", 83443.02680376362))
                    delta = max(1, int(max_speed * units * 0.01))
                    for slot, requested in self.targets.items():
                        actual = self.real.axis_state[slot].axis_position
                        target = max(self.limits[slot][0], min(self.limits[slot][1], requested))
                        direction = self.motion_direction.get(slot, 0)
                        if direction:
                            progress = (actual - self.motion_watch_position.get(slot, actual)) * direction
                            # ZeroLegacy feedback can advance in small bursts;
                            # using the full display tolerance (83 counts at
                            # 0.001 rad) falsely classified those bursts as a
                            # stall. Keep a small progress threshold while the
                            # independent timeout still protects hard stops.
                            progress_tolerance = float(self.arm_config.get("limit_progress_tolerance_rad", 0.0001))
                            progress_units = max(1, int(progress_tolerance * units))
                            if progress >= progress_units:
                                self.motion_watch_position[slot] = actual
                                self.motion_watch_started[slot] = time.monotonic()
                            elif (
                                time.monotonic() - self.motion_watch_started.get(slot, time.monotonic())
                                >= float(self.arm_config.get("limit_stall_timeout_s", 0.5))
                                and (requested - actual) * direction >= max(1, int(float(self.arm_config.get("jog_step_rad", 0.005)) * units / 2.0))
                            ):
                                self.targets[slot] = actual
                                self.command_positions[slot] = actual
                                self.motion_direction[slot] = 0
                                self.blocked_direction[slot] = direction
                                target = actual
                                LOG.warning(
                                    "arm axis %s stalled: actual=%d requested=%d direction=%d "
                                    "progress=%d threshold=%d",
                                    arm_axis_name(slot), actual, requested, direction,
                                    progress, progress_units,
                                )
                        commanded = self.command_positions.get(slot, actual)
                        step = max(-delta, min(delta, target - commanded))
                        commanded += step
                        self.command_positions[slot] = commanded
                        self.desire.axis_ctr[slot].axis_position = commanded
                else:
                    self.desire.ec_poweron = 0
                    self._hold_locked()

    def command(self, command: str, payload: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            if command == "ENABLE":
                self._hold_locked(); self.enabled = True
            elif command in {"DISABLE", "HOLD"}:
                self._hold_locked()
                if command == "DISABLE":
                    self.enabled = False
            elif command == "HOME":
                if not self.enabled:
                    raise WorkerError("enable the arm before homing")
                enabled, faults, modes_ok, total = arm_status(self.real, list(range(14)))
                if enabled != total or faults or modes_ok != total:
                    raise WorkerError("all 14 arm axes must be Operation Enabled before homing")
                joint = payload.get("joint")
                if joint not in self.configured_names():
                    raise WorkerError("unknown arm joint")
                slot = list(self.configured_names()).index(joint)
                self.targets[slot] = self.zero_offsets[slot]
                self.command_positions[slot] = self.real.axis_state[slot].axis_position
                self.motion_direction[slot] = 0
                self.blocked_direction[slot] = 0
            elif command == "JOG":
                if not self.enabled:
                    raise WorkerError("enable the arm before jogging")
                enabled, faults, modes_ok, total = arm_status(self.real, list(range(14)))
                if enabled != total or faults or modes_ok != total:
                    raise WorkerError("all 14 arm axes must be Operation Enabled before jogging")
                if payload.get("joint") not in {arm_axis_name(i) for i in range(14)}:
                    raise WorkerError("unknown arm joint")
                direction = payload.get("direction")
                if direction not in (-1, 1):
                    raise WorkerError("direction must be -1 or 1")
                slot = list(self.configured_names()).index(payload["joint"])
                step = float(payload.get("step_rad", self.arm_config.get("jog_step_rad", 0.005)))
                max_step = float(self.arm_config.get("max_step_rad", 0.1))
                if not math.isfinite(step) or step <= 0 or step > max_step:
                    raise WorkerError("arm jog step exceeds lower-machine limit")
                if self.blocked_direction.get(slot, 0) == direction:
                    raise WorkerError("axis is blocked at its limit; jog in the opposite direction first")
                actual = self.real.axis_state[slot].axis_position
                if self.blocked_direction.get(slot, 0) == -direction or self.motion_direction.get(slot, 0) != direction:
                    self.blocked_direction[slot] = 0
                    self.targets[slot] = actual
                    self.command_positions[slot] = actual
                self.targets[slot] = self.targets.get(slot, actual) + direction * arm_rad_to_units(step, float(self.arm_config.get("position_units_per_rad", 83443.02680376362)))
                self.targets[slot] = max(self.limits[slot][0], min(self.limits[slot][1], self.targets[slot]))
                self.motion_direction[slot] = direction
                self.motion_watch_position[slot] = actual
                self.motion_watch_started[slot] = time.monotonic()
            else:
                raise WorkerError(f"unsupported arm command: {command}")
            return {"accepted": True, "enabled": self.enabled, "backend": "real"}

    @staticmethod
    def configured_names() -> tuple[str, ...]:
        return tuple([f"ljoint{i}" for i in range(1, 8)] + [f"rjoint{i}" for i in range(1, 8)])

    def stop(self) -> None:
        self.stop_event.set()
        if self.bus_monitor is not None:
            self.bus_monitor.stop()
            self.bus_monitor = None
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        with self.lock:
            if self.desire is not None:
                self.desire.ec_poweron = 0
        if self.driver is not None and self.driver.poll() is None:
            self.driver.send_signal(signal.SIGINT)
            try: self.driver.wait(timeout=3.0)
            except subprocess.TimeoutExpired: self.driver.kill()
        self.desire = None
        self.real = None
        for block in (self.desire_block, self.real_block):
            if block is not None: block.close()
        self.driver = None
        self.thread = None

    def state(self) -> dict[str, Any]:
        with self.lock:
            axes = []
            units = float(self.arm_config.get("position_units_per_rad", 83443.02680376362))
            for slot in range(14):
                axis = self.real.axis_state[slot] if self.real is not None else None
                raw = int(axis.axis_position) if axis is not None else 0
                status = int(axis.ec_ctrstate) if axis is not None else 0
                status_names = {
                    0x27: "OPERATION_ENABLED",
                    0x23: "SWITCHED_ON",
                    0x21: "READY_TO_SWITCH_ON",
                    0x40: "SWITCH_ON_DISABLED",
                    0x08: "FAULT",
                }
                axes.append({
                    "joint": arm_axis_name(slot),
                    "status": status_names.get(status & 0x006F, "UNKNOWN"),
                    "status_code": status,
                    "mode": int(axis.ec_modestate) if axis is not None else 0,
                    "error": int(axis.axis_error_code) if axis is not None else 0,
                    "position_rad": (raw - self.zero_offsets.get(slot, 0)) / units,
                })
            return {
                "backend": "real",
                "enabled": self.enabled,
                "power": bool(self.desire.ec_poweron) if self.desire is not None else False,
                "axes": axes,
                "bus": self.bus_monitor.snapshot() if self.bus_monitor is not None else {},
            }


class LiftWorker:
    def __init__(self, config: dict[str, Any], config_path: Path) -> None:
        self.config = config
        self.config_path = config_path
        self.proc: subprocess.Popen[bytes] | None = None
        self.lock = threading.Lock()
        self.enabled = False
        self.stop_event = threading.Event()
        # Partial stdout line carried between non-blocking reads.
        self._pending = ""

    def _raise_if_start_cancelled(self) -> None:
        if self.stop_event.is_set():
            raise WorkerError("lift worker startup cancelled")

    def start(self) -> None:
        try:
            self._raise_if_start_cancelled()
            lift = self.config["lift"]
            binary = (self.config_path.parent / str(lift["cli_binary"])).resolve()
            if not binary.is_file() or not os.access(binary, os.X_OK):
                raise WorkerError(f"lift CLI binary is missing: {binary}")
            cmd = [str(binary), "--command-mode", "--interface", str(lift["interface"]), "--master", str(lift["master"]), "--alias", str(lift["slave_alias"]), "--position", str(lift["slave_position"]), "--min-position", str(lift["min_position_m"]), "--max-position", str(lift["max_position_m"]), "--speed", str(lift.get("max_speed_mps", 0.015)), "--accel", str(lift.get("acceleration_mps2", 0.033333333)), "--zero-offset-file", str(lift["zero_offset_file"])]
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self._raise_if_start_cancelled()
            # Read stdout without blocking.  The CLI is legitimately silent for
            # seconds while it activates the master and configures PDOs, and a
            # complete line that the buffered reader already holds must still be
            # read even though select() on the raw fd reports nothing new.
            os.set_blocking(self.proc.stdout.fileno(), False)
            self._pending = ""
            # The lift backend may need several EtherCAT cycles to configure
            # PDOs and obtain its first valid sample after a master restart.
            deadline = time.monotonic() + 30.0
            output_tail: list[str] = []
            while time.monotonic() < deadline:
                self._raise_if_start_cancelled()
                if self.proc.stdout is None:
                    raise WorkerError("lift CLI stdout is unavailable")
                line = self._read_cli_line()
                if line is not None:
                    output_tail.append(line)
                    del output_tail[:-8]
                    if "COMMAND_MODE_READY" in line:
                        self._raise_if_start_cancelled()
                        return
                    continue
                if self.proc.poll() is not None:
                    detail = "; ".join(item for item in output_tail if item)
                    raise WorkerError(
                        f"lift CLI exited with code {self.proc.returncode}"
                        + (f": {detail}" if detail else "")
                    )
                # Silence is not failure: keep waiting until the deadline.
                time.sleep(0.02)
            detail = "; ".join(item for item in output_tail if item)
            raise WorkerError(
                "lift CLI did not become ready within 30 seconds"
                + (f": {detail}" if detail else "")
            )
        except Exception:
            self.stop()
            raise

    def command(self, command: str, payload: dict[str, Any]) -> dict[str, Any]:
        if self.proc is None or self.proc.stdin is None:
            raise WorkerError("lift worker is not running")
        if command == "JOG":
            if not self.enabled:
                raise WorkerError("enable the lift before jogging")
            direction = payload.get("direction")
            if direction not in (-1, 1): raise WorkerError("direction must be -1 or 1")
            lift = self.config["lift"]
            step = float(payload.get("step_m", lift.get("jog_step_m", 0.01)))
            max_step = float(lift.get("max_step_m", 0.1))
            if not math.isfinite(step) or step <= 0 or step > max_step: raise WorkerError("lift jog step exceeds lower-machine limit")
            line = f"MOVE_STEP {direction * step}\n"
        elif command in {"ENABLE", "DISABLE", "HOLD", "ZERO", "HOME"}:
            line = command + "\n"
            if command == "ENABLE": self.enabled = True
            elif command in {"DISABLE", "ZERO"}: self.enabled = False
            elif command == "HOME": self.enabled = True
        else:
            raise WorkerError(f"unsupported lift command: {command}")
        with self.lock:
            self.proc.stdin.write(line.encode("ascii")); self.proc.stdin.flush()
        return {"accepted": True, "enabled": self.enabled, "backend": "real"}

    @staticmethod
    def _parse_state_line(line: str) -> dict[str, Any] | None:
        if not line.startswith("LIFT_STATE "):
            return None
        fields: dict[str, str] = {}
        for token in line.split()[1:]:
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value
        if fields.get("available") != "1":
            return {"feedback_available": False}
        try:
            return {
                "feedback_available": True,
                "position_m": float(fields["position_m"]),
                "velocity_mps": float(fields["velocity_mps"]),
                "pdo_fresh": fields.get("pdo_fresh") == "1",
                "link_state": fields.get("link_state", "unknown"),
                "status_word": int(fields.get("status_word", "0"), 0),
                "error_code": int(fields.get("error_code", "0"), 0),
            }
        except (KeyError, ValueError):
            return {"feedback_available": False, "feedback_error": "invalid lift state response"}

    def _read_cli_line(self) -> str | None:
        """Return the next complete CLI stdout line, or None when none is ready.

        The stream is non-blocking, so a read can return a partial line: bytes
        are accumulated until a newline arrives, and a caller keeps control of
        its own deadline instead of blocking.
        """
        if self.proc is None or self.proc.stdout is None:
            return None
        while "\n" not in self._pending:
            try:
                raw = self.proc.stdout.readline()
            except BlockingIOError:
                return None
            if not raw:
                return None
            self._pending += raw.decode("utf-8", errors="replace")
        line, self._pending = self._pending.split("\n", 1)
        return line.strip()

    def _read_state(self, timeout_s: float = 0.5) -> dict[str, Any]:
        if self.proc is None or self.proc.stdin is None or self.proc.stdout is None:
            return {"feedback_available": False, "feedback_error": "lift worker is not running"}
        deadline = time.monotonic() + timeout_s
        with self.lock:
            self.proc.stdin.write(b"STATE\n")
            self.proc.stdin.flush()
            while time.monotonic() < deadline:
                line = self._read_cli_line()
                if line is None:
                    if self.proc.poll() is not None:
                        break
                    time.sleep(0.005)
                    continue
                parsed = self._parse_state_line(line)
                if parsed is not None:
                    return parsed
        return {"feedback_available": False, "feedback_error": "lift state response timed out"}

    def stop(self) -> None:
        self.stop_event.set()
        if self.proc is None: return
        try:
            if self.proc.stdin is not None:
                self.proc.stdin.write(b"HOLD\nQUIT\n"); self.proc.stdin.flush()
        except OSError: pass
        try: self.proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired: self.proc.kill()
        self.proc = None
        self.enabled = False

    def state(self) -> dict[str, Any]:
        process_alive = self.proc is not None and self.proc.poll() is None
        feedback = self._read_state() if process_alive else {"feedback_available": False}
        return {
            "backend": "real",
            "enabled": self.enabled,
            "process_alive": process_alive,
            **feedback,
        }
