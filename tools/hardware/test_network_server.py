from __future__ import annotations

import socket
import sys
import tempfile
import threading
from unittest import mock
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
NETWORK_DIR = SCRIPT_DIR / "network"
sys.path.insert(0, str(NETWORK_DIR))

from config_store import ConfigError, ConfigStore  # noqa: E402
import hardware_server  # noqa: E402
from hardware_server import ServerState, build_tls_context  # noqa: E402
from mode_guard import HardwareBusy, HardwareOwner  # noqa: E402
from protocol import encode_frame, recv_frame  # noqa: E402
import workers as workers_module  # noqa: E402
from workers import ArmWorker, DesireRegion, HardwareWorkers, LiftWorker, RealRegion, WorkerError  # noqa: E402


BASE_CONFIG = """
config_revision: 2
robot_variant: heavy_v1
server:
  listen_address: 127.0.0.1
  port: 7447
arm:
  jog_step_rad: 0.005
  max_step_rad: 0.1
  max_speed_rad_s: 0.1
lift:
  min_position_m: -1.0
  max_position_m: 0.0
  jog_step_m: 0.01
  max_step_m: 0.1
  max_speed_mps: 0.015
  acceleration_mps2: 0.033333333
"""


def make_state(tmp_path: Path) -> ServerState:
    config = tmp_path / "hardware_io.yaml"
    config.write_text(BASE_CONFIG, encoding="utf-8")
    return ServerState(ConfigStore(config), tmp_path / "owner.lock", activity_probe=AvailableProbe())


class AvailableProbe:
    def assert_available(self) -> None:
        return None


class BusyProbe:
    def assert_available(self) -> None:
        raise HardwareBusy("hardware is already active: test owner")


def request(request_id: int, message_type: str, session: str = "client-1", **payload):
    return {
        "version": 1,
        "request_id": request_id,
        "session_id": session,
        "type": message_type,
        "payload": payload,
    }


def test_server_state_direct_lease_and_config_transaction(tmp_path: Path) -> None:
    state = make_state(tmp_path)
    try:
        assert state.handle(request(1, "HELLO"))["ok"]
        direct = state.handle(request(2, "ENTER_DIRECT"))
        assert direct["payload"]["mode"] == "DIRECT_ACTIVE"
        assert direct["payload"]["lift_test_range"] is False
        assert state.handle(request(3, "ACQUIRE_LEASE"))["payload"]["lease"] is True
        denied = state.handle(request(4, "ARM_JOG", joint="ljoint1", direction=1))
        assert denied["ok"]
        state.handle(request(5, "RELEASE_LEASE"))
        state.handle(request(6, "LEAVE_DIRECT"))
        state.handle(request(7, "BEGIN_CONFIG"))
        state.handle(request(8, "PATCH_CONFIG", path="arm.max_speed_rad_s", value=0.08))
        result = state.handle(request(9, "COMMIT_CONFIG", expected_revision=2))
        assert result["payload"]["revision"] == 3
    finally:
        state.stop()


def test_lift_home_and_zero_are_lease_protected_commands(tmp_path: Path) -> None:
    state = make_state(tmp_path)
    try:
        state.handle(request(1, "ENTER_DIRECT"))
        state.handle(request(2, "ACQUIRE_LEASE"))
        for request_id, message_type in ((3, "LIFT_HOME"), (4, "LIFT_ZERO")):
            response = state.handle(request(request_id, message_type))
            assert response["ok"]
            assert response["payload"]["backend"] == "dry_run"
    finally:
        state.stop()


def test_console_lift_test_range_is_session_only(tmp_path: Path) -> None:
    state = make_state(tmp_path)
    try:
        response = state.handle(request(1, "ENTER_DIRECT", lift_test_range=True))
        assert response["payload"]["lift_test_range"] is True
        assert state.config.snapshot()["config"]["lift"]["max_position_m"] == 0.0
    finally:
        state.stop()


def test_lift_worker_parses_measured_state() -> None:
    parsed = LiftWorker._parse_state_line(
        "LIFT_STATE available=1 position_m=-0.123456789 velocity_mps=0.010000000 "
        "pdo_fresh=1 link_state=operational status_word=39 error_code=0"
    )
    assert parsed == {
        "feedback_available": True,
        "position_m": -0.123456789,
        "velocity_mps": 0.01,
        "pdo_fresh": True,
        "link_state": "operational",
        "status_word": 39,
        "error_code": 0,
    }
    assert LiftWorker._parse_state_line("LIFT_STATE available=0") == {
        "feedback_available": False
    }


def test_lift_worker_startup_is_cancelled_when_direct_mode_releases(tmp_path: Path) -> None:
    binary = tmp_path / "lift_cli"
    binary.write_text("#!/bin/sh\n", encoding="ascii")
    binary.chmod(0o755)
    config_path = tmp_path / "hardware_io.yaml"
    config_path.write_text(BASE_CONFIG, encoding="utf-8")
    worker = LiftWorker(
        {
            "lift": {
                "cli_binary": binary.name,
                "interface": "enp5s0",
                "master": 2,
                "slave_alias": 0,
                "slave_position": 0,
                "min_position_m": -1.0,
                "max_position_m": 0.0,
                "zero_offset_file": "lift_zero.yaml",
            }
        },
        config_path,
    )
    startup_waiting = threading.Event()
    release_wait = threading.Event()

    class FakeInput:
        def write(self, _data):
            return 0

        def flush(self):
            return None

    class FakeProcess:
        def __init__(self):
            self.stdin = FakeInput()
            self.stdout = object()
            self.wait_calls = 0

        def poll(self):
            return None

        def wait(self, timeout=None):
            self.wait_calls += 1
            return 0

        def kill(self):
            return None

    process = FakeProcess()
    errors: list[Exception] = []

    def wait_for_cli(_read, _write, _error, _timeout):
        startup_waiting.set()
        release_wait.wait(timeout=1.0)
        return [], [], []

    def start() -> None:
        try:
            worker.start()
        except Exception as exc:
            errors.append(exc)

    with mock.patch.object(workers_module.subprocess, "Popen", return_value=process), mock.patch.object(
        workers_module.select, "select", side_effect=wait_for_cli
    ):
        thread = threading.Thread(target=start)
        thread.start()
        assert startup_waiting.wait(timeout=0.5)
        worker.stop()
        release_wait.set()
        thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert errors and "startup cancelled" in str(errors[0])
    assert worker.proc is None
    assert process.wait_calls >= 1


def test_console_lift_test_range_only_changes_runtime_worker_config(tmp_path: Path) -> None:
    captured: dict[str, object] = {}
    config_path = tmp_path / "hardware_io.yaml"
    config_path.write_text(BASE_CONFIG, encoding="utf-8")

    class FakeWorkers:
        def __init__(self, config, config_path):
            captured["config"] = config
            captured["config_path"] = config_path
            captured["started"] = False

        def start(self):
            captured["started"] = True

        def stop(self, **kwargs):
            return None

    with mock.patch.object(hardware_server, "HardwareWorkers", FakeWorkers):
        state = ServerState(
            ConfigStore(config_path),
            tmp_path / "owner.lock",
            real_backend=True,
            activity_probe=AvailableProbe(),
        )
        try:
            response = state.handle(request(1, "ENTER_DIRECT", lift_test_range=True))
            runtime_config = captured["config"]
            assert response["payload"]["lift_test_range"] is True
            assert runtime_config["lift"]["min_position_m"] == -1.0
            assert runtime_config["lift"]["max_position_m"] == 1.0
            assert state.config.snapshot()["config"]["lift"]["max_position_m"] == 0.0
            assert captured["started"] is False
        finally:
            state.stop()


def test_real_direct_state_starts_only_the_requested_worker(tmp_path: Path) -> None:
    captured: dict[str, object] = {}

    class FakeWorkers:
        def __init__(self, config, config_path):
            self.arm = None
            self.lift = None
            self.requested: list[str] = []
            captured["workers"] = self

        def ensure_worker(self, device):
            self.requested.append(device)

        def startup_state(self, device):
            return "starting", ""

        def stop(self, **kwargs):
            return None

    config_path = tmp_path / "hardware_io.yaml"
    config_path.write_text(BASE_CONFIG, encoding="utf-8")
    with mock.patch.object(hardware_server, "HardwareWorkers", FakeWorkers):
        state = ServerState(
            ConfigStore(config_path),
            tmp_path / "owner.lock",
            real_backend=True,
            activity_probe=AvailableProbe(),
        )
        try:
            state.handle(request(1, "ENTER_DIRECT"))
            arm = state.handle(request(2, "GET_ARM_STATE"))
            assert arm["payload"]["worker_state"] == "starting"
            assert captured["workers"].requested == ["arm"]
        finally:
            state.stop()


def test_existing_ethercat_masters_are_not_claimed_for_cleanup(tmp_path: Path) -> None:
    config = {
        "ethercat": {
            "init_script": "/etc/init.d/ethercat",
            "masters": {
                "left_arm": {"index": 0},
                "right_arm": {"index": 1},
                "lift": {"index": 2},
            },
        }
    }
    workers = HardwareWorkers(config, tmp_path / "hardware_io.yaml")
    with mock.patch.object(workers_module.Path, "exists", return_value=True), mock.patch.object(
        workers_module.subprocess, "run"
    ) as run:
        workers._ensure_ethercat()
    run.assert_not_called()
    assert workers.ethercat_started is False


def test_server_rejects_second_direct_session(tmp_path: Path) -> None:
    state = make_state(tmp_path)
    try:
        state.handle(request(1, "ENTER_DIRECT", "first"))
        try:
            state.handle(request(2, "ENTER_DIRECT", "second"))
        except HardwareBusy:
            pass
        else:
            raise AssertionError("second Direct session should be rejected")
    finally:
        state.stop()


def test_disconnect_releases_direct_lock_even_without_lease(tmp_path: Path) -> None:
    state = make_state(tmp_path)
    competing_owner = HardwareOwner(tmp_path / "owner.lock")
    try:
        state.handle(request(1, "ENTER_DIRECT", "first"))
        state.session_closed("first")
        competing_owner.acquire()
        assert state.mode == "IDLE"
        assert not state.direct_active
    finally:
        competing_owner.release()
        state.stop()


def test_busy_runtime_probe_rejects_direct_and_releases_lock(tmp_path: Path) -> None:
    config_path = tmp_path / "hardware_io.yaml"
    config_path.write_text(BASE_CONFIG, encoding="utf-8")
    lock_path = tmp_path / "owner.lock"
    state = ServerState(ConfigStore(config_path), lock_path, activity_probe=BusyProbe())
    competing_owner = HardwareOwner(lock_path)
    try:
        try:
            state.handle(request(1, "ENTER_DIRECT"))
        except HardwareBusy as exc:
            assert "test owner" in str(exc)
        else:
            raise AssertionError("active ROS/IGH ownership should reject Direct mode")
        competing_owner.acquire()
    finally:
        competing_owner.release()
        state.stop()


def test_config_commit_persists_backup_and_history(tmp_path: Path) -> None:
    config_path = tmp_path / "hardware_io.yaml"
    config_path.write_text(BASE_CONFIG, encoding="utf-8")
    store = ConfigStore(config_path)
    store.begin()
    store.patch("arm.max_speed_rad_s", 0.08)
    committed = store.commit(2)
    assert committed["revision"] == 3

    history = store.history()
    assert len(history) == 1
    assert history[0]["previous_revision"] == 2
    assert (store.history_dir / history[0]["backup"]).read_text(encoding="utf-8") == BASE_CONFIG

    reloaded = ConfigStore(config_path)
    assert reloaded.snapshot()["revision"] == 3
    assert reloaded.history() == history


def test_travel_limits_are_not_remote_mutable(tmp_path: Path) -> None:
    state = make_state(tmp_path)
    try:
        state.handle(request(1, "BEGIN_CONFIG"))
        try:
            state.handle(request(2, "PATCH_CONFIG", path="lift.max_position_m", value=1.0))
        except Exception as exc:
            assert "not remotely mutable" in str(exc)
        else:
            raise AssertionError("travel limits must remain lower-machine-only")
    finally:
        state.stop()


def test_config_transaction_is_session_owned_and_disconnect_rolls_back(tmp_path: Path) -> None:
    state = make_state(tmp_path)
    try:
        state.handle(request(1, "BEGIN_CONFIG", "first"))
        try:
            state.handle(request(2, "PATCH_CONFIG", "second", path="arm.max_speed_rad_s", value=0.04))
        except HardwareBusy:
            pass
        else:
            raise AssertionError("another session must not take over a configuration transaction")
        state.session_closed("first")
        state.handle(request(3, "BEGIN_CONFIG", "second"))
        assert state.config_session == "second"
    finally:
        state.stop()


def test_mtls_requires_all_certificate_paths() -> None:
    try:
        build_tls_context({"tls": {"enabled": True}})
    except ConfigError as exc:
        assert "paths are empty" in str(exc)
    else:
        raise AssertionError("enabled mTLS without certificate paths must fail")


def test_arm_worker_requires_enable_and_clamps_to_urdf_limit(tmp_path: Path) -> None:
    joints = "\n".join(
        f'<joint name="{side}joint{index}"><limit lower="-0.1" upper="0.1"/></joint>'
        for side in ("l", "r") for index in range(1, 8)
    )
    urdf = tmp_path / "limits.urdf"
    urdf.write_text(f"<robot>{joints}</robot>", encoding="utf-8")
    config_path = tmp_path / "hardware_io.yaml"
    config = {
        "arm": {
            "limits_urdf": str(urdf),
            "zero_offset_file": str(tmp_path / "missing-offsets.yaml"),
            "position_units_per_rad": 1000.0,
            "csp_mode": 8,
            "jog_step_rad": 0.05,
            "max_step_rad": 0.1,
        }
    }
    worker = ArmWorker(config, config_path)
    worker.desire = DesireRegion()
    worker.real = RealRegion()
    worker._load_limits(1000.0)

    try:
        worker.command("JOG", {"joint": "ljoint1", "direction": 1})
    except WorkerError as exc:
        assert "enable" in str(exc)
    else:
        raise AssertionError("arm jog must require explicit enable")

    for slot in range(14):
        worker.real.axis_state[slot].ec_ctrstate = 0x27
        worker.real.axis_state[slot].ec_modestate = 0x08
    worker.command("ENABLE", {})
    worker.command("JOG", {"joint": "ljoint1", "direction": 1, "step_rad": 0.1})
    worker.command("JOG", {"joint": "ljoint1", "direction": 1, "step_rad": 0.1})
    assert worker.targets[0] == 100


def test_protocol_can_round_trip_over_socketpair() -> None:
    left, right = socket.socketpair()
    try:
        message = request(1, "PING")
        left.sendall(encode_frame(message))
        assert recv_frame(right) == message
    finally:
        left.close()
        right.close()
