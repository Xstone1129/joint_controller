#!/usr/bin/env python3
"""Non-ROS TCP control server for the lower machine.

The initial server deliberately exposes a dry-run backend by default.  The
real EtherCAT workers are selected explicitly after the protocol, lease,
configuration, and mutual-exclusion checks have been validated on the target.
"""

from __future__ import annotations

import argparse
import copy
import logging
from pathlib import Path
import socket
import socketserver
import ssl
import threading
import time
from typing import Any

from config_store import ConfigError, ConfigStore
from mode_guard import HardwareActivityProbe, HardwareBusy, HardwareOwner
from protocol import ProtocolError, recv_frame, send_frame
from workers import HardwareWorkers, WorkerError

LOG = logging.getLogger("hardware_server")


class ServerState:
    def __init__(
        self,
        config: ConfigStore,
        lock_path: Path,
        real_backend: bool = False,
        activity_probe: HardwareActivityProbe | None = None,
    ) -> None:
        self.config = config
        self.lock_path = lock_path
        self.real_backend = real_backend
        self.owner = HardwareOwner(lock_path)
        self.activity_probe = activity_probe or HardwareActivityProbe()
        self.workers: HardwareWorkers | None = None
        server_config = config.snapshot()["config"].get("server", {})
        self.heartbeat_timeout_s = float(server_config.get("heartbeat_timeout_s", 0.75))
        if not 0.25 <= self.heartbeat_timeout_s <= 10.0:
            raise ConfigError("server.heartbeat_timeout_s must be in [0.25, 10]")
        self.lock = threading.RLock()
        self.mode = "IDLE"
        self.direct_active = False
        self.direct_session: str | None = None
        self.lease_session: str | None = None
        self.last_heartbeat: dict[str, float] = {}
        self.server_sequence = 0
        self.config_session: str | None = None
        self._stop_reaper = threading.Event()
        self._reaper = threading.Thread(target=self._watchdog, name="hardware-watchdog", daemon=True)
        self._reaper.start()

    def _watchdog(self) -> None:
        while not self._stop_reaper.wait(0.1):
            with self.lock:
                session = self.lease_session
                if session is None:
                    continue
                if time.monotonic() - self.last_heartbeat.get(session, 0.0) <= self.heartbeat_timeout_s:
                    continue
                LOG.error("heartbeat timeout for lease %s; forcing HOLD and releasing lease", session)
                self._release_direct_locked()

    def _release_direct_locked(self, *, stop_ethercat: bool = False) -> None:
        if self.workers is not None:
            self.workers.stop(stop_ethercat=stop_ethercat)
            self.workers = None
        self.lease_session = None
        self.direct_active = False
        self.direct_session = None
        self.mode = "IDLE"
        self.owner.release()

    def session_closed(self, session: str | None) -> None:
        if session is None:
            return
        with self.lock:
            self.last_heartbeat.pop(session, None)
            if self.direct_session == session or self.lease_session == session:
                LOG.warning("session %s disconnected; forcing HOLD and releasing Direct mode", session)
                self._release_direct_locked()
            if self.config_session == session:
                self.config.rollback()
                self.config_session = None

    def stop(self) -> None:
        self._stop_reaper.set()
        self._reaper.join(timeout=1.0)
        with self.lock:
            self._release_direct_locked(stop_ethercat=True)

    def response(self, request: dict[str, Any], ok: bool = True, **payload: Any) -> dict[str, Any]:
        return {
            "version": 1,
            "request_id": request.get("request_id"),
            "type": "RESPONSE",
            "ok": ok,
            "payload": payload,
        }

    def require_session(self, request: dict[str, Any]) -> str:
        session = request.get("session_id")
        if not isinstance(session, str) or not session:
            raise ConfigError("session_id is required")
        return session

    def require_lease(self, session: str) -> None:
        if self.lease_session != session:
            raise HardwareBusy("control lease is not held by this session")

    def enter_direct(self, session: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        with self.lock:
            if self.direct_active:
                if self.direct_session == session:
                    return {"mode": self.mode, "already_active": True}
                raise HardwareBusy("another Direct session is active")
            self.owner.acquire()
            try:
                self.activity_probe.assert_available()
                if self.real_backend:
                    runtime_config = copy.deepcopy(self.config.snapshot()["config"])
                    lift_test_range = bool((payload or {}).get("lift_test_range", False))
                    if lift_test_range:
                        runtime_config.setdefault("lift", {})["min_position_m"] = -1.0
                        runtime_config.setdefault("lift", {})["max_position_m"] = 1.0
                    self.workers = HardwareWorkers(runtime_config, self.config.path)
                    self.workers.start()
            except Exception:
                if self.workers is not None:
                    self.workers.stop()
                    self.workers = None
                self.owner.release()
                raise
            self.direct_active = True
            self.direct_session = session
            self.mode = "DIRECT_ACTIVE"
            return {
                "mode": self.mode,
                "backend": "real" if self.real_backend else "dry_run",
                "lift_test_range": bool((payload or {}).get("lift_test_range", False)),
            }

    def leave_direct(self, session: str) -> dict[str, Any]:
        with self.lock:
            if self.direct_session not in (None, session):
                raise HardwareBusy("Direct mode belongs to another session")
            self._release_direct_locked()
            return {"mode": self.mode}

    def handle(self, request: dict[str, Any]) -> dict[str, Any]:
        message_type = request.get("type")
        if not isinstance(message_type, str):
            raise ConfigError("type is required")
        session = self.require_session(request)
        with self.lock:
            self.last_heartbeat[session] = time.monotonic()
        if message_type == "HELLO":
            return self.response(request, robot_id="heavy_v1", mode=self.mode, config=self.config.snapshot(), capabilities=self.config.schema())
        if message_type in {"PING", "GET_MODE"}:
            return self.response(request, mode=self.mode, direct_active=self.direct_active, lease=bool(self.lease_session == session))
        if message_type == "GET_CAPABILITIES":
            return self.response(request, robot_id="heavy_v1", backend="real" if self.real_backend else "dry_run", config=self.config.snapshot(), capabilities=self.config.schema())
        if message_type == "GET_ARM_STATE":
            if self.workers is not None:
                return self.response(request, **self.workers.arm.state())
            return self.response(request, backend="dry_run", enabled=False, axes=[
                {"joint": f"{side}joint{i}", "status": "DISABLED", "status_code": 0x40, "mode": 0, "error": 0, "position_rad": 0.0}
                for side in ("l", "r") for i in range(1, 8)
            ])
        if message_type == "GET_LIFT_STATE":
            if self.workers is not None:
                return self.response(request, **self.workers.lift.state())
            return self.response(request, backend="dry_run", enabled=False, process_alive=False)
        if message_type == "ENTER_DIRECT":
            payload = request.get("payload", {})
            if not isinstance(payload, dict):
                raise ConfigError("payload must be an object")
            if "lift_test_range" in payload and not isinstance(payload["lift_test_range"], bool):
                raise ConfigError("lift_test_range must be boolean")
            return self.response(request, **self.enter_direct(session, payload))
        if message_type == "LEAVE_DIRECT":
            return self.response(request, **self.leave_direct(session))
        if message_type == "ACQUIRE_LEASE":
            with self.lock:
                if not self.direct_active or self.direct_session != session:
                    raise HardwareBusy("enter Direct mode before acquiring a lease")
                if self.lease_session not in (None, session):
                    raise HardwareBusy("another session already holds the lease")
                self.lease_session = session
            return self.response(request, lease=True)
        if message_type == "RELEASE_LEASE":
            with self.lock:
                self.require_lease(session)
                self.lease_session = None
            return self.response(request, lease=False)
        if message_type in {"ARM_ENABLE", "ARM_DISABLE", "ARM_HOLD", "LIFT_ENABLE", "LIFT_DISABLE", "LIFT_HOLD", "LIFT_HOME", "LIFT_ZERO", "HOLD_ALL"}:
            with self.lock:
                self.require_lease(session)
            result = self.workers.command(message_type, request.get("payload", {})) if self.workers is not None else {"accepted": True, "backend": "dry_run"}
            return self.response(request, **result)
        if message_type in {"ARM_JOG", "ARM_HOME", "LIFT_JOG", "LIFT_HOME", "LIFT_ZERO"}:
            with self.lock:
                self.require_lease(session)
            payload = request.get("payload", {})
            if not isinstance(payload, dict):
                raise ConfigError("payload must be an object")
            if message_type == "ARM_JOG" and payload.get("joint") not in {f"ljoint{i}" for i in range(1, 8)} | {f"rjoint{i}" for i in range(1, 8)}:
                raise ConfigError("unknown arm joint")
            if message_type in {"ARM_JOG", "LIFT_JOG"} and payload.get("direction") not in (-1, 1):
                raise ConfigError("direction must be -1 or 1")
            result = self.workers.command(message_type, payload) if self.workers is not None else {"accepted": True, "backend": "dry_run"}
            return self.response(request, **result)
        if message_type == "GET_CONFIG":
            return self.response(request, **self.config.snapshot())
        if message_type == "GET_CONFIG_SCHEMA":
            return self.response(request, schema=self.config.schema())
        if message_type == "BEGIN_CONFIG":
            self._require_config_maintenance(session)
            with self.lock:
                if self.config_session not in (None, session):
                    raise HardwareBusy("another session owns the configuration transaction")
                self.config_session = session
            return self.response(request, **self.config.begin())
        if message_type == "PATCH_CONFIG":
            self._require_config_maintenance(session)
            self._require_config_owner(session)
            payload = request.get("payload", {})
            return self.response(request, **self.config.patch(str(payload["path"]), payload.get("value")))
        if message_type == "VALIDATE_CONFIG":
            if self.config_session is not None:
                self._require_config_owner(session)
            return self.response(request, **self.config.validate())
        if message_type == "COMMIT_CONFIG":
            self._require_config_maintenance(session)
            self._require_config_owner(session)
            payload = request.get("payload", {})
            result = self.config.commit(int(payload["expected_revision"]))
            self.config_session = None
            return self.response(request, **result)
        if message_type == "ROLLBACK_CONFIG":
            self._require_config_maintenance(session)
            self._require_config_owner(session)
            result = self.config.rollback()
            self.config_session = None
            return self.response(request, **result)
        if message_type == "GET_CONFIG_HISTORY":
            return self.response(request, history=self.config.history())
        raise ConfigError(f"unsupported message type: {message_type}")

    def _require_config_maintenance(self, session: str) -> None:
        with self.lock:
            if self.mode != "IDLE" or self.lease_session is not None:
                raise HardwareBusy("configuration requires IDLE with no active lease")

    def _require_config_owner(self, session: str) -> None:
        if self.config_session != session:
            raise HardwareBusy("configuration transaction belongs to another session")


class RequestHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        state: ServerState = self.server.state  # type: ignore[attr-defined]
        session: str | None = None
        self.request.settimeout(1.0)
        LOG.info("client connected from %s", self.client_address)
        try:
            while True:
                try:
                    request = recv_frame(self.request)
                except (EOFError, socket.timeout):
                    return
                except (OSError, ProtocolError) as exc:
                    LOG.warning("bad frame from %s: %s", self.client_address, exc)
                    return
                try:
                    request_session = request.get("session_id")
                    if session is None and isinstance(request_session, str):
                        session = request_session
                    elif request_session != session:
                        raise ConfigError("one TCP connection may use only one session_id")
                    LOG.info(
                        "request from %s session=%s type=%s request_id=%s",
                        self.client_address,
                        session,
                        request.get("type"),
                        request.get("request_id"),
                    )
                    response = state.handle(request)
                except (ConfigError, HardwareBusy, WorkerError, KeyError, TypeError, ValueError) as exc:
                    LOG.warning(
                        "request failed from %s session=%s type=%s request_id=%s: %s",
                        self.client_address,
                        session,
                        request.get("type"),
                        request.get("request_id"),
                        exc,
                    )
                    response = state.response(request, ok=False, error=type(exc).__name__, message=str(exc))
                try:
                    send_frame(self.request, response)
                except OSError:
                    return
                if request.get("type") == "LEAVE_DIRECT":
                    return
        finally:
            state.session_closed(session)
            LOG.info("client disconnected from %s session=%s", self.client_address, session)


class ThreadedServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], state: ServerState, tls_context: ssl.SSLContext | None = None) -> None:
        super().__init__(address, RequestHandler)
        self.state = state
        self.tls_context = tls_context

    def get_request(self) -> tuple[socket.socket, Any]:
        sock, address = super().get_request()
        if self.tls_context is None:
            return sock, address
        try:
            return self.tls_context.wrap_socket(sock, server_side=True), address
        except Exception:
            sock.close()
            raise


def build_tls_context(config: dict[str, Any]) -> ssl.SSLContext | None:
    tls = config.get("tls", {})
    if not isinstance(tls, dict) or not bool(tls.get("enabled", False)):
        return None
    required = {name: str(tls.get(name, "")).strip() for name in ("ca_file", "cert_file", "key_file")}
    missing = [name for name, value in required.items() if not value]
    if missing:
        raise ConfigError("mTLS is enabled but these paths are empty: " + ", ".join(missing))
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.verify_mode = ssl.CERT_REQUIRED
    context.load_verify_locations(cafile=required["ca_file"])
    context.load_cert_chain(required["cert_file"], required["key_file"])
    return context


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="non-ROS lower-machine hardware server")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "hardware_io.yaml")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--lock", type=Path, default=Path("/run/lock/junior-hardware-owner.lock"))
    parser.add_argument("--real-backend", action="store_true", help="start the lower-machine EtherCAT workers")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    config = ConfigStore(args.config)
    server_config = config.snapshot()["config"].get("server", {})
    if not isinstance(server_config, dict):
        server_config = {}
    host = args.host or str(server_config.get("listen_address", "0.0.0.0"))
    port = args.port or int(server_config.get("port", 7447))
    state = ServerState(config, args.lock, args.real_backend)
    tls_context = build_tls_context(server_config)
    server = ThreadedServer((host, port), state, tls_context)
    LOG.info("hardware_server listening on %s:%s backend=%s", host, port, "real" if args.real_backend else "dry_run")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        state.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
