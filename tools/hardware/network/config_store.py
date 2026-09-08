"""Authoritative lower-machine configuration and transactional updates."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import threading
from datetime import datetime, timezone
from typing import Any

import yaml


class ConfigError(ValueError):
    pass


class ConfigStore:
    def __init__(self, path: Path, history_dir: Path | None = None) -> None:
        self.path = path
        self.history_dir = history_dir or path.parent / f"{path.stem}.history"
        self._lock = threading.RLock()
        self._config: dict[str, Any] = {}
        self._revision = 0
        self._transaction: dict[str, Any] | None = None
        self._transaction_base = 0
        self._history: list[dict[str, Any]] = []
        self.reload()
        self._load_history()

    @staticmethod
    def _canonical(config: dict[str, Any]) -> bytes:
        return json.dumps(config, sort_keys=True, ensure_ascii=True, separators=(",", ":")).encode()

    def _hash(self, config: dict[str, Any] | None = None) -> str:
        return hashlib.sha256(self._canonical(config if config is not None else self._config)).hexdigest()

    def reload(self) -> None:
        with self._lock:
            data = yaml.safe_load(self.path.read_text(encoding="utf-8")) or {}
            if not isinstance(data, dict):
                raise ConfigError("hardware configuration root must be a mapping")
            self._config = data
            self._revision = int(data.get("config_revision", 0))
            self._transaction = None

    def _load_history(self) -> None:
        with self._lock:
            history: list[dict[str, Any]] = []
            try:
                entries = sorted(self.history_dir.glob("commit-*.json"))
            except OSError:
                entries = []
            for entry in entries[-20:]:
                try:
                    value = json.loads(entry.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError):
                    continue
                if isinstance(value, dict):
                    history.append(value)
            self._history = history

    @staticmethod
    def _write_atomic(path: Path, data: bytes) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
        try:
            with os.fdopen(fd, "wb") as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, path)
            directory_fd = os.open(path.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if os.path.exists(name):
                os.unlink(name)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {"config": copy.deepcopy(self._config), "revision": self._revision, "hash": self._hash()}

    def schema(self) -> dict[str, Any]:
        return {
            "remote_mutable": [
                "arm.jog_step_rad",
                "arm.max_speed_rad_s",
                "arm.max_step_rad",
                "lift.jog_step_m",
                "lift.max_speed_mps",
                "lift.acceleration_mps2",
            ],
            "read_only": [
                "ethercat",
                "arm.driver_binary",
                "arm.shared_memory",
                "arm.limits_urdf",
                "server",
                "lift.min_position_m",
                "lift.max_position_m",
                "lift.max_step_m",
            ],
            "requires_idle_and_disabled": True,
        }

    def begin(self) -> dict[str, Any]:
        with self._lock:
            if self._transaction is None:
                self._transaction = copy.deepcopy(self._config)
                self._transaction_base = self._revision
            return self.transaction_snapshot()

    def transaction_snapshot(self) -> dict[str, Any]:
        with self._lock:
            config = self._transaction if self._transaction is not None else self._config
            return {"config": copy.deepcopy(config), "base_revision": self._transaction_base, "hash": self._hash(config)}

    @staticmethod
    def _get_parent(config: dict[str, Any], path: str, create: bool = False) -> tuple[dict[str, Any], str]:
        parts = path.split(".")
        if not parts or any(not part or part.startswith("_") for part in parts):
            raise ConfigError("invalid configuration path")
        parent: Any = config
        for part in parts[:-1]:
            if not isinstance(parent, dict):
                raise ConfigError("configuration path does not name a mapping")
            if part not in parent:
                if not create:
                    raise ConfigError(f"unknown configuration path: {path}")
                parent[part] = {}
            parent = parent[part]
        if not isinstance(parent, dict):
            raise ConfigError("configuration path parent is not a mapping")
        return parent, parts[-1]

    def patch(self, path: str, value: Any) -> dict[str, Any]:
        with self._lock:
            if path not in self.schema()["remote_mutable"]:
                raise ConfigError(f"configuration field is not remotely mutable: {path}")
            if self._transaction is None:
                self.begin()
            assert self._transaction is not None
            parent, key = self._get_parent(self._transaction, path)
            if not isinstance(value, (bool, int, float, str)) or isinstance(value, bool):
                raise ConfigError("configuration value must be a scalar number/string")
            parent[key] = value
            self.validate(self._transaction)
            return self.transaction_snapshot()

    def validate(self, config: dict[str, Any] | None = None) -> dict[str, Any]:
        with self._lock:
            candidate = config if config is not None else (self._transaction or self._config)
            arm = candidate.get("arm", {})
            lift = candidate.get("lift", {})
            if not isinstance(arm, dict) or not isinstance(lift, dict):
                raise ConfigError("arm and lift sections are required mappings")
            for key in ("jog_step_rad", "max_step_rad", "max_speed_rad_s"):
                value = float(arm.get(key, 0.0))
                upper = 0.1 if key in {"jog_step_rad", "max_step_rad"} else 1.0
                if not math.isfinite(value) or not 0.0 < value <= upper:
                    raise ConfigError(f"arm.{key} must be in (0, {upper}]")
            if float(arm["jog_step_rad"]) > float(arm["max_step_rad"]):
                raise ConfigError("arm.jog_step_rad must not exceed arm.max_step_rad")
            min_m, max_m = float(lift.get("min_position_m", -1.0)), float(lift.get("max_position_m", 0.0))
            if not min_m < max_m:
                raise ConfigError("lift.min_position_m must be less than max_position_m")
            for key, upper in (("jog_step_m", 0.1), ("max_step_m", 0.1), ("max_speed_mps", 0.1), ("acceleration_mps2", 1.0)):
                value = float(lift.get(key, 0.0))
                if not math.isfinite(value) or not 0.0 < value <= upper:
                    raise ConfigError(f"lift.{key} must be in (0, {upper}]")
            if float(lift["jog_step_m"]) > float(lift["max_step_m"]):
                raise ConfigError("lift.jog_step_m must not exceed lift.max_step_m")
            return {"valid": True, "hash": self._hash(candidate)}

    def commit(self, expected_revision: int) -> dict[str, Any]:
        with self._lock:
            if self._transaction is None:
                raise ConfigError("no configuration transaction")
            if expected_revision != self._revision or self._transaction_base != self._revision:
                raise ConfigError(f"revision conflict: current={self._revision}, expected={expected_revision}")
            self.validate(self._transaction)
            old_hash = self._hash()
            next_config = copy.deepcopy(self._transaction)
            next_revision = self._revision + 1
            next_config["config_revision"] = next_revision
            new_hash = self._hash(next_config)
            self.history_dir.mkdir(parents=True, exist_ok=True)
            backup_name = f"revision-{self._revision:06d}-{old_hash[:12]}.yaml"
            backup_path = self.history_dir / backup_name
            if not backup_path.exists():
                self._write_atomic(backup_path, self.path.read_bytes())
            serialized = yaml.safe_dump(next_config, sort_keys=False).encode("utf-8")
            self._write_atomic(self.path, serialized)
            history_entry = {
                "revision": next_revision,
                "previous_revision": self._revision,
                "old_hash": old_hash,
                "new_hash": new_hash,
                "backup": backup_name,
                "committed_at": datetime.now(timezone.utc).isoformat(),
            }
            history_path = self.history_dir / f"commit-{next_revision:06d}-{new_hash[:12]}.json"
            self._write_atomic(
                history_path,
                (json.dumps(history_entry, ensure_ascii=True, sort_keys=True) + "\n").encode("utf-8"),
            )
            self._history.append(history_entry)
            self._history = self._history[-20:]
            self._config, self._revision, self._transaction = next_config, next_revision, None
            return self.snapshot()

    def rollback(self) -> dict[str, Any]:
        with self._lock:
            self._transaction = None
            return self.snapshot()

    def history(self) -> list[dict[str, Any]]:
        with self._lock:
            return copy.deepcopy(self._history)
