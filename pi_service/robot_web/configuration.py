"""Sectioned configuration for the formal Pi service.

Tracked defaults describe the supported shape.  Deployment tuning is written
only to the ignored local override, so source updates cannot replace a tuned
vehicle.
"""
from __future__ import annotations

from copy import deepcopy
import json
import os
from pathlib import Path
import threading


SERVICE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = SERVICE_ROOT / "config" / "defaults.json"
LOCAL_CONFIG_PATH = SERVICE_ROOT / "config" / "local.json"


def _deep_merge(base: dict, override: dict) -> dict:
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _read_json_object(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"配置文件必须是 JSON 对象: {path}")
    return value


class UnifiedConfigStore:
    """Thread-safe defaults + local override store with dotted sections."""

    def __init__(
        self,
        defaults_path: Path = DEFAULT_CONFIG_PATH,
        local_path: Path = LOCAL_CONFIG_PATH,
    ) -> None:
        self.defaults_path = Path(defaults_path).expanduser()
        self.local_path = Path(local_path).expanduser()
        self._lock = threading.RLock()

    @staticmethod
    def _section(document: dict, dotted_name: str) -> dict:
        current: object = document
        for key in dotted_name.split("."):
            if not isinstance(current, dict):
                return {}
            current = current.get(key, {})
        return deepcopy(current) if isinstance(current, dict) else {}

    @staticmethod
    def _set_section(document: dict, dotted_name: str, value: dict) -> None:
        current = document
        keys = dotted_name.split(".")
        for key in keys[:-1]:
            child = current.get(key)
            if not isinstance(child, dict):
                child = {}
                current[key] = child
            current = child
        current[keys[-1]] = deepcopy(value)

    def read_section(self, dotted_name: str) -> dict:
        with self._lock:
            defaults = self._section(_read_json_object(self.defaults_path), dotted_name)
            local = self._section(_read_json_object(self.local_path), dotted_name)
            return _deep_merge(defaults, local)

    def has_local_section(self, dotted_name: str) -> bool:
        with self._lock:
            document = _read_json_object(self.local_path)
            current: object = document
            for key in dotted_name.split("."):
                if not isinstance(current, dict) or key not in current:
                    return False
                current = current[key]
            return isinstance(current, dict)

    def write_section(self, dotted_name: str, value: dict) -> None:
        if not isinstance(value, dict):
            raise ValueError("配置分区必须是 JSON 对象")
        with self._lock:
            document = _read_json_object(self.local_path)
            document.setdefault("schema_version", 1)
            self._set_section(document, dotted_name, value)
            self._atomic_write(document)

    def migrate_section(self, dotted_name: str, legacy_value: dict) -> bool:
        """Import legacy data once; an existing local section always wins."""
        if not isinstance(legacy_value, dict) or not legacy_value:
            return False
        with self._lock:
            if self.has_local_section(dotted_name):
                return False
            document = _read_json_object(self.local_path)
            document.setdefault("schema_version", 1)
            self._set_section(document, dotted_name, legacy_value)
            self._atomic_write(document)
            return True

    def _atomic_write(self, document: dict) -> None:
        self.local_path.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(document, ensure_ascii=False, indent=2) + "\n"
        temporary = self.local_path.with_name(
            f".{self.local_path.name}.{os.getpid()}.tmp"
        )
        with temporary.open("w", encoding="utf-8", newline="\n") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(self.local_path)


def read_legacy_json(path: Path) -> dict:
    """Best-effort reader used only by the one-time startup migration."""
    try:
        return _read_json_object(Path(path))
    except (json.JSONDecodeError, OSError, ValueError):
        return {}


def migrate_formal_legacy_config(store: UnifiedConfigStore) -> None:
    """Import the three formal legacy configuration families once."""
    web_root = Path(__file__).resolve().parent
    drive = read_legacy_json(web_root / "drive_config.json")
    if not drive:
        drive = read_legacy_json(web_root / "robot_config.json")
    store.migrate_section("drive", drive)
    store.migrate_section("camera", read_legacy_json(web_root / "camera_config.json"))

    experiment = SERVICE_ROOT / "experiments" / "end_line_turn_validation"
    route = read_legacy_json(experiment / "end_line_web_tuning.json")
    turn_90 = read_legacy_json(experiment / "turn_90.json")
    turn_180 = read_legacy_json(experiment / "turn_180.json")
    if turn_90:
        route.update({
            "turn_90_pwm": turn_90.get("pwm"),
            "turn_90_step_seconds": turn_90.get("step_seconds"),
        })
    if turn_180:
        route.update({
            "turn_180_pwm": turn_180.get("pwm"),
            "turn_180_step_seconds": turn_180.get("step_seconds"),
        })
    store.migrate_section(
        "routes.end_line",
        {key: value for key, value in route.items() if value is not None},
    )
