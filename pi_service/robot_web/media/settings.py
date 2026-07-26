"""Camera settings validation and persistence independent of camera hardware."""
from __future__ import annotations

import json
from pathlib import Path

from .profiles import (
    CAMERA_MODES,
    DEFAULT_CAMERA_MODE,
    DEFAULT_COLOR_CORRECTION,
    DEFAULT_EXPOSURE,
    DEFAULT_HIGHRES_FPS,
    DEFAULT_HIGHRES_PROFILE,
    DEFAULT_STREAM_PROFILE,
    HIGHRES_PROFILES,
    MAX_HIGHRES_FPS,
    MIN_HIGHRES_FPS,
    STREAM_PROFILES,
)


def normalize_camera_settings(data: object) -> dict:
    settings = {
        "mode": DEFAULT_CAMERA_MODE,
        "exposure": dict(DEFAULT_EXPOSURE),
        "stream_profile": DEFAULT_STREAM_PROFILE,
        "highres_profile": DEFAULT_HIGHRES_PROFILE,
        "highres_fps": DEFAULT_HIGHRES_FPS,
        "color_correction": dict(DEFAULT_COLOR_CORRECTION),
    }
    if not isinstance(data, dict):
        return settings
    if data.get("mode") in CAMERA_MODES:
        settings["mode"] = data["mode"]
    if data.get("stream_profile") in STREAM_PROFILES:
        settings["stream_profile"] = data["stream_profile"]
    if data.get("highres_profile") in HIGHRES_PROFILES:
        settings["highres_profile"] = data["highres_profile"]
    try:
        settings["highres_fps"] = max(
            MIN_HIGHRES_FPS,
            min(MAX_HIGHRES_FPS, float(data.get("highres_fps", DEFAULT_HIGHRES_FPS))),
        )
        exposure = data.get("exposure", {})
        if isinstance(exposure, dict):
            settings["exposure"]["auto"] = bool(exposure.get("auto", settings["exposure"]["auto"]))
            settings["exposure"]["ev"] = max(-8.0, min(8.0, float(exposure.get("ev", settings["exposure"]["ev"]))))
            settings["exposure"]["shutter_denominator"] = max(1, int(exposure.get("shutter_denominator", settings["exposure"]["shutter_denominator"])))
        correction = data.get("color_correction", {})
        if isinstance(correction, dict):
            settings["color_correction"]["enabled"] = bool(correction.get("enabled", settings["color_correction"]["enabled"]))
            settings["color_correction"]["strength"] = max(0.0, min(1.5, float(correction.get("strength", settings["color_correction"]["strength"]))))
    except (TypeError, ValueError):
        pass
    return settings


class CameraSettingsRepository:
    def __init__(self, path: Path, config_store=None) -> None:
        self.path = Path(path).expanduser()
        self.config_store = config_store

    def read_raw(self) -> dict:
        try:
            value = (
                self.config_store.read_section("camera")
                if self.config_store is not None
                else json.loads(self.path.read_text(encoding="utf-8"))
            )
            return value if isinstance(value, dict) else {}
        except (FileNotFoundError, json.JSONDecodeError, OSError, TypeError, ValueError):
            return {}

    def load(self) -> dict:
        return normalize_camera_settings(self.read_raw())

    def save(self, payload: dict) -> None:
        if self.config_store is not None:
            self.config_store.write_section("camera", payload)
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(f".{self.path.name}.tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.path)
