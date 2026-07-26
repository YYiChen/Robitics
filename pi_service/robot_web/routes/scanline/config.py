"""Scanline route tuning schema and legacy-file loader."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path


@dataclass(frozen=True)
class ScanlineIRouteConfig:
    process_fps: float = 20.0
    straight_pwm: int = 120
    pivot_pwm: int = 200
    correction_deadband: float = 0.05
    correction_gain: float = 120.0
    minimum_correction_pwm: int = 20
    maximum_correction_pwm: int = 60
    pivot_min_seconds: float = 2.5
    pivot_max_seconds: float = 5.0
    bar_mark_timeout_seconds: float = 4.0
    early_junction_trigger_y_ratio: float = 0.75
    early_line_lost_confirm_frames: int = 1
    red_exit_arm_y_ratio: float = 0.84
    tuning_path: Path | None = None
    use_hybrid: bool = True


SCANLINE_TUNING_FIELDS = {
    "straight_pwm": (int, 0, 255),
    "pivot_pwm": (int, 0, 255),
    "correction_deadband": (float, 0.0, 1.0),
    "correction_gain": (float, 0.0, 1000.0),
    "minimum_correction_pwm": (int, 0, 255),
    "maximum_correction_pwm": (int, 0, 255),
    "pivot_min_seconds": (float, 0.0, 20.0),
    "pivot_max_seconds": (float, 0.1, 30.0),
    "bar_mark_timeout_seconds": (float, 0.2, 15.0),
    "early_junction_trigger_y_ratio": (float, 0.35, 0.98),
    "early_line_lost_confirm_frames": (int, 1, 10),
    "red_exit_arm_y_ratio": (float, 0.60, 0.98),
}


def load_scanline_tuning_config(tuning_path: Path) -> ScanlineIRouteConfig:
    values: dict[str, object] = {}
    if tuning_path.exists():
        try:
            stored = json.loads(tuning_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            stored = {}
        if isinstance(stored, dict):
            for field, (converter, low, high) in SCANLINE_TUNING_FIELDS.items():
                try:
                    value = converter(stored[field])
                except (KeyError, TypeError, ValueError):
                    continue
                if low <= value <= high:
                    values[field] = value
    return ScanlineIRouteConfig(tuning_path=tuning_path, **values)
