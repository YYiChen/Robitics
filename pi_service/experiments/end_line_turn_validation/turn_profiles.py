"""Persistent, validated open-loop turn profiles for the current course."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path


@dataclass(frozen=True)
class TurnProfile:
    pwm: int
    preset_seconds: float


def load_turn_profile(path: Path, fallback: TurnProfile) -> TurnProfile:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        pwm, seconds = int(payload["pwm"]), float(payload["preset_seconds"])
        if not 0 <= pwm <= 255 or not .05 <= seconds <= 20.0:
            raise ValueError("profile value outside safe range")
        return TurnProfile(pwm, seconds)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return fallback


def save_turn_profile(path: Path, profile: TurnProfile) -> None:
    if not 0 <= profile.pwm <= 255 or not .05 <= profile.preset_seconds <= 20.0:
        raise ValueError("转向 PWM 或时间超出安全范围")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(asdict(profile), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
