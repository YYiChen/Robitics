"""Persistent, validated open-loop turn profiles for the current course."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path


@dataclass(frozen=True)
class TurnProfile:
    pwm: int
    step_seconds: float


def load_turn_profile(path: Path, fallback: TurnProfile, *, steps: int = 1) -> TurnProfile:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        pwm = int(payload["pwm"])
        # Migrate the previous total-duration format without making an old
        # 90/180 profile run two/four times longer after this update.
        seconds = float(payload["step_seconds"]) if "step_seconds" in payload else float(payload["preset_seconds"]) / steps
        if not 0 <= pwm <= 255 or not .05 <= seconds <= 20.0:
            raise ValueError("profile value outside safe range")
        return TurnProfile(pwm, seconds)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return fallback


def save_turn_profile(path: Path, profile: TurnProfile) -> None:
    if not 0 <= profile.pwm <= 255 or not .05 <= profile.step_seconds <= 20.0:
        raise ValueError("转向 PWM 或时间超出安全范围")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(asdict(profile), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
