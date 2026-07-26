"""Validated drive configuration and wheel-profile normalization."""
from __future__ import annotations

from dataclasses import dataclass, field


PROFILE_ACTIONS = ("F", "SF", "B", "PL", "PR", "SPL", "SPR", "FL", "FR", "BL", "BR")
WHEELS = ("rf", "lf", "lr", "rr")


def default_profiles() -> dict[str, dict[str, int]]:
    return {
        "F": {"rf": 255, "lf": 255, "lr": 255, "rr": 255}, "SF": {"rf": 100, "lf": 100, "lr": 100, "rr": 100}, "B": {"rf": -255, "lf": -255, "lr": -255, "rr": -255},
        "PL": {"rf": 180, "lf": -180, "lr": -180, "rr": 180}, "PR": {"rf": -180, "lf": 180, "lr": 180, "rr": -180},
        "SPL": {"rf": 120, "lf": -120, "lr": -120, "rr": 120}, "SPR": {"rf": -120, "lf": 120, "lr": 120, "rr": -120},
        "FL": {"rf": 255, "lf": 60, "lr": 60, "rr": 255}, "FR": {"rf": 60, "lf": 255, "lr": 255, "rr": 60},
        "BL": {"rf": -255, "lf": -60, "lr": -60, "rr": -255}, "BR": {"rf": -60, "lf": -255, "lr": -255, "rr": -60},
    }


def normalize_profiles(raw: object) -> dict[str, dict[str, int]]:
    profiles = default_profiles()
    if not isinstance(raw, dict):
        return profiles
    for action in PROFILE_ACTIONS:
        values = raw.get(action)
        if not isinstance(values, dict):
            continue
        for wheel in WHEELS:
            try:
                profiles[action][wheel] = max(
                    -255, min(255, int(values.get(wheel, profiles[action][wheel])))
                )
            except (TypeError, ValueError):
                pass
    return profiles


@dataclass
class Config:
    speed_mode: bool = False
    target_speed: float = 30.0
    kp: float = 2.0
    ki: float = 0.8
    kd: float = 0.05
    straight_pwm: int = 80
    pivot_pwm: int = 150
    curve_outer_pwm: int = 160
    curve_inner_pwm: int = 60
    servo_center_angle: int = 90
    servo_speed_dps: float = 45.0
    servo_acceleration_dps2: float = 120.0
    servo_qe_reversed: bool = True
    profiles: dict[str, dict[str, int]] = field(default_factory=default_profiles)


def legacy_scalar_profiles(config: Config) -> dict[str, dict[str, int]]:
    """Translate pre-profile PWM settings into the current action profiles."""
    straight = max(0, min(255, int(config.straight_pwm)))
    pivot = max(0, min(255, int(config.pivot_pwm)))
    outer = max(0, min(255, int(config.curve_outer_pwm)))
    inner = max(0, min(255, int(config.curve_inner_pwm)))
    profiles = default_profiles()
    profiles.update({
        "F": {wheel: straight for wheel in WHEELS},
        "B": {wheel: -straight for wheel in WHEELS},
        "PL": {"rf": pivot, "lf": -pivot, "lr": -pivot, "rr": pivot},
        "PR": {"rf": -pivot, "lf": pivot, "lr": pivot, "rr": -pivot},
        "FL": {"rf": outer, "lf": inner, "lr": inner, "rr": outer},
        "FR": {"rf": inner, "lf": outer, "lr": outer, "rr": inner},
        "BL": {"rf": -inner, "lf": -outer, "lr": -outer, "rr": -inner},
        "BR": {"rf": -outer, "lf": -inner, "lr": -inner, "rr": -outer},
    })
    return profiles
