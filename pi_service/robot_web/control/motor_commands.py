"""Pure conversion from route actions to Arduino motor command values."""
from __future__ import annotations

from .drive_config import Config, default_profiles


def raw_motor_output(action: str, config: Config) -> tuple[int, int, int, int]:
    if action == "STOP":
        return (0, 0, 0, 0)
    profile = config.profiles.get(action, default_profiles()[action])
    # M1/M2 are right/left drive; M3/M4 belong to the card mechanism.
    return (profile["rf"], profile["lf"], 0, 0)


def speed_targets(action: str, config: Config) -> tuple[float, float, int, int]:
    target, half = config.target_speed, config.target_speed * .5
    return {
        "F": (target, target, 0, 0), "SF": (target * .5, target * .5, 0, 0),
        "B": (-target, -target, 0, 0),
        "PL": (-target, target, -config.pivot_pwm, config.pivot_pwm),
        "PR": (target, -target, config.pivot_pwm, -config.pivot_pwm),
        "SPL": (-half, half, -config.pivot_pwm, config.pivot_pwm),
        "SPR": (half, -half, config.pivot_pwm, -config.pivot_pwm),
        "FL": (half, target, config.curve_inner_pwm, config.curve_outer_pwm),
        "FR": (target, half, config.curve_outer_pwm, config.curve_inner_pwm),
        "BL": (-target, -half, -config.curve_inner_pwm, -config.curve_outer_pwm),
        "BR": (-half, -target, -config.curve_outer_pwm, -config.curve_inner_pwm),
        "STOP": (0, 0, 0, 0),
    }[action]
