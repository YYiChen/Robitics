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
    if action == "STOP":
        return (0.0, 0.0, 0, 0)
    profile = config.profiles.get(action, default_profiles()[action])
    peak = max(1, abs(profile["rf"]), abs(profile["lf"]))
    right = config.target_speed * profile["rf"] / peak
    left = config.target_speed * profile["lf"] / peak
    # V accepts left then right. M3/M4 always remain outside drive profiles.
    return (left, right, 0, 0)
