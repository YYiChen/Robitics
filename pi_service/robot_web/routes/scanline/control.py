"""Pure scanline evidence to differential M1/M2 PWM conversion."""
from __future__ import annotations

from dataclasses import dataclass
import math
from types import SimpleNamespace
from typing import Protocol

from .config import ScanlineIRouteConfig


class OffsetObservation(Protocol):
    offset: float | None
    valid_bands: int


@dataclass(frozen=True)
class StraightMotorConfig:
    straight_pwm: int = 65
    correction_deadband: float = 0.05
    correction_gain: float = 120.0
    minimum_correction_pwm: int = 20
    maximum_correction_pwm: int = 60


def drive_pwm_for_offset(
    observation: OffsetObservation, config: StraightMotorConfig
) -> tuple[int, int]:
    offset = observation.offset
    if (
        offset is None
        or observation.valid_bands < 3
        or abs(offset) <= config.correction_deadband
    ):
        return config.straight_pwm, config.straight_pwm
    correction = max(
        config.minimum_correction_pwm,
        min(
            config.maximum_correction_pwm,
            math.ceil(abs(float(offset)) * config.correction_gain),
        ),
    )
    inner = max(0, config.straight_pwm - correction)
    outer = min(255, config.straight_pwm + correction)
    return (inner, outer) if offset > 0 else (outer, inner)


def straight_control(
    evidence, frame_width: int, config: ScanlineIRouteConfig
) -> tuple[tuple[int, int], dict[str, object]]:
    offset = (
        None
        if evidence.line_center_x is None
        else (evidence.line_center_x - frame_width / 2.0)
        / max(1.0, frame_width / 2.0)
    )
    valid_bands = len(evidence.line_centers)
    motor = StraightMotorConfig(
        straight_pwm=config.straight_pwm,
        correction_deadband=config.correction_deadband,
        correction_gain=config.correction_gain,
        minimum_correction_pwm=config.minimum_correction_pwm,
        maximum_correction_pwm=config.maximum_correction_pwm,
    )
    pair = drive_pwm_for_offset(
        SimpleNamespace(offset=offset, valid_bands=valid_bands), motor
    )
    raw_correction = None if offset is None else abs(float(offset)) * config.correction_gain
    applied_correction = abs(pair[0] - pair[1]) // 2
    if offset is None:
        mode, limit_reason = "FOLLOW_NO_OFFSET_FALLBACK", "OFFSET_UNAVAILABLE"
    elif valid_bands < 3:
        mode, limit_reason = "FOLLOW_INSUFFICIENT_BANDS_FALLBACK", "VALID_BANDS_BELOW_THREE"
    elif abs(offset) <= config.correction_deadband:
        mode, limit_reason = "FOLLOW_DEADBAND", "OFFSET_INSIDE_DEADBAND"
    else:
        mode = "FOLLOW_CORRECTION"
        unclamped = math.ceil(abs(float(offset)) * config.correction_gain)
        if unclamped < config.minimum_correction_pwm:
            limit_reason = "MINIMUM_CORRECTION_APPLIED"
        elif unclamped > config.maximum_correction_pwm:
            limit_reason = "MAXIMUM_CORRECTION_APPLIED"
        else:
            limit_reason = None
    return pair, {
        "mode": mode,
        "frame_center_x_px": frame_width / 2.0,
        "line_center_x_px": evidence.line_center_x,
        "offset_normalized": offset,
        "valid_bands": valid_bands,
        "deadband": config.correction_deadband,
        "correction_gain": config.correction_gain,
        "raw_correction_pwm": raw_correction,
        "applied_correction_pwm": applied_correction,
        "base_pwm": config.straight_pwm,
        "limit_reason": limit_reason,
        "commanded_right_pwm": pair[0],
        "commanded_left_pwm": pair[1],
    }


def straight_pair(evidence, frame_width: int, config: ScanlineIRouteConfig):
    return straight_control(evidence, frame_width, config)[0]
