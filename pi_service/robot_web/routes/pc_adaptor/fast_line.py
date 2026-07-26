"""Formal Pi-side near-field white-tape follower.

This deliberately examines only three rows near the camera bottom.  It is the
reusable low-latency white-line primitive for current and archived adaptors;
terminal and turning policies belong to their own experiment modules.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class FastLineConfig:
    row_ratios: tuple[float, ...] = (.92, .86, .80)
    white_min_value: int = 145
    white_max_saturation: int = 115
    min_run_width: int = 5
    max_run_width_ratio: float = .30
    deadband: float = .035
    correction_gain: float = 155.0
    min_correction_pwm: int = 18
    max_correction_pwm: int = 180


@dataclass(frozen=True)
class FastLineResult:
    valid: bool
    center_x: float | None
    centers: tuple[tuple[int, float, int], ...]
    confidence: float


def _runs(row) -> Iterable[tuple[int, int]]:
    start = None
    for index, value in enumerate(row):
        if value and start is None:
            start = index
        elif not value and start is not None:
            yield start, index - 1
            start = None
    if start is not None:
        yield start, len(row) - 1


def find_fast_line(frame, previous_center_x: float | None = None, config: FastLineConfig = FastLineConfig()) -> FastLineResult:
    import cv2
    import numpy as np

    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = (hsv[:, :, 2] >= config.white_min_value) & (hsv[:, :, 1] <= config.white_max_saturation)
    target = previous_center_x if previous_center_x is not None else width / 2.0
    centers: list[tuple[int, float, int]] = []
    max_width = int(width * config.max_run_width_ratio)
    for ratio in config.row_ratios:
        y = min(height - 1, max(0, int(height * ratio)))
        choices = []
        for left, right in _runs(mask[y]):
            run_width = right - left + 1
            if config.min_run_width <= run_width <= max_width:
                center = (left + right) / 2.0
                choices.append((abs(center - target), center, run_width))
        if choices:
            _, center, run_width = min(choices, key=lambda item: item[0])
            centers.append((y, center, run_width))
    if len(centers) < 2:
        return FastLineResult(False, None, tuple(centers), len(centers) / len(config.row_ratios))
    center_x = float(np.median([center for _y, center, _w in centers]))
    return FastLineResult(True, center_x, tuple(centers), len(centers) / len(config.row_ratios))


def pwm_for_line(result: FastLineResult, frame_width: int, straight_pwm: int, config: FastLineConfig = FastLineConfig()) -> tuple[int, int] | None:
    if not result.valid or result.center_x is None:
        return None
    offset = (result.center_x - frame_width / 2.0) / max(1.0, frame_width / 2.0)
    if abs(offset) <= config.deadband:
        return straight_pwm, straight_pwm
    # The correction is differential: one wheel receives base + correction.
    # A configuration such as base=85 and max_correction=180 used to yield
    # 265, which the controller correctly rejects and which must not kill the
    # vision worker.  Limit correction to the remaining PWM headroom first.
    headroom = max(0, 255 - abs(int(straight_pwm)))
    correction_cap = min(config.max_correction_pwm, headroom)
    correction = int(round(min(correction_cap, max(config.min_correction_pwm, abs(offset) * config.correction_gain))))
    # Centre to the right means steer right: right wheel slows, left wheel speeds up.
    right, left = ((straight_pwm - correction, straight_pwm + correction) if offset > 0 else (straight_pwm + correction, straight_pwm - correction))
    # Retain a final hard guard for invalid future configuration values.
    return max(-255, min(255, int(right))), max(-255, min(255, int(left)))
