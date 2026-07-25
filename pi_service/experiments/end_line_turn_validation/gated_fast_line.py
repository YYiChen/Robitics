"""Low-latency white-line follower constrained to the green course field."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np


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
    green_gate_enabled: bool = True
    green_hue_min: int = 32
    green_hue_max: int = 96
    green_saturation_min: int = 95
    green_dilate_radius_px: int = 22
    green_support_inner_px: int = 3
    green_support_outer_px: int = 30
    green_support_min_ratio: float = .25
    green_bottom_anchor_ratio: float = .62
    green_min_component_ratio: float = .015


@dataclass(frozen=True)
class FastLineResult:
    valid: bool
    center_x: float | None
    centers: tuple[tuple[int, float, int], ...]
    confidence: float


@dataclass(frozen=True)
class GatedLineObservation:
    result: FastLineResult
    green_mask: np.ndarray
    course_mask: np.ndarray
    white_mask: np.ndarray


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


def _bottom_connected_green(green: np.ndarray, config: FastLineConfig) -> np.ndarray:
    """Keep only substantial green components connected to the lower course."""
    height, width = green.shape
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(green.astype(np.uint8), connectivity=8)
    min_area = int(height * width * config.green_min_component_ratio)
    anchor_y = int(height * config.green_bottom_anchor_ratio)
    accepted: list[int] = []
    for label in range(1, count):
        _x, y, _component_width, component_height, area = map(int, stats[label])
        if area < min_area:
            continue
        if y + component_height - 1 >= anchor_y:
            accepted.append(label)
    if not accepted and count > 1:
        accepted = [1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))]
    return np.isin(labels, accepted)


def _green_supported(green: np.ndarray, y: int, left: int, right: int, config: FastLineConfig) -> bool:
    """A valid centre tape has visible green cloth on both horizontal sides."""
    height, width = green.shape
    y0, y1 = max(0, y - 3), min(height, y + 4)
    left0, left1 = max(0, left - config.green_support_outer_px), max(0, left - config.green_support_inner_px)
    right0, right1 = min(width, right + 1 + config.green_support_inner_px), min(width, right + 1 + config.green_support_outer_px)
    if left1 <= left0 or right1 <= right0:
        return False
    return (
        float(np.mean(green[y0:y1, left0:left1])) >= config.green_support_min_ratio
        and float(np.mean(green[y0:y1, right0:right1])) >= config.green_support_min_ratio
    )


def analyse_fast_line(frame: np.ndarray, previous_center_x: float | None = None, config: FastLineConfig = FastLineConfig()) -> GatedLineObservation:
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green = (
        (hsv[:, :, 0] >= config.green_hue_min)
        & (hsv[:, :, 0] <= config.green_hue_max)
        & (hsv[:, :, 1] >= config.green_saturation_min)
    )
    if config.green_gate_enabled:
        lower_green = _bottom_connected_green(green, config)
        radius = max(1, int(config.green_dilate_radius_px))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius * 2 + 1, radius * 2 + 1))
        course = cv2.dilate(lower_green.astype(np.uint8), kernel).astype(bool)
    else:
        lower_green = green
        course = np.ones((height, width), dtype=bool)
    white = (hsv[:, :, 2] >= config.white_min_value) & (hsv[:, :, 1] <= config.white_max_saturation)
    candidate = white & course
    target = previous_center_x if previous_center_x is not None else width / 2.0
    centers: list[tuple[int, float, int]] = []
    max_width = int(width * config.max_run_width_ratio)
    for ratio in config.row_ratios:
        y = min(height - 1, max(0, int(height * ratio)))
        choices = []
        for left, right in _runs(candidate[y]):
            run_width = right - left + 1
            if config.min_run_width <= run_width <= max_width and (not config.green_gate_enabled or _green_supported(lower_green, y, left, right, config)):
                center = (left + right) / 2.0
                choices.append((abs(center - target), center, run_width))
        if choices:
            _distance, center, run_width = min(choices, key=lambda item: item[0])
            centers.append((y, center, run_width))
    if len(centers) < 2:
        result = FastLineResult(False, None, tuple(centers), len(centers) / len(config.row_ratios))
    else:
        result = FastLineResult(True, float(np.median([center for _y, center, _width in centers])), tuple(centers), len(centers) / len(config.row_ratios))
    return GatedLineObservation(result, lower_green, course, candidate)


def pwm_for_line(result: FastLineResult, frame_width: int, straight_pwm: int, config: FastLineConfig = FastLineConfig()) -> tuple[int, int] | None:
    if not result.valid or result.center_x is None:
        return None
    offset = (result.center_x - frame_width / 2.0) / max(1.0, frame_width / 2.0)
    if abs(offset) <= config.deadband:
        return straight_pwm, straight_pwm
    headroom = max(0, 255 - abs(int(straight_pwm)))
    correction_cap = min(config.max_correction_pwm, headroom)
    correction = int(round(min(correction_cap, max(config.min_correction_pwm, abs(offset) * config.correction_gain))))
    right, left = ((straight_pwm - correction, straight_pwm + correction) if offset > 0 else (straight_pwm + correction, straight_pwm - correction))
    return max(-255, min(255, int(right))), max(-255, min(255, int(left)))
