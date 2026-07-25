"""Straight white line and red direction-band logic.

This deliberately contains no skeleton, T-junction, transverse-white-bar,
two-red-band, PC-offload, or 180-degree turnaround behaviour.  The red band
is a direction observation only; white-line disappearance triggers the stop.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import cv2
import numpy as np


@dataclass(frozen=True)
class EndLineConfig:
    red_channel_min: int = 120
    red_excess_min: int = 44
    red_roi_top_ratio: float = .18
    red_roi_side_ratio: float = .04
    red_min_component_area: int = 100
    red_min_span_ratio: float = .25
    line_lost_confirm_frames: int = 3
    red_direction_memory_frames: int = 30
    brake_hold_seconds: float = .18


@dataclass(frozen=True)
class RedEndBandObservation:
    detected: bool
    center_x: int | None = None
    y: int | None = None
    bottom_y: int | None = None
    span: int | None = None
    area: int = 0
    angle_degrees: float | None = None  # long axis: 0 horizontal, 90 vertical


class EndLineState(str, Enum):
    FOLLOW_LINE = "FOLLOW_LINE"
    RED_DIRECTION_LOCKED = "RED_DIRECTION_LOCKED"
    STOPPED_LINE_END = "STOPPED_LINE_END"
    STOPPED_UNSAFE_LINE_LOST = "STOPPED_UNSAFE_LINE_LOST"


@dataclass(frozen=True)
class EndLineDecision:
    state: EndLineState
    stop: bool
    reason: str


class RedEndBandDetector:
    """Detect the single wide red terminal band without treating it as tape."""

    def __init__(self, config: EndLineConfig = EndLineConfig()) -> None:
        self.config = config

    def detect(self, frame: np.ndarray) -> RedEndBandObservation:
        height, width = frame.shape[:2]
        blue, green, red_channel = cv2.split(frame)
        red_excess = red_channel.astype(np.int16) - np.maximum(blue, green).astype(np.int16)
        mask = np.where(
            (red_channel >= self.config.red_channel_min) & (red_excess >= self.config.red_excess_min), 255, 0
        ).astype(np.uint8)
        top = int(height * self.config.red_roi_top_ratio)
        side = int(width * self.config.red_roi_side_ratio)
        mask[:top, :] = 0
        mask[:, :side] = 0
        mask[:, width - side:] = 0
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        count, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        candidates = []
        for label in range(1, count):
            x, y, component_width, component_height, area = map(int, stats[label])
            if area < self.config.red_min_component_area or component_width < width * self.config.red_min_span_ratio:
                continue
            candidates.append((area, x, y, component_width, component_height, centroids[label], label))
        if not candidates:
            return RedEndBandObservation(False)
        area, x, y, component_width, component_height, centroid, label = max(candidates, key=lambda item: item[0])
        points = np.column_stack(np.where(_labels == label))[:, ::-1].astype(np.float32)
        (_centre, (rect_width, rect_height), angle) = cv2.minAreaRect(points)
        long_angle = float(angle + (90.0 if rect_height > rect_width else 0.0)) % 180.0
        if long_angle > 90.0:
            long_angle = 180.0 - long_angle
        return RedEndBandObservation(True, int(round(centroid[0])), int(round(centroid[1])), y + component_height - 1, component_width, area, long_angle)


class EndLineStopPlanner:
    """Stop only after a confirmed white-line loss; red never triggers motion."""

    def __init__(self, config: EndLineConfig = EndLineConfig()) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._line_lost_frames = 0
        self._stopped: EndLineDecision | None = None

    def step(self, *, line_valid: bool, red_detected: bool) -> EndLineDecision:
        if self._stopped is not None:
            return self._stopped
        self._line_lost_frames = 0 if line_valid else self._line_lost_frames + 1
        if self._line_lost_frames >= self.config.line_lost_confirm_frames:
            if red_detected:
                self._stopped = EndLineDecision(EndLineState.STOPPED_LINE_END, True, "white_line_lost_after_red_direction_seen")
            else:
                self._stopped = EndLineDecision(EndLineState.STOPPED_UNSAFE_LINE_LOST, True, "white_line_lost_without_recent_red_direction")
        elif red_detected:
            return EndLineDecision(EndLineState.RED_DIRECTION_LOCKED, False, "red_direction_recorded_keep_following")
        else:
            return EndLineDecision(EndLineState.FOLLOW_LINE, False, "following_single_white_line")
        return self._stopped
