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


@dataclass(frozen=True)
class RedAlignmentObservation:
    """Central-ROI red-line pose used only for 90-degree closed-loop turns."""
    detected: bool
    center_x: int | None = None
    center_y: int | None = None
    signed_angle_degrees: float | None = None  # 0 = vertical, sign = lean side
    length_px: float = 0.0


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

    def _red_mask(self, frame: np.ndarray) -> np.ndarray:
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
        return mask

    def detect(self, frame: np.ndarray) -> RedEndBandObservation:
        height, width = frame.shape[:2]
        mask = self._red_mask(frame)
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

    def detect_central_alignment(self, frame: np.ndarray, *, left_ratio: float, right_ratio: float, top_ratio: float, bottom_ratio: float, min_area: int) -> RedAlignmentObservation:
        """Fit the red line only in the optical centre, avoiding fish-eye edges."""
        height, width = frame.shape[:2]
        left, right = int(width * left_ratio), int(width * right_ratio)
        top, bottom = int(height * top_ratio), int(height * bottom_ratio)
        if not (0 <= left < right <= width and 0 <= top < bottom <= height):
            return RedAlignmentObservation(False)
        mask = self._red_mask(frame)
        central = np.zeros_like(mask)
        central[top:bottom, left:right] = mask[top:bottom, left:right]
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(central, connectivity=8)
        candidates = []
        for label in range(1, count):
            x, y, component_width, component_height, area = map(int, stats[label])
            length = max(component_width, component_height)
            if area >= min_area and length >= 24:
                candidates.append((area, length, centroids[label], label))
        if not candidates:
            return RedAlignmentObservation(False)
        area, length, centroid, label = max(candidates, key=lambda item: (item[1], item[0]))
        points = np.column_stack(np.where(labels == label))[:, ::-1].astype(np.float32)
        vx, vy, _x0, _y0 = (float(value) for value in cv2.fitLine(points, cv2.DIST_L2, 0, .01, .01).reshape(-1))
        # Orient the undirected line toward the top of the image, so its lean
        # around vertical has a stable sign in the central, low-distortion ROI.
        if vy > 0:
            vx, vy = -vx, -vy
        signed_angle = float(np.degrees(np.arctan2(vx, -vy)))
        return RedAlignmentObservation(True, int(round(centroid[0])), int(round(centroid[1])), signed_angle, float(length))


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
