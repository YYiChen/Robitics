"""I-shape route evidence from scanlines, not skeleton branches."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import statistics

import cv2
import numpy as np


@dataclass(frozen=True)
class ScanlineConfig:
    # The terminal bar can fill the middle of the picture while the remaining
    # longitudinal stem exists only close to the car.  Keep all line-proof
    # samples in that near field so the wide bar cannot erase the evidence for
    # the stem that leads into it.
    track_rows: tuple[float, ...] = (0.92, 0.86, 0.80)
    # Scan the complete lower terminal region independently for a wide run.
    # These rows are deliberately separate from ``track_rows``: the bar is
    # endpoint evidence, never a candidate left/right route.
    bar_rows: tuple[float, ...] = (0.58, 0.61, 0.64, 0.67, 0.70, 0.73, 0.76, 0.79, 0.82)
    near_anchor_ratio: float = 0.90
    narrow_width_ratio: float = 0.18
    endpoint_width_ratio: float = 0.32
    endpoint_width_multiplier: float = 3.0
    endpoint_min_y_ratio: float = 0.55
    minimum_track_rows: int = 2
    maximum_center_spread_ratio: float = 0.10


@dataclass(frozen=True)
class ScanlineEvidence:
    confidence: float
    valid_line: bool
    line_lost: bool
    line_center_x: float | None
    line_centers: tuple[tuple[int, float, int], ...]
    endpoint_detected: bool
    endpoint_y: int | None
    endpoint_width: int | None
    normal_tape_width: float | None


@dataclass(frozen=True)
class ScanlineResult:
    evidence: ScanlineEvidence
    component_mask: np.ndarray


class TurnaroundState(str, Enum):
    FOLLOW_STRAIGHT = "FOLLOW_STRAIGHT"
    BAR_MARKED = "BAR_MARKED"
    BRAKE_BEFORE_PIVOT = "BRAKE_BEFORE_PIVOT"
    PIVOT_180 = "PIVOT_180"
    STOP = "STOP"


@dataclass(frozen=True)
class TurnaroundConfig:
    endpoint_confirm_frames: int = 2
    line_lost_confirm_frames: int = 3
    reacquire_confirm_frames: int = 3
    minimum_confidence: float = 0.55
    bar_mark_timeout_seconds: float = 2.0
    brake_seconds: float = 0.15
    pivot_min_seconds: float = 2.5
    pivot_max_seconds: float = 5.0


@dataclass(frozen=True)
class TurnaroundDecision:
    state: TurnaroundState
    reason: str
    endpoint_frames: int
    line_lost_frames: int
    reacquire_frames: int
    pivot_elapsed_seconds: float | None


class IShapeScanlineAnalyzer:
    def __init__(self, config: ScanlineConfig = ScanlineConfig()) -> None:
        self.config = config

    @staticmethod
    def _row_run(mask: np.ndarray, y: int) -> tuple[float, int] | None:
        xs = np.flatnonzero(mask[y] > 0)
        if xs.size == 0:
            return None
        breaks = np.flatnonzero(np.diff(xs) > 1)
        starts = np.r_[0, breaks + 1]
        ends = np.r_[breaks, xs.size - 1]
        groups = [(int(xs[start]), int(xs[end])) for start, end in zip(starts, ends)]
        left, right = max(groups, key=lambda group: group[1] - group[0])
        return (left + right) / 2.0, right - left + 1

    def _near_component(self, binary: np.ndarray) -> np.ndarray:
        height, width = binary.shape
        count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
        candidates: list[tuple[float, int]] = []
        anchor_y = int(round(height * self.config.near_anchor_ratio))
        for label in range(1, count):
            x, y, component_width, component_height, area = stats[label]
            if area < 20 or y + component_height < anchor_y:
                continue
            centre_distance = abs((x + component_width / 2.0) - width / 2.0) / max(1, width)
            candidates.append((centre_distance - min(0.20, area / max(1, width * height)), label))
        if not candidates:
            return np.zeros_like(binary)
        return np.where(labels == min(candidates)[1], 255, 0).astype(np.uint8)

    def analyze(self, frame: np.ndarray) -> ScanlineResult:
        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _threshold, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        component = self._near_component(binary)
        rows: list[tuple[int, float, int]] = []
        for ratio in self.config.track_rows:
            y = min(height - 1, max(0, int(round(height * ratio))))
            run = self._row_run(component, y)
            if run is not None:
                rows.append((y, run[0], run[1]))
        narrow = [item for item in rows if item[2] <= width * self.config.narrow_width_ratio]
        normal_width = float(statistics.median(item[2] for item in narrow)) if narrow else None
        centers = tuple(narrow)
        if len(centers) >= self.config.minimum_track_rows:
            center_values = [item[1] for item in centers]
            spread = max(center_values) - min(center_values)
            valid_line = spread <= width * self.config.maximum_center_spread_ratio
            confidence = min(1.0, len(centers) / len(self.config.track_rows)) * (1.0 - min(1.0, spread / max(1.0, width * self.config.maximum_center_spread_ratio)))
            center_x = float(statistics.median(center_values))
        else:
            valid_line, confidence, center_x = False, 0.0, None
        endpoint_y: int | None = None
        endpoint_width: int | None = None
        required_width = max(width * self.config.endpoint_width_ratio, (normal_width or 1.0) * self.config.endpoint_width_multiplier)
        for ratio in self.config.bar_rows:
            y = min(height - 1, max(0, int(round(height * ratio))))
            run = self._row_run(component, y)
            if run is not None and y >= height * self.config.endpoint_min_y_ratio and run[1] >= required_width:
                endpoint_y, endpoint_width = y, run[1]
                break
        evidence = ScanlineEvidence(
            confidence=float(max(0.0, confidence)), valid_line=valid_line, line_lost=not valid_line,
            line_center_x=center_x, line_centers=centers, endpoint_detected=endpoint_y is not None,
            endpoint_y=endpoint_y, endpoint_width=endpoint_width, normal_tape_width=normal_width,
        )
        return ScanlineResult(evidence, component)


class IShapeTurnaroundPlanner:
    def __init__(self, config: TurnaroundConfig = TurnaroundConfig()) -> None:
        self.config = config
        self.state = TurnaroundState.FOLLOW_STRAIGHT
        self._endpoint_frames = 0
        self._line_lost_frames = 0
        self._reacquire_frames = 0
        self._bar_marked_at: float | None = None
        self._brake_started_at: float | None = None
        self._pivot_started_at: float | None = None

    def step(self, evidence: ScanlineEvidence, now: float) -> TurnaroundDecision:
        usable = evidence.valid_line and evidence.confidence >= self.config.minimum_confidence
        if self.state is TurnaroundState.FOLLOW_STRAIGHT:
            self._endpoint_frames = self._endpoint_frames + 1 if usable and evidence.endpoint_detected else 0
            if self._endpoint_frames >= self.config.endpoint_confirm_frames:
                self.state, self._bar_marked_at, self._line_lost_frames = TurnaroundState.BAR_MARKED, now, 0
                return TurnaroundDecision(self.state, "lower_transverse_bar_marked_follow_until_stem_lost", self._endpoint_frames, 0, 0, None)
            return TurnaroundDecision(self.state, "following_near_anchored_longitudinal_line", self._endpoint_frames, 0, 0, None)
        if self.state is TurnaroundState.BAR_MARKED:
            marked_at = self._bar_marked_at if self._bar_marked_at is not None else now
            if now - marked_at >= self.config.bar_mark_timeout_seconds:
                self.state, self._endpoint_frames, self._line_lost_frames = TurnaroundState.FOLLOW_STRAIGHT, 0, 0
                return TurnaroundDecision(self.state, "bar_mark_timeout_returning_to_follow", 0, 0, 0, None)
            # A missing near longitudinal stem means the car has passed the
            # bar.  Do not require the bar itself to remain visible: after it
            # passes the bottom anchor, it is intentionally absent too.
            self._line_lost_frames = self._line_lost_frames + 1 if evidence.line_lost else 0
            if self._line_lost_frames >= self.config.line_lost_confirm_frames:
                self.state, self._brake_started_at = TurnaroundState.BRAKE_BEFORE_PIVOT, now
                return TurnaroundDecision(self.state, "longitudinal_stem_lost_after_bar_braking", self._endpoint_frames, self._line_lost_frames, 0, 0.0)
            return TurnaroundDecision(self.state, "bar_marked_following_until_stem_lost", self._endpoint_frames, self._line_lost_frames, 0, None)
        if self.state is TurnaroundState.BRAKE_BEFORE_PIVOT:
            started = self._brake_started_at if self._brake_started_at is not None else now
            if now - started < self.config.brake_seconds:
                return TurnaroundDecision(self.state, "braking_before_right_pivot", self._endpoint_frames, self._line_lost_frames, 0, None)
            self.state, self._pivot_started_at, self._reacquire_frames = TurnaroundState.PIVOT_180, now, 0
            return TurnaroundDecision(self.state, "brake_complete_starting_right_pivot", self._endpoint_frames, self._line_lost_frames, 0, 0.0)
        if self.state is TurnaroundState.PIVOT_180:
            started = self._pivot_started_at if self._pivot_started_at is not None else now
            elapsed = max(0.0, now - started)
            if elapsed >= self.config.pivot_max_seconds:
                self.state = TurnaroundState.STOP
                return TurnaroundDecision(self.state, "pivot_timeout_without_longitudinal_reacquire", self._endpoint_frames, self._line_lost_frames, self._reacquire_frames, elapsed)
            if elapsed < self.config.pivot_min_seconds:
                return TurnaroundDecision(self.state, "pivoting_minimum_time", self._endpoint_frames, self._line_lost_frames, 0, elapsed)
            self._reacquire_frames = self._reacquire_frames + 1 if usable and not evidence.endpoint_detected else 0
            if self._reacquire_frames >= self.config.reacquire_confirm_frames:
                self.state, self._endpoint_frames = TurnaroundState.FOLLOW_STRAIGHT, 0
                return TurnaroundDecision(self.state, "longitudinal_line_reacquired", 0, self._line_lost_frames, self._reacquire_frames, elapsed)
            return TurnaroundDecision(self.state, "pivoting_until_longitudinal_reacquire", self._endpoint_frames, self._line_lost_frames, self._reacquire_frames, elapsed)
        return TurnaroundDecision(self.state, "stopped_after_pivot_timeout", self._endpoint_frames, self._line_lost_frames, self._reacquire_frames, None)
