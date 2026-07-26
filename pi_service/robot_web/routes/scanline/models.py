"""Shared scanline evidence and turnaround state models."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


@dataclass(frozen=True)
class ScanlineConfig:
    # The terminal bar can fill the middle of the picture while the remaining
    # longitudinal stem exists only close to the car.  Keep all line-proof
    # samples in that near field so the wide bar cannot erase the evidence for
    # the stem that leads into it.
    track_rows: tuple[float, ...] = (0.92, 0.86, 0.80)
    # Scan from the middle distance down to the terminal region independently
    # for a wide run.  The bar must be seen early enough for mark -> stem-loss
    # confirmation before the vehicle reaches it.
    # These rows are deliberately separate from ``track_rows``: the bar is
    # endpoint evidence, never a candidate left/right route.
    bar_rows: tuple[float, ...] = (0.38, 0.41, 0.44, 0.47, 0.50, 0.53, 0.56, 0.59, 0.62, 0.65, 0.68, 0.71, 0.74, 0.77, 0.80, 0.83)
    near_anchor_ratio: float = 0.90
    narrow_width_ratio: float = 0.18
    endpoint_width_ratio: float = 0.32
    endpoint_width_multiplier: float = 3.0
    endpoint_min_y_ratio: float = 0.35
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
    # ---- Skeleton-augmented fields (HybridScanlineAnalyzer) ----
    lookahead_x: float | None = None
    lookahead_y: int | None = None
    path_length_px: int = 0
    junction_detected: bool = False
    junction_y: int | None = None
    junction_arm_count: int = 0
    red_marker_detected: bool = False
    red_marker_y: int | None = None
    red_marker_span: int | None = None
    frame_height: int = 0


@dataclass(frozen=True)
class ScanlineResult:
    evidence: ScanlineEvidence
    component_mask: Any


class TurnaroundState(str, Enum):
    FOLLOW_STRAIGHT = "FOLLOW_STRAIGHT"
    EARLY_BAR_PREDICTED = "EARLY_BAR_PREDICTED"
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
    bar_mark_timeout_seconds: float = 4.0
    brake_seconds: float = 0.15
    pivot_min_seconds: float = 2.5
    pivot_max_seconds: float = 5.0
    # ---- Hybrid early-prediction fields ----
    # Once the most recently observed junction is this close to the bottom of
    # the frame, BAR_MARKED may use the faster stem-loss confirmation.
    early_junction_trigger_y_ratio: float = 0.75
    early_line_lost_confirm_frames: int = 1
    junction_confirm_frames: int = 2
    # Fixed-course red-marker exit trigger.  Disabled by default so the
    # original black-tape experiment keeps its existing behaviour.
    red_exit_enabled: bool = False
    red_exit_arm_y_ratio: float = 0.84
    red_exit_confirm_frames: int = 1


@dataclass(frozen=True)
class TurnaroundDecision:
    state: TurnaroundState
    reason: str
    endpoint_frames: int
    line_lost_frames: int
    reacquire_frames: int
    pivot_elapsed_seconds: float | None
    # ---- Hybrid ----
    junction_frames: int = 0
