"""Green-floor / white-tape mask for the proven I-shape scanline state machine."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

import cv2
import numpy as np


BLACK_LINE_EXPERIMENT = Path(__file__).resolve().parents[1] / "i_shape_scanline_turnaround_validation"
if str(BLACK_LINE_EXPERIMENT) not in sys.path:
    sys.path.insert(0, str(BLACK_LINE_EXPERIMENT))

from scanline_i_logic import HybridScanlineAnalyzer, HybridScanlineConfig  # noqa: E402


@dataclass(frozen=True)
class GreenWhiteScanlineConfig(HybridScanlineConfig):
    """HSV thresholds verified by the fixed green-course detector config."""
    white_saturation_max: int = 82
    white_value_min: int = 168
    green_hue_min: int = 32
    green_hue_max: int = 96
    green_saturation_min: int = 45
    green_value_min: int = 28
    minimum_green_roi_ratio: float = 0.18
    roi_top_ratio: float = 0.38
    green_neighbour_kernel: int = 31


class GreenWhiteHybridScanlineAnalyzer(HybridScanlineAnalyzer):
    """Reuse all I-turn geometry; replace only black-tape Otsu segmentation."""
    def __init__(self, config: GreenWhiteScanlineConfig = GreenWhiteScanlineConfig()) -> None:
        super().__init__(config)
        self.config = config

    @staticmethod
    def _odd(value: int) -> int:
        return value if value % 2 else value + 1

    def _make_mask(self, frame: np.ndarray, blur_kernel: int, morphology_kernel: int) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        config = self.config
        white = cv2.inRange(
            hsv,
            np.array((0, 0, config.white_value_min), dtype=np.uint8),
            np.array((180, config.white_saturation_max, 255), dtype=np.uint8),
        )
        green = cv2.inRange(
            hsv,
            np.array((config.green_hue_min, config.green_saturation_min, config.green_value_min), dtype=np.uint8),
            np.array((config.green_hue_max, 255, 255), dtype=np.uint8),
        )
        height = frame.shape[0]
        roi_start = min(height - 1, max(0, int(round(height * config.roi_top_ratio))))
        green_roi_ratio = float(np.count_nonzero(green[roi_start:])) / max(1, green[roi_start:].size)
        if green_roi_ratio < config.minimum_green_roi_ratio:
            return np.zeros_like(white)
        # White tape replaces the green beneath it.  Keep white pixels only
        # when they are immediately surrounded by the expected green floor.
        neighbour_size = self._odd(max(3, config.green_neighbour_kernel))
        neighbour = cv2.dilate(green, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (neighbour_size, neighbour_size)))
        mask = cv2.bitwise_and(white, neighbour)
        cleanup_size = self._odd(max(3, morphology_kernel))
        cleanup = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cleanup_size, cleanup_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cleanup)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cleanup)
