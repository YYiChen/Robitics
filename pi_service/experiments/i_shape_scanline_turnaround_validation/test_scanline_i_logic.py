import unittest

import cv2
import numpy as np

from scanline_i_logic import IShapeScanlineAnalyzer, IShapeTurnaroundPlanner, TurnaroundConfig, TurnaroundState


def frame(*, endpoint=False, lower_stem_only=False):
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    stem_top = 336 if lower_stem_only else 80
    cv2.line(image, (320, 479), (320, stem_top), (0, 0, 0), 20)
    if endpoint:
        cv2.line(image, (100, 336), (540, 336), (0, 0, 0), 20)
    return image


class ScanlineIShapeTests(unittest.TestCase):
    def test_vertical_tape_remains_the_main_line(self):
        evidence = IShapeScanlineAnalyzer().analyze(frame()).evidence
        self.assertTrue(evidence.valid_line)
        self.assertFalse(evidence.endpoint_detected)
        self.assertAlmostEqual(evidence.line_center_x, 320, delta=3)

    def test_lower_wide_bar_is_endpoint_not_a_left_or_right_path(self):
        evidence = IShapeScanlineAnalyzer().analyze(frame(endpoint=True)).evidence
        self.assertTrue(evidence.valid_line)
        self.assertTrue(evidence.endpoint_detected)
        self.assertGreater(evidence.endpoint_width or 0, 300)

    def test_near_stem_still_confirms_bar_when_it_ends_at_the_bar(self):
        evidence = IShapeScanlineAnalyzer().analyze(frame(endpoint=True, lower_stem_only=True)).evidence
        self.assertTrue(evidence.valid_line)
        self.assertGreaterEqual(evidence.confidence, 0.55)
        self.assertTrue(evidence.endpoint_detected)
        self.assertAlmostEqual(evidence.line_center_x, 320, delta=3)

    def test_confirmed_bar_forces_pivot_before_reacquiring_vertical_line(self):
        analyzer = IShapeScanlineAnalyzer()
        planner = IShapeTurnaroundPlanner(TurnaroundConfig(endpoint_confirm_frames=2, reacquire_confirm_frames=2, pivot_min_seconds=1, pivot_max_seconds=5))
        # Matches the live camera scene: only the lower stem remains visible
        # beneath the transverse endpoint bar.
        bar = analyzer.analyze(frame(endpoint=True, lower_stem_only=True)).evidence
        straight = analyzer.analyze(frame()).evidence
        self.assertIs(planner.step(bar, 0).state, TurnaroundState.FOLLOW_STRAIGHT)
        self.assertIs(planner.step(bar, .1).state, TurnaroundState.PIVOT_180)
        self.assertIs(planner.step(straight, .5).state, TurnaroundState.PIVOT_180)
        self.assertIs(planner.step(straight, 1.1).state, TurnaroundState.PIVOT_180)
        self.assertIs(planner.step(straight, 1.2).state, TurnaroundState.FOLLOW_STRAIGHT)
