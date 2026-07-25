from __future__ import annotations

import unittest

import cv2
import numpy as np

from end_line_logic import EndLineConfig, EndLineState, EndLineStopPlanner, RedEndBandDetector


def frame(*, red_y: int | None = None, red_height: int = 24) -> np.ndarray:
    image = np.full((480, 640, 3), (45, 140, 55), dtype=np.uint8)
    if red_y is not None:
        cv2.rectangle(image, (70, red_y), (570, red_y + red_height), (0, 0, 255), -1)
    return image


class EndLineLogicTests(unittest.TestCase):
    def test_detects_wide_red_terminal_not_small_red_noise(self):
        detector = RedEndBandDetector()
        image = frame(red_y=280)
        cv2.rectangle(image, (250, 180), (260, 190), (0, 0, 255), -1)
        observation = detector.detect(image)
        self.assertTrue(observation.detected)
        self.assertGreater(observation.span or 0, 450)
        self.assertIsNotNone(observation.angle_degrees)
        self.assertLess(float(observation.angle_degrees), 15)

    def test_red_band_never_stops_a_valid_white_line(self):
        detector = RedEndBandDetector()
        planner = EndLineStopPlanner()
        decision = planner.step(line_valid=True, red_detected=detector.detect(frame(red_y=370, red_height=30)).detected)
        self.assertEqual(decision.state, EndLineState.RED_DIRECTION_LOCKED)
        self.assertFalse(decision.stop)

    def test_unexpected_loss_stops_without_claiming_endpoint(self):
        planner = EndLineStopPlanner(EndLineConfig(line_lost_confirm_frames=2))
        detector = RedEndBandDetector()
        planner.step(line_valid=False, red_detected=detector.detect(frame()).detected)
        decision = planner.step(line_valid=False, red_detected=detector.detect(frame()).detected)
        self.assertEqual(decision.state, EndLineState.STOPPED_UNSAFE_LINE_LOST)

    def test_central_alignment_uses_only_low_distortion_roi(self):
        detector = RedEndBandDetector()
        image = frame()
        cv2.line(image, (320, 170), (320, 400), (0, 0, 255), 18)
        cv2.line(image, (18, 150), (18, 420), (0, 0, 255), 18)
        alignment = detector.detect_central_alignment(image, left_ratio=.25, right_ratio=.75, top_ratio=.20, bottom_ratio=.85, min_area=60)
        self.assertTrue(alignment.detected)
        self.assertIsNotNone(alignment.signed_angle_degrees)
        self.assertLess(abs(float(alignment.signed_angle_degrees)), 3.0)
