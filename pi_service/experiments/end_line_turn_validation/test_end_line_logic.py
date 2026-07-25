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

    def test_confirmed_band_stops_when_its_bottom_enters_stop_zone(self):
        detector = RedEndBandDetector()
        planner = EndLineStopPlanner()
        planner.step(line_valid=True, red_band=detector.detect(frame(red_y=280)), frame_height=480)
        decision = planner.step(line_valid=True, red_band=detector.detect(frame(red_y=370, red_height=30)), frame_height=480)
        self.assertEqual(decision.state, EndLineState.STOPPED_RED_BAND)
        self.assertTrue(decision.stop)

    def test_unexpected_loss_stops_without_claiming_endpoint(self):
        planner = EndLineStopPlanner(EndLineConfig(line_lost_confirm_frames=2))
        detector = RedEndBandDetector()
        planner.step(line_valid=False, red_band=detector.detect(frame()), frame_height=480)
        decision = planner.step(line_valid=False, red_band=detector.detect(frame()), frame_height=480)
        self.assertEqual(decision.state, EndLineState.STOPPED_UNSAFE_LINE_LOST)
