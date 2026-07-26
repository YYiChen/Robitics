from __future__ import annotations

import unittest

import cv2
import numpy as np

from pi_fast_red import PiFastRedBandPlanner


def red_frame(*, bands: tuple[tuple[int, int], ...]) -> np.ndarray:
    image = np.full((480, 640, 3), (55, 150, 45), dtype=np.uint8)
    for y, bottom in bands:
        cv2.rectangle(image, (190, y), (300, bottom), (0, 0, 255), -1)
        cv2.rectangle(image, (340, y), (450, bottom), (0, 0, 255), -1)
    return image


class PiFastRedTests(unittest.TestCase):
    def test_groups_skewed_fragments_and_ignores_side_roi(self):
        planner = PiFastRedBandPlanner()
        image = red_frame(bands=((250, 270),))
        cv2.rectangle(image, (5, 300), (100, 330), (0, 0, 255), -1)
        layers = planner.detect_layers(image)
        self.assertEqual(len(layers), 1)
        self.assertGreater(layers[0].span, 200)

    def test_brakes_then_pivots_on_near_band(self):
        planner = PiFastRedBandPlanner()
        self.assertEqual(planner.step(red_frame(bands=((180, 200), (250, 270))),)[0].event, "SLOW_DOWN")
        self.assertEqual(planner.step(red_frame(bands=((245, 255),)),)[0].event, "BRAKE_NOW")
        self.assertEqual(planner.step(red_frame(bands=((380, 420),)),)[0].event, "PIVOT_REQUEST")

    def test_armed_band_exit_requests_reverse(self):
        planner = PiFastRedBandPlanner()
        planner.step(red_frame(bands=((180, 200), (250, 270))))
        planner.step(red_frame(bands=((300, 310),)))
        self.assertEqual(planner.step(red_frame(bands=()),)[0].event, "REVERSE_REQUEST")
