from __future__ import annotations

import unittest

import cv2
import numpy as np

from gated_fast_line import FastLineConfig, analyse_fast_line


def course_frame(*, white_line: bool = True, floor_white: bool = False) -> np.ndarray:
    image = np.full((480, 640, 3), (180, 180, 180), dtype=np.uint8)
    cv2.rectangle(image, (100, 120), (540, 479), (45, 155, 45), -1)
    if white_line:
        cv2.line(image, (320, 479), (320, 150), (255, 255, 255), 18)
    if floor_white:
        cv2.rectangle(image, (0, 360), (90, 479), (255, 255, 255), -1)
    return image


class GatedFastLineTests(unittest.TestCase):
    def test_follows_white_tape_surrounded_by_green_cloth(self):
        result = analyse_fast_line(course_frame()).result
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.center_x or 0, 320, delta=3)

    def test_rejects_white_floor_outside_green_course(self):
        result = analyse_fast_line(course_frame(white_line=False, floor_white=True)).result
        self.assertFalse(result.valid)

    def test_gate_can_be_disabled_only_for_diagnostic_comparison(self):
        result = analyse_fast_line(course_frame(white_line=False, floor_white=True), config=FastLineConfig(green_gate_enabled=False)).result
        self.assertTrue(result.valid)
