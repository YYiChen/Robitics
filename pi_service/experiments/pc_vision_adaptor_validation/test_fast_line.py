from __future__ import annotations
import unittest
import numpy as np
from fast_line import find_fast_line, pwm_for_line


class FastLineTests(unittest.TestCase):
    def test_follows_narrow_white_tape(self):
        frame = np.zeros((120, 160, 3), dtype=np.uint8); frame[:] = (45, 110, 30)
        frame[:, 105:115] = (245, 245, 245)
        result = find_fast_line(frame)
        self.assertTrue(result.valid); self.assertGreater(result.center_x, 100)
        right, left = pwm_for_line(result, 160, 85)
        self.assertLess(right, left)

    def test_rejects_wide_horizontal_floor_patch(self):
        frame = np.zeros((120, 160, 3), dtype=np.uint8); frame[:] = (45, 110, 30)
        frame[96:, :] = (245, 245, 245)
        self.assertFalse(find_fast_line(frame).valid)
