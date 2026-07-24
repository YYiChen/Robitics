import unittest

import numpy as np

from live_rectangle_route_monitor import track_corridor


class TrackCorridorTests(unittest.TestCase):
    def test_keeps_dark_line_inside_and_masks_dark_object_outside(self):
        frame = np.full((100, 200, 3), 255, dtype=np.uint8)
        frame[70:100, 95:105] = 0  # guide tape inside the lower corridor
        frame[45:75, 185:195] = 0  # chair leg outside the corridor

        filtered, _polygon = track_corridor(
            frame,
            roi_top_ratio=0.42,
            top_width=0.70,
            bottom_width=0.56,
        )

        self.assertTrue(np.array_equal(filtered[80, 100], np.array([0, 0, 0])))
        self.assertTrue(np.array_equal(filtered[60, 190], np.array([255, 255, 255])))

    def test_rejects_invalid_width(self):
        frame = np.full((100, 200, 3), 255, dtype=np.uint8)
        with self.assertRaises(ValueError):
            track_corridor(frame, roi_top_ratio=0.42, top_width=0, bottom_width=0.56)


if __name__ == "__main__":
    unittest.main(verbosity=2)
