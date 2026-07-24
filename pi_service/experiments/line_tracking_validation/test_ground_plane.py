import unittest

import numpy as np

from ground_plane import GroundPlaneLineFilter


class GroundPlaneLineFilterTests(unittest.TestCase):
    def test_projects_corridor_centre_to_vehicle_centre(self):
        polygon = np.asarray([(0, 0), (100, 0), (100, 100), (0, 100)], dtype=np.int32)
        projected = GroundPlaneLineFilter.project(((50, 20), (50, 80)), polygon)
        self.assertTrue(np.allclose(projected[:, 0], (0.0, 0.0), atol=1e-5))

    def test_limits_single_frame_offset_jump(self):
        polygon = np.asarray([(0, 0), (100, 0), (100, 100), (0, 100)], dtype=np.int32)
        filter_ = GroundPlaneLineFilter()
        first, _ = filter_.update(
            line_lost=False,
            points_px=((50, 20), (50, 80)),
            corridor_polygon=polygon,
        )
        second, _ = filter_.update(
            line_lost=False,
            points_px=((100, 20), (100, 80)),
            corridor_polygon=polygon,
        )
        self.assertAlmostEqual(first, 0.0)
        self.assertAlmostEqual(second, 0.028, places=3)
