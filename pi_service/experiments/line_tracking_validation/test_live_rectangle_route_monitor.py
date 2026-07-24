import unittest

import numpy as np

from live_rectangle_route_monitor import (
    has_connected_right_branch,
    keep_near_connected_points,
    planner_config_for_processing_rate,
    track_corridor,
)
from track_line.observations import LineDetectionResult, LineObservation


class TrackCorridorTests(unittest.TestCase):
    def test_corner_timing_scales_with_processing_rate(self):
        config = planner_config_for_processing_rate(30.0)
        self.assertEqual(config.missing_before_turn, 9)
        self.assertEqual(config.max_turn_frames, 300)

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

    def test_rejects_far_point_from_a_different_tape_component(self):
        mask = np.zeros((90, 100), dtype=np.uint8)
        mask[5:25, 10:25] = 255  # unrelated dark object in the far band
        mask[35:90, 45:58] = 255  # actual guide tape in middle and near bands
        points = ((17, 20), (51, 55), (51, 85))
        observation = LineObservation(
            frame_index=0,
            timestamp_ns=1,
            offset=0.0,
            heading=-0.7,
            curvature=0.0,
            confidence=0.8,
            line_lost=False,
            valid_bands=3,
            points_normalized=((0.17, 0.20), (0.51, 0.55), (0.51, 0.85)),
        )
        result = LineDetectionResult(
            observation=observation,
            mask=mask,
            roi_top=0,
            points_px=points,
            band_boundaries_px=(0, 30, 60, 90),
        )

        filtered = keep_near_connected_points(result, (90, 100, 3))

        self.assertEqual(filtered.points_px, points[1:])
        self.assertEqual(filtered.observation.valid_bands, 2)
        self.assertEqual(filtered.observation.rejection_reason, "far_candidate_disconnected")
        self.assertAlmostEqual(filtered.observation.heading, 0.0)

    def test_detects_connected_horizontal_right_arm(self):
        mask = np.zeros((100, 120), dtype=np.uint8)
        mask[35:95, 50:62] = 255
        mask[35:48, 50:112] = 255
        observation = LineObservation(
            frame_index=0,
            timestamp_ns=1,
            offset=0.0,
            heading=0.0,
            curvature=0.0,
            confidence=0.8,
            line_lost=False,
            valid_bands=2,
            points_normalized=((0.47, 0.42), (0.47, 0.78)),
        )
        result = LineDetectionResult(
            observation=observation,
            mask=mask,
            roi_top=0,
            points_px=((56, 42), (56, 78)),
            band_boundaries_px=(0, 33, 66, 100),
        )

        self.assertTrue(has_connected_right_branch(result))


if __name__ == "__main__":
    unittest.main(verbosity=2)
