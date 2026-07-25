from __future__ import annotations

import unittest

import numpy as np

from node_planner import NodePlanner, NodePlannerConfig, NodeState
from track_line.config import LineDetectorConfig
from track_line.detector import OpenCVLineDetector


class NodePlannerTests(unittest.TestCase):
    def test_confirm_approach_hold_and_automatic_resume(self):
        planner = NodePlanner(NodePlannerConfig(confirm_frames=2, clear_frames=2, stop_y_ratio=.72, hold_seconds=2.0))
        self.assertEqual(planner.step(marker_detected=True, marker_y_ratio=.45, now=0).state, NodeState.FOLLOW)
        self.assertEqual(planner.step(marker_detected=True, marker_y_ratio=.52, now=.1).state, NodeState.APPROACH_NODE)
        stopped = planner.step(marker_detected=True, marker_y_ratio=.75, now=.2)
        self.assertEqual((stopped.state, stopped.completed_node, stopped.lap_count), (NodeState.HOLD_NODE, 1, 0))
        self.assertTrue(stopped.should_stop)
        self.assertEqual(planner.step(marker_detected=False, marker_y_ratio=None, now=1.9).state, NodeState.HOLD_NODE)
        self.assertEqual(planner.step(marker_detected=False, marker_y_ratio=None, now=2.3).state, NodeState.FOLLOW)

    def test_manual_resume_mode_waits_for_signal(self):
        planner = NodePlanner(NodePlannerConfig(confirm_frames=1, stop_y_ratio=.5, auto_resume=False))
        self.assertEqual(planner.step(marker_detected=True, marker_y_ratio=.2, now=0).state, NodeState.APPROACH_NODE)
        self.assertEqual(planner.step(marker_detected=True, marker_y_ratio=.6, now=.1).state, NodeState.HOLD_NODE)
        self.assertEqual(planner.step(marker_detected=True, marker_y_ratio=.6, now=.2).state, NodeState.WAIT_RESUME)
        planner.resume()
        self.assertEqual(planner.step(marker_detected=False, marker_y_ratio=None, now=.3).state, NodeState.FOLLOW)


class TransverseGeometryTests(unittest.TestCase):
    def test_crossbar_is_wider_than_a_normal_route(self):
        detector = OpenCVLineDetector(LineDetectorConfig(marker_minimum_arm_pixels=15))
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[:, 46:55] = 255
        path = tuple((50, y) for y in range(95, 4, -1))
        self.assertFalse(detector._transverse_marker_evidence(mask, path, 0)[0])
        mask[46:55, 10:91] = 255
        detected, point, span = detector._transverse_marker_evidence(mask, path, 0)
        self.assertTrue(detected)
        self.assertIsNotNone(point)
        self.assertIsNotNone(span)
        self.assertLess(span[1][0], point[0])
        self.assertGreater(span[0][0], point[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
