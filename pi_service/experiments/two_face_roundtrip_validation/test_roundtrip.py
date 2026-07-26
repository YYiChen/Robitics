from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROBOT_WEB = Path(__file__).resolve().parents[2] / "robot_web"
if str(ROBOT_WEB) not in sys.path:
    sys.path.insert(0, str(ROBOT_WEB))

from routes.end_line.roundtrip import LandmarkTarget, RoundtripPhase, TwoFaceRoundtripPlanner


class TwoFaceRoundtripTests(unittest.TestCase):
    def _run_outbound(self, side: str) -> TwoFaceRoundtripPlanner:
        planner = TwoFaceRoundtripPlanner()
        planner.start(side)
        self.assertEqual(planner.phase, RoundtripPhase.FOLLOW_OUTBOUND)
        planner.endpoint_reached()
        expected_sides = [side, side, side, "RIGHT" if side == "LEFT" else "LEFT"]
        expected_targets = [
            LandmarkTarget.FACE,
            LandmarkTarget.WHITE_LINE,
            LandmarkTarget.FACE,
            LandmarkTarget.WHITE_LINE,
        ]
        for expected_side, expected_target in zip(expected_sides, expected_targets):
            instruction = planner.expected_turn()
            self.assertEqual(instruction.side, expected_side)
            self.assertEqual(instruction.target, expected_target)
            planner.target_reached(expected_target)
        self.assertEqual(planner.phase, RoundtripPhase.READY_RETURN)
        return planner

    def test_left_sweep_sequence_and_return(self):
        planner = self._run_outbound("LEFT")
        planner.start_return()
        planner.endpoint_reached()
        self.assertEqual(planner.phase, RoundtripPhase.COMPLETE)

    def test_right_sweep_is_mirrored(self):
        self._run_outbound("RIGHT")

    def test_wrong_landmark_cannot_advance_sequence(self):
        planner = TwoFaceRoundtripPlanner()
        planner.start("LEFT")
        planner.endpoint_reached()
        with self.assertRaises(ValueError):
            planner.target_reached(LandmarkTarget.WHITE_LINE)
        self.assertEqual(planner.phase, RoundtripPhase.TURN_FACE_1)

    def test_return_requires_ready_state(self):
        planner = TwoFaceRoundtripPlanner()
        planner.start("LEFT")
        with self.assertRaises(ValueError):
            planner.start_return()


if __name__ == "__main__":
    unittest.main()
