from dataclasses import dataclass
import unittest

from fixed_rectangle_planner import FixedClockwiseRectanglePlanner, FixedRectangleConfig, RectangleIntent, RectangleState


@dataclass(frozen=True)
class Observation:
    line_lost: bool = False
    confidence: float = 0.8
    corner_direction: str | None = None
    corner_distance_px: float | None = None


class FixedRectanglePlannerTests(unittest.TestCase):
    def test_start_without_line_stops(self):
        planner = FixedClockwiseRectanglePlanner()
        self.assertEqual(planner.step(Observation(True, 0.0), 0.0).intent, RectangleIntent.STOP)

    def test_fixed_right_turn_then_reacquires_line(self):
        planner = FixedClockwiseRectanglePlanner(FixedRectangleConfig(line_lost_corner_frames=2, corner_forward_seconds=.2, right_turn_seconds=.3, reacquire_frames=2, reacquire_timeout_seconds=1, corners_to_complete=4))
        self.assertEqual(planner.step(Observation(), 0.0).intent, RectangleIntent.FOLLOW_LINE)
        self.assertEqual(planner.step(Observation(True, 0), .1).intent, RectangleIntent.FOLLOW_LINE)
        self.assertEqual(planner.step(Observation(True, 0), .2).intent, RectangleIntent.FORWARD_APPROACH)
        self.assertEqual(planner.step(Observation(True, 0), .41).intent, RectangleIntent.PIVOT_RIGHT)
        self.assertEqual(planner.step(Observation(True, 0), .72).intent, RectangleIntent.FORWARD_REACQUIRE)
        self.assertEqual(planner.step(Observation(), .73).state, RectangleState.REACQUIRE)
        self.assertEqual(planner.step(Observation(), .74).state, RectangleState.FOLLOW)

    def test_fourth_turn_stops(self):
        planner = FixedClockwiseRectanglePlanner(FixedRectangleConfig(line_lost_corner_frames=1, corner_forward_seconds=0, right_turn_seconds=0, corners_to_complete=1))
        planner.step(Observation(), 0)
        planner.step(Observation(True, 0), .1)
        planner.step(Observation(True, 0), .2)
        self.assertEqual(planner.step(Observation(True, 0), .3).intent, RectangleIntent.STOP)

    def test_prearmed_right_corner_pivots_on_first_lost_frame(self):
        planner = FixedClockwiseRectanglePlanner(FixedRectangleConfig(line_lost_corner_frames=5, right_turn_seconds=.3, prearm_corner_distance_px=170))
        self.assertEqual(planner.step(Observation(), 0).intent, RectangleIntent.FOLLOW_LINE)
        armed = planner.step(Observation(False, .8, "RIGHT", 120), .1)
        self.assertTrue(armed.corner_armed)
        turn = planner.step(Observation(True, 0), .2)
        self.assertEqual(turn.intent, RectangleIntent.PIVOT_RIGHT)
        self.assertEqual(turn.reason, "prearmed_right_corner_line_end_reached")


if __name__ == "__main__":
    unittest.main(verbosity=2)
