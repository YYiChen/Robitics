from dataclasses import dataclass
import unittest

from rectangle_route_planner import (
    ClockwiseRectanglePlanner,
    RouteIntent,
    RouteState,
)


@dataclass(frozen=True)
class Observation:
    offset: float | None
    heading: float | None
    confidence: float = 0.9
    line_lost: bool = False


LOST = Observation(offset=None, heading=None, confidence=0.0, line_lost=True)


class ClockwiseRectanglePlannerTests(unittest.TestCase):
    def test_straight_line_stays_straight(self):
        planner = ClockwiseRectanglePlanner()
        decisions = [planner.step(Observation(0.02, 0.01)) for _ in range(4)]
        self.assertTrue(all(item.intent is RouteIntent.STRAIGHT for item in decisions))
        self.assertEqual(planner.state, RouteState.FOLLOW)

    def test_confirmed_right_corner_turns_after_line_end(self):
        planner = ClockwiseRectanglePlanner()
        observations = (
            Observation(0.02, 0.42),
            Observation(0.05, 0.45),
            LOST,
            LOST,
            LOST,
        )
        decisions = [planner.step(item) for item in observations]
        self.assertEqual(
            [item.intent for item in decisions],
            [
                RouteIntent.STRAIGHT,
                RouteIntent.STRAIGHT,
                RouteIntent.STRAIGHT,
                RouteIntent.TURN_RIGHT,
                RouteIntent.TURN_RIGHT,
            ],
        )
        self.assertEqual(planner.state, RouteState.TURNING_RIGHT)

    def test_unexpected_loss_stops_instead_of_turning(self):
        planner = ClockwiseRectanglePlanner()
        decision = planner.step(LOST)
        self.assertEqual(decision.intent, RouteIntent.STOP)
        self.assertEqual(decision.reason, "unexpected_line_loss")

    def test_new_edge_needs_three_frames_before_straight(self):
        planner = ClockwiseRectanglePlanner()
        for item in (Observation(0.03, 0.43), Observation(0.05, 0.45), LOST, LOST):
            planner.step(item)
        decisions = [planner.step(Observation(-0.02, 0.01)) for _ in range(3)]
        self.assertEqual(
            [item.intent for item in decisions],
            [RouteIntent.TURN_RIGHT, RouteIntent.TURN_RIGHT, RouteIntent.STRAIGHT],
        )
        self.assertEqual(decisions[-1].reason, "new_edge_confirmed")


if __name__ == "__main__":
    unittest.main(verbosity=2)
