from dataclasses import dataclass
import unittest

from rectangle_route_planner import (
    ClockwiseRectanglePlanner,
    RectanglePlannerConfig,
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
        planner = ClockwiseRectanglePlanner(RectanglePlannerConfig(missing_before_turn=1))
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
                RouteIntent.TURN_RIGHT,
                RouteIntent.TURN_RIGHT,
                RouteIntent.TURN_RIGHT,
            ],
        )
        self.assertEqual(planner.state, RouteState.TURNING_RIGHT)

    def test_unexpected_loss_stops_instead_of_turning(self):
        planner = ClockwiseRectanglePlanner(
            RectanglePlannerConfig(fixed_right_turn_on_line_end=False)
        )
        decision = planner.step(LOST)
        self.assertEqual(decision.intent, RouteIntent.STOP)
        self.assertEqual(decision.reason, "unexpected_line_loss")

    def test_fixed_clockwise_route_turns_right_when_line_ends_without_branch_evidence(self):
        planner = ClockwiseRectanglePlanner(RectanglePlannerConfig(missing_before_turn=2))
        first = planner.step(LOST)
        decision = planner.step(LOST)
        self.assertEqual(first.intent, RouteIntent.STRAIGHT)
        self.assertEqual(first.state, RouteState.APPROACHING_RIGHT_CORNER)
        self.assertEqual(first.reason, "line_end_confirming")
        self.assertEqual(decision.intent, RouteIntent.TURN_RIGHT)
        self.assertEqual(decision.state, RouteState.TURNING_RIGHT)
        self.assertEqual(decision.reason, "fixed_route_line_end_confirmed")

    def test_line_reacquired_while_approaching_corner_cancels_turn(self):
        planner = ClockwiseRectanglePlanner(RectanglePlannerConfig(missing_before_turn=3))
        planner.step(LOST)
        decision = planner.step(Observation(0.01, 0.01))
        self.assertEqual(decision.intent, RouteIntent.STRAIGHT)
        self.assertEqual(decision.state, RouteState.FOLLOW)
        self.assertEqual(decision.reason, "line_reacquired_before_turn")

    def test_fixed_route_turn_still_stops_at_safety_timeout(self):
        planner = ClockwiseRectanglePlanner(
            RectanglePlannerConfig(max_turn_frames=2, missing_before_turn=1)
        )
        decisions = [planner.step(LOST) for _ in range(3)]
        self.assertEqual(
            [item.intent for item in decisions],
            [RouteIntent.TURN_RIGHT, RouteIntent.TURN_RIGHT, RouteIntent.STOP],
        )
        self.assertEqual(decisions[-1].reason, "right_turn_timeout")

    def test_new_edge_needs_three_frames_before_straight(self):
        planner = ClockwiseRectanglePlanner(RectanglePlannerConfig(missing_before_turn=1))
        for item in (Observation(0.03, 0.43), Observation(0.05, 0.45), LOST):
            planner.step(item)
        decisions = [planner.step(Observation(-0.02, 0.01), new_line_ready=True) for _ in range(3)]
        self.assertEqual(
            [item.intent for item in decisions],
            [RouteIntent.TURN_RIGHT, RouteIntent.TURN_RIGHT, RouteIntent.STRAIGHT],
        )
        self.assertEqual(decisions[-1].reason, "new_edge_confirmed")

    def test_explicit_right_branch_arms_even_when_far_heading_is_unavailable(self):
        planner = ClockwiseRectanglePlanner()
        observation = Observation(0.01, 0.02)
        planner.step(observation, right_corner_ahead=True)
        decision = planner.step(observation, right_corner_ahead=True)
        self.assertEqual(planner.state, RouteState.RIGHT_CORNER_ARMED)
        self.assertEqual(decision.reason, "right_corner_seen_ahead")

    def test_armed_corner_survives_three_missing_branch_frames(self):
        planner = ClockwiseRectanglePlanner()
        observation = Observation(0.01, 0.02)
        planner.step(observation, right_corner_ahead=True)
        planner.step(observation, right_corner_ahead=True)
        decisions = [planner.step(observation) for _ in range(3)]
        self.assertTrue(all(item.state is RouteState.RIGHT_CORNER_ARMED for item in decisions))
        self.assertTrue(all(item.reason == "right_corner_latched" for item in decisions))

    def test_turning_ignores_old_corner_line_until_new_edge_is_ready(self):
        planner = ClockwiseRectanglePlanner(RectanglePlannerConfig(missing_before_turn=1))
        observation = Observation(0.01, 0.02)
        planner.step(observation, right_corner_ahead=True)
        planner.step(observation, right_corner_ahead=True)
        planner.step(LOST)
        decisions = [planner.step(observation, new_line_ready=False) for _ in range(3)]
        self.assertTrue(all(item.intent is RouteIntent.TURN_RIGHT for item in decisions))
        self.assertTrue(all(item.reason == "turning_until_new_line" for item in decisions))


if __name__ == "__main__":
    unittest.main(verbosity=2)
