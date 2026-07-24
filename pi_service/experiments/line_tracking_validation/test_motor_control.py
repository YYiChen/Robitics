from dataclasses import dataclass
import unittest

from motor_control import MotorControlConfig, action_for_decision
from rectangle_route_planner import PlannerDecision, RouteIntent, RouteState


@dataclass(frozen=True)
class Observation:
    offset: float | None


class MotorActionMappingTests(unittest.TestCase):
    def test_straight_uses_deadband_and_curve_actions(self):
        decision = PlannerDecision(RouteIntent.STRAIGHT, RouteState.FOLLOW, "test")
        config = MotorControlConfig("http://robot")
        self.assertEqual(action_for_decision(decision, Observation(0.0), config), "F")
        self.assertEqual(action_for_decision(decision, Observation(-0.2), config), "FL")
        self.assertEqual(action_for_decision(decision, Observation(0.2), config), "FR")

    def test_right_turn_and_stop_override_offset(self):
        config = MotorControlConfig("http://robot")
        turn = PlannerDecision(RouteIntent.TURN_RIGHT, RouteState.TURNING_RIGHT, "test")
        stop = PlannerDecision(RouteIntent.STOP, RouteState.LOST, "test")
        self.assertEqual(action_for_decision(turn, Observation(-0.4), config), "PR")
        self.assertEqual(action_for_decision(stop, Observation(0.4), config), "STOP")
        self.assertEqual(action_for_decision(turn, Observation(None), config), "PR")

    def test_approaching_corner_keeps_moving_forward_during_short_line_loss(self):
        decision = PlannerDecision(
            RouteIntent.STRAIGHT,
            RouteState.APPROACHING_RIGHT_CORNER,
            "line_end_confirming",
        )
        config = MotorControlConfig("http://robot")
        self.assertEqual(action_for_decision(decision, Observation(None), config), "F")


if __name__ == "__main__":
    unittest.main(verbosity=2)
