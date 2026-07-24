from dataclasses import dataclass
import unittest

from continuous_motor_control import ContinuousMotorConfig, drive_pwm_with_last_path, path_drive_pwm
from continuous_path_planner import ContinuousPathPlanner, PathIntent


@dataclass(frozen=True)
class Observation:
    line_lost: bool = False
    confidence: float = 0.8
    lookahead_offset: float | None = 0.0
    heading: float | None = 0.0


class ContinuousPathTests(unittest.TestCase):
    def test_right_lookahead_arcs_right(self):
        right, left = path_drive_pwm(Observation(lookahead_offset=.25), ContinuousMotorConfig("http://example"))
        self.assertLess(right, left)

    def test_short_line_loss_keeps_last_turning_pair(self):
        pair, label = drive_pwm_with_last_path(
            Observation(True, 0, None),
            ContinuousMotorConfig("http://example"),
            (35, 115),
        )
        self.assertEqual(pair, (35, 115))
        self.assertEqual(label, "HOLD_LAST_PATH")

    def test_blind_zone_predicts_then_stops_and_resumes(self):
        planner = ContinuousPathPlanner()
        self.assertEqual(planner.step(Observation(), 0).intent, PathIntent.FOLLOW_PATH)
        self.assertEqual(planner.step(Observation(True, 0, None), .10).intent, PathIntent.FOLLOW_PATH)
        self.assertEqual(planner.step(Observation(True, 0, None), .20).intent, PathIntent.FOLLOW_PATH)
        self.assertEqual(planner.step(Observation(True, 0, None), .30).intent, PathIntent.FOLLOW_PATH)
        predicted = planner.step(Observation(True, 0, None), .90)
        self.assertEqual(predicted.reason, "blind_zone_coast_to_safety_stop")
        self.assertEqual(planner.step(Observation(True, 0, None), 1.20).intent, PathIntent.STOP)
        resumed = planner.step(Observation(), 1.30)
        self.assertEqual(resumed.intent, PathIntent.FOLLOW_PATH)
        self.assertEqual(resumed.reason, "line_reacquired_resume_following")


if __name__ == "__main__":
    unittest.main(verbosity=2)
