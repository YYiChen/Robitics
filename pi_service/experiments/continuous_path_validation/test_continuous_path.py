from dataclasses import dataclass
import unittest

from continuous_motor_control import ContinuousMotorConfig, path_drive_pwm
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

    def test_stops_after_three_lost_frames(self):
        planner = ContinuousPathPlanner()
        self.assertEqual(planner.step(Observation()).intent, PathIntent.FOLLOW_PATH)
        self.assertEqual(planner.step(Observation(True, 0, None)).intent, PathIntent.FOLLOW_PATH)
        self.assertEqual(planner.step(Observation(True, 0, None)).intent, PathIntent.FOLLOW_PATH)
        self.assertEqual(planner.step(Observation(True, 0, None)).intent, PathIntent.STOP)


if __name__ == "__main__":
    unittest.main(verbosity=2)
