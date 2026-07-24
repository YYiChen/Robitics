from dataclasses import dataclass
import unittest

from line_stop_planner import LineIntent, LineStopConfig, StraightLineStopPlanner
from straight_motor_control import StraightMotorConfig, drive_pwm_for_offset


@dataclass(frozen=True)
class Observation:
    offset: float | None = 0.0
    line_lost: bool = False
    confidence: float = 0.8


class StraightLineStopTests(unittest.TestCase):
    def test_stops_after_five_consecutive_lost_frames_and_latches(self):
        planner = StraightLineStopPlanner(LineStopConfig(line_lost_stop_frames=5))
        self.assertEqual(planner.step(Observation()).intent, LineIntent.STRAIGHT)
        for count in range(1, 5):
            decision = planner.step(Observation(line_lost=True, confidence=0.0))
            self.assertEqual(decision.intent, LineIntent.STRAIGHT)
            self.assertEqual(decision.lost_frames, count)
        stopped = planner.step(Observation(line_lost=True, confidence=0.0))
        self.assertEqual(stopped.intent, LineIntent.STOP)
        self.assertEqual(planner.step(Observation()).intent, LineIntent.STOP)

    def test_visible_line_resets_brief_loss_counter(self):
        planner = StraightLineStopPlanner(LineStopConfig(line_lost_stop_frames=3))
        planner.step(Observation(line_lost=True, confidence=0.0))
        self.assertEqual(planner.step(Observation()).lost_frames, 0)

    def test_nonzero_p_correction_is_at_least_twenty_pwm(self):
        config = StraightMotorConfig("http://robot")
        self.assertEqual(drive_pwm_for_offset(Observation(offset=0.0), config), (65, 65))
        self.assertEqual(drive_pwm_for_offset(Observation(offset=0.06), config), (45, 85))
        self.assertEqual(drive_pwm_for_offset(Observation(offset=-0.06), config), (85, 45))


if __name__ == "__main__":
    unittest.main(verbosity=2)
