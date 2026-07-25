from dataclasses import dataclass
import unittest

from continuous_motor_control import ContinuousMotorConfig, drive_pwm_with_last_path, path_drive_pwm
from continuous_path_planner import ContinuousPathPlanner, PathIntent
from marker_counter import MarkerCounter, MarkerCounterConfig, MarkerState


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

    def test_turn_never_drops_inner_wheel_below_running_threshold(self):
        right, left = path_drive_pwm(
            Observation(lookahead_offset=.9),
            ContinuousMotorConfig("http://example", straight_pwm=90, maximum_correction_pwm=70, minimum_wheel_pwm=55),
        )
        self.assertGreaterEqual(right, 55)
        self.assertGreaterEqual(left, 55)

    def test_visible_sharp_turn_gets_stronger_than_micro_adjustment(self):
        right, left = path_drive_pwm(
            Observation(lookahead_offset=.09, heading=.0),
            ContinuousMotorConfig("http://example", straight_pwm=95, sharp_turn_error=.08, sharp_turn_correction_pwm=55),
        )
        self.assertEqual((right, left), (55, 150))

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

    def test_marker_is_counted_once_then_rearmed_after_it_leaves(self):
        counter = MarkerCounter(MarkerCounterConfig(confirm_frames=2, clear_frames=3, markers_per_lap=4))
        self.assertFalse(counter.update(True).event)
        passed = counter.update(True)
        self.assertTrue(passed.event)
        self.assertEqual((passed.marker_in_lap, passed.lap_count), (1, 0))
        self.assertFalse(counter.update(True).event)
        self.assertEqual(counter.update(False).state, MarkerState.COOLDOWN)
        self.assertEqual(counter.update(False).state, MarkerState.COOLDOWN)
        self.assertEqual(counter.update(False).state, MarkerState.ARMED)
        counter.update(True)
        second = counter.update(True)
        self.assertTrue(second.event)
        self.assertEqual(second.marker_in_lap, 2)

    def test_new_marker_high_in_image_rearms_when_old_candidate_leaks(self):
        counter = MarkerCounter(MarkerCounterConfig(
            confirm_frames=2,
            clear_frames=12,
            markers_per_lap=4,
            rearm_y_drop_ratio=.18,
        ))
        counter.update(True, .74)
        first = counter.update(True, .76)
        self.assertTrue(first.event)
        # The old candidate incorrectly stays detected. A new marker enters
        # near the top of the image, therefore it must not remain locked out.
        rearmed = counter.update(True, .52)
        self.assertEqual(rearmed.state, MarkerState.ARMED)
        counter.update(True, .54)
        second = counter.update(True, .56)
        self.assertTrue(second.event)
        self.assertEqual(second.marker_in_lap, 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
