import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

from scanline_i_route import (  # noqa: E402
    RUN_LOG_SCHEMA_VERSION,
    ScanlineIRouteConfig,
    ScanlineIShapeRouteTracker,
    load_scanline_tuning_config,
)
from scanline_i_logic import IShapeTurnaroundPlanner, TurnaroundDecision, TurnaroundState  # noqa: E402


class _Gate:
    def enabled(self):
        return False


class ScanlineIRouteTests(unittest.TestCase):
    def test_web_tuning_persists_i_shape_speeds_only(self):
        with tempfile.TemporaryDirectory() as directory:
            tuning_path = Path(directory) / "scanline_web_tuning.json"
            tracker = ScanlineIShapeRouteTracker(None, None, None, _Gate(), ScanlineIRouteConfig(tuning_path=tuning_path))
            status = tracker.update_tuning({"straight_pwm": 135, "pivot_pwm": 210, "correction_gain": 140, "pivot_min_seconds": 3.5, "pivot_max_seconds": 6.0, "early_junction_trigger_y_ratio": 0.78, "early_line_lost_confirm_frames": 1})
            self.assertEqual(status["tuning"]["straight_pwm"], 135)
            self.assertEqual(status["tuning"]["pivot_pwm"], 210)
            self.assertEqual(json.loads(tuning_path.read_text(encoding="utf-8"))["correction_gain"], 140)
            reloaded = load_scanline_tuning_config(tuning_path)
            self.assertEqual(reloaded.straight_pwm, 135)
            self.assertEqual(reloaded.pivot_pwm, 210)
            self.assertEqual(reloaded.pivot_min_seconds, 3.5)
            self.assertEqual(reloaded.early_junction_trigger_y_ratio, 0.78)
            self.assertEqual(reloaded.early_line_lost_confirm_frames, 1)

    def test_rejects_pivot_minimum_longer_than_safety_timeout(self):
        tracker = ScanlineIShapeRouteTracker(None, None, None, _Gate())
        with self.assertRaisesRegex(ValueError, "最短掉头时间"):
            tracker.update_tuning({"pivot_min_seconds": 6.0, "pivot_max_seconds": 5.0})

    def test_web_tuning_updates_running_planner_without_resetting_route_state(self):
        tracker = ScanlineIShapeRouteTracker(None, None, None, _Gate())
        planner = IShapeTurnaroundPlanner(tracker._planner_config(tracker.config))
        planner.state = TurnaroundState.BAR_MARKED
        tracker._planner = planner

        tracker.update_tuning(
            {
                "pivot_min_seconds": 1.2,
                "pivot_max_seconds": 3.4,
                "bar_mark_timeout_seconds": 2.1,
                "early_junction_trigger_y_ratio": 0.68,
                "early_line_lost_confirm_frames": 2,
                "red_exit_arm_y_ratio": 0.79,
            }
        )

        self.assertIs(tracker._planner, planner)
        self.assertIs(planner.state, TurnaroundState.BAR_MARKED)
        self.assertEqual(planner.config.pivot_min_seconds, 1.2)
        self.assertEqual(planner.config.pivot_max_seconds, 3.4)
        self.assertEqual(planner.config.bar_mark_timeout_seconds, 2.1)
        self.assertEqual(planner.config.early_junction_trigger_y_ratio, 0.68)
        self.assertEqual(planner.config.early_line_lost_confirm_frames, 2)
        self.assertEqual(planner.config.red_exit_arm_y_ratio, 0.79)

    def test_straight_following_uses_differential_pwm_for_off_center_line(self):
        evidence = SimpleNamespace(line_center_x=400.0, line_centers=((1, 400.0, 20),) * 3)
        right, left = ScanlineIShapeRouteTracker._straight_pair(evidence, 640, ScanlineIRouteConfig(straight_pwm=120))
        self.assertLess(right, left)
        self.assertEqual(right + left, 240)

    def test_early_bar_prediction_keeps_line_following_active(self):
        self.assertTrue(ScanlineIShapeRouteTracker._keeps_forward_motion(TurnaroundState.EARLY_BAR_PREDICTED))
        self.assertFalse(ScanlineIShapeRouteTracker._keeps_forward_motion(TurnaroundState.BRAKE_BEFORE_PIVOT))

    def test_runtime_log_has_versioned_session_frame_and_transition_events(self):
        with tempfile.TemporaryDirectory() as directory:
            tuning_path = Path(directory) / "scanline_web_tuning.json"
            tracker = ScanlineIShapeRouteTracker(None, None, None, _Gate(), ScanlineIRouteConfig(tuning_path=tuning_path))
            tracker._open_run_log(tracker.config, (480, 640, 3))
            planner = IShapeTurnaroundPlanner(tracker._planner_config(tracker.config))
            evidence = SimpleNamespace(
                confidence=.88,
                valid_line=True,
                line_lost=False,
                line_center_x=330.0,
                line_centers=((442, 330.0, 24), (413, 328.0, 23), (384, 326.0, 25)),
                endpoint_detected=False,
                endpoint_y=None,
                endpoint_width=None,
                junction_detected=False,
                junction_y=None,
                junction_arm_count=0,
                red_marker_detected=False,
                red_marker_y=None,
                red_marker_span=None,
                lookahead_x=329.0,
                lookahead_y=300,
                path_length_px=180,
            )
            pair, control = tracker._straight_control(evidence, 640, tracker.config)
            following = TurnaroundDecision(TurnaroundState.FOLLOW_STRAIGHT, "following_near_anchored_longitudinal_line", 0, 0, 0, None)
            tracker._write_run_log(evidence, following, planner, 10, 100.0, "P_STRAIGHT", pair, control, (480, 640, 3))
            marked = TurnaroundDecision(TurnaroundState.BAR_MARKED, "lower_transverse_bar_marked_follow_until_stem_lost", 2, 0, 0, None)
            tracker._write_run_log(evidence, marked, planner, 11, 100.05, "P_STRAIGHT", pair, control, (480, 640, 3))
            tracker._last_frame_index = 11
            tracker._close_run_log(outcome="stopped", reason_code="TEST_COMPLETE")

            log_path = next((tuning_path.parent / "runtime_logs").glob("*.jsonl"))
            records = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual([record["event"] for record in records], [
                "session_start",
                "frame_observation",
                "frame_observation",
                "state_transition",
                "session_end",
            ])
            frame = records[1]
            self.assertEqual(frame["schema_version"], RUN_LOG_SCHEMA_VERSION)
            self.assertEqual(frame["vision"]["valid_bands"], 3)
            self.assertEqual(frame["vision"]["scanlines"][0]["width_px"], 24)
            self.assertEqual(frame["planner"]["counters"]["line_lost_frames"], 0)
            self.assertEqual(frame["control"]["mode"], "FOLLOW_DEADBAND")
            transition = records[3]
            self.assertEqual(transition["from_state"], "FOLLOW_STRAIGHT")
            self.assertEqual(transition["to_state"], "BAR_MARKED")
            self.assertEqual(transition["reason_code"], "WHITE_BAR_CONFIRMED")

    def test_tuning_change_is_audited_with_revision_and_old_new_values(self):
        with tempfile.TemporaryDirectory() as directory:
            tuning_path = Path(directory) / "scanline_web_tuning.json"
            tracker = ScanlineIShapeRouteTracker(None, None, None, _Gate(), ScanlineIRouteConfig(tuning_path=tuning_path))
            tracker._open_run_log(tracker.config, (480, 640, 3))
            tracker.update_tuning({"straight_pwm": 95})
            tracker._close_run_log(outcome="stopped", reason_code="TEST_COMPLETE")

            log_path = next((tuning_path.parent / "runtime_logs").glob("*.jsonl"))
            records = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
            tuning = next(record for record in records if record["event"] == "tuning_changed")
            self.assertEqual(tuning["config_revision"], 1)
            self.assertEqual(tuning["changes"]["straight_pwm"], {"old": 120, "new": 95})
