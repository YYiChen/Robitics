from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
ROBOT_WEB = ROOT / "robot_web"
if str(ROBOT_WEB) not in sys.path:
    sys.path.insert(0, str(ROBOT_WEB))

import end_line_turn_adaptor as adaptor_module
from end_line_turn_adaptor import EndLineTurnAdaptorRouteTracker
from turn_profiles import TurnProfile, save_turn_profile


class _Gate:
    def __init__(self, enabled=False): self.value = enabled
    def enabled(self): return self.value
    def toggle(self): self.value = not self.value; return self.value
    def set_enabled(self, enabled): self.value = bool(enabled); return self.value


class _Controller:
    def __init__(self): self.commands = []
    def set_direct_drive(self, right, left): self.commands.append((right, left))
    def stop_now(self): self.commands.append((0, 0))


class EndLineAdaptorTuningTests(unittest.TestCase):
    def test_update_is_visible_immediate_and_persisted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.json"
            turn_90, turn_180 = Path(directory) / "turn_90.json", Path(directory) / "turn_180.json"
            save_turn_profile(turn_90, TurnProfile(200, .3))
            save_turn_profile(turn_180, TurnProfile(200, .3))
            with patch.object(adaptor_module, "TURN_90_PATH", turn_90), patch.object(adaptor_module, "TURN_180_PATH", turn_180):
                tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(), tuning_path=path)
                status = tracker.update_tuning({"straight_pwm": 91, "turn_90_step_seconds": 1.4, "turn_interstep_pause_seconds": 1.5, "red_alignment_min_angle": 78, "red_alignment_confirm_frames": 3, "turn_90_max_steps": 5, "correction_gain": 190})
                self.assertEqual(status["tuning"]["straight_pwm"], 91)
                self.assertEqual(status["tuning"]["correction_gain"], 190.0)
                self.assertEqual(status["tuning"]["turn_90_step_seconds"], 1.4)
                self.assertEqual(status["tuning"]["turn_interstep_pause_seconds"], 1.5)
                self.assertEqual(status["tuning"]["red_alignment_min_angle"], 78.0)
                self.assertEqual(status["tuning"]["red_alignment_confirm_frames"], 3)
                self.assertEqual(status["tuning"]["turn_90_max_steps"], 5)
                self.assertTrue(path.exists())
                reloaded = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(), tuning_path=path)
                self.assertEqual(reloaded.status_dict()["tuning"]["straight_pwm"], 91)

    def test_rejects_invalid_correction_range(self):
        with tempfile.TemporaryDirectory() as directory:
            tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(), tuning_path=Path(directory) / "tuning.json")
            with self.assertRaises(ValueError):
                tracker.update_tuning({"minimum_correction_pwm": 200, "maximum_correction_pwm": 100})

    def test_manual_turn_is_m_gated_and_uses_90_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.json"
            turn_90, turn_180 = Path(directory) / "turn_90.json", Path(directory) / "turn_180.json"
            save_turn_profile(turn_90, TurnProfile(200, 2.5))
            save_turn_profile(turn_180, TurnProfile(200, 5.0))
            with patch.object(adaptor_module, "TURN_90_PATH", turn_90), patch.object(adaptor_module, "TURN_180_PATH", turn_180):
                tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(True), tuning_path=path)
                save_turn_profile(turn_90, TurnProfile(137, 1.7))
                status = tracker.request_manual_turn("LEFT_90")
            self.assertEqual(status["state"], "MANUAL_STEP")
            self.assertEqual(tracker._pending_turn_side, "LEFT")
            self.assertEqual(tracker._manual_degrees, 90)
            self.assertEqual(tracker._manual_profile, TurnProfile(137, 1.7))
            self.assertEqual((tracker._manual_max_steps, tracker._manual_steps_started), (4, 1))

    def test_m_remains_manual_only_and_n_starts_auto_left_mission(self):
        with tempfile.TemporaryDirectory() as directory:
            gate = _Gate()
            tracker = EndLineTurnAdaptorRouteTracker(None, None, None, gate, tuning_path=Path(directory) / "tuning.json")

            manual_status = tracker.toggle_drive()
            self.assertTrue(manual_status["enabled"])
            self.assertFalse(manual_status["auto_mission_active"])
            self.assertTrue(tracker._manual_only)
            self.assertEqual(manual_status["state"], "MANUAL_READY")

            auto_status = tracker.toggle_auto_left_mission()
            self.assertTrue(auto_status["enabled"])
            self.assertTrue(auto_status["auto_mission_active"])
            self.assertFalse(tracker._manual_only)
            self.assertEqual(tracker._pending_turn_side, "LEFT")
            self.assertEqual(auto_status["state"], "AUTO_FOLLOW")
            with self.assertRaises(ValueError):
                tracker.request_manual_turn("LEFT_90")

            stopped_status = tracker.toggle_drive()
            self.assertFalse(stopped_status["enabled"])
            self.assertFalse(stopped_status["auto_mission_active"])
            self.assertTrue(tracker._manual_only)

    def test_second_n_press_cancels_auto_mission(self):
        with tempfile.TemporaryDirectory() as directory:
            tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(), tuning_path=Path(directory) / "tuning.json")
            tracker.toggle_auto_left_mission()
            status = tracker.toggle_auto_left_mission()
            self.assertFalse(status["enabled"])
            self.assertFalse(status["auto_mission_active"])
            self.assertEqual(status["state"], "AUTO_CANCELLED")

    def test_j_left_face_turn_stops_after_four_centered_observations(self):
        with tempfile.TemporaryDirectory() as directory:
            gate = _Gate()
            tracker = EndLineTurnAdaptorRouteTracker(_Controller(), None, None, gate, tuning_path=Path(directory) / "tuning.json")
            status = tracker.request_face_turn("LEFT")
            self.assertTrue(status["face_turn_active"])
            self.assertEqual(status["face_search_side"], "LEFT")

            now = tracker._face_turn_started_at + 0.1
            for index in range(4):
                tracker.submit_face_observation({"found": True, "frame_width": 640, "center_x": 320})
                state, _motor, _command, _detail = tracker._step_face_turn(now + index * 0.01)
            self.assertEqual(state, "FACE_CENTERED")
            self.assertFalse(tracker.status_dict()["face_turn_active"])
            self.assertFalse(gate.enabled())

    def test_l_right_face_turn_searches_right_and_stale_stream_stops(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = _Controller()
            gate = _Gate()
            tracker = EndLineTurnAdaptorRouteTracker(controller, None, None, gate, tuning_path=Path(directory) / "tuning.json")
            tracker.request_face_turn("RIGHT")
            tracker.submit_face_observation({"found": False, "frame_width": 640})
            now = tracker._face_turn_started_at + 0.1
            state, _motor, _command, _detail = tracker._step_face_turn(now)
            self.assertEqual(state, "FACE_START_RIGHT")
            state, _motor, command, _detail = tracker._step_face_turn(now + 0.01)
            self.assertEqual(state, "FACE_PULSE_RIGHT")
            self.assertEqual(command, (-adaptor_module.FACE_TURN_PWM, adaptor_module.FACE_TURN_PWM))

            state, _motor, _command, _detail = tracker._step_face_turn(
                tracker._face_observation_at + adaptor_module.FACE_OBSERVATION_TIMEOUT_SECONDS + 0.01
            )
            self.assertEqual(state, "FACE_STREAM_LOST")
            self.assertFalse(gate.enabled())
