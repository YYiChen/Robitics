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

    def test_face_turn_requires_m_and_uses_pi_deadman_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(False), tuning_path=Path(directory) / "tuning.json")
            with self.assertRaises(ValueError):
                tracker.request_face_center_turn("START_LEFT")
            tracker.gate.value = True
            status = tracker.request_face_center_turn("START_LEFT")
            self.assertEqual(status["state"], "FACE_CENTER_TURN")
            self.assertEqual(tracker._face_turn_side, "LEFT")
            self.assertEqual(adaptor_module.FACE_TURN_PWM, 255)
            self.assertTrue(tracker._face_turn_pulse_active)
            self.assertAlmostEqual(
                tracker._face_turn_phase_until - tracker._face_turn_started,
                adaptor_module.FACE_TURN_PULSE_SECONDS,
                places=3,
            )
            self.assertEqual(status["tuning"]["face_turn_cooldown_seconds"], 2.0)
            self.assertEqual(status["tuning"]["face_turn_heartbeat_seconds"], 3.0)
            self.assertEqual(status["tuning"]["face_turn_max_seconds"], 15.0)
            first_deadline = tracker._face_turn_deadline
            tracker.request_face_center_turn("HEARTBEAT")
            self.assertGreaterEqual(tracker._face_turn_deadline, first_deadline)
            status = tracker.request_face_center_turn("STOP")
            self.assertEqual(status["state"], "FACE_CENTERED_STOP")
            self.assertIsNone(tracker._face_turn_side)
