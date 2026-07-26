from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
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
    def test_line_turn_stop_arms_only_after_departure(self):
        with tempfile.TemporaryDirectory() as directory:
            tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(True), tuning_path=Path(directory) / "tuning.json")
            centred = SimpleNamespace(valid=True, center_x=320.0)
            lost = SimpleNamespace(valid=False, center_x=None)
            self.assertFalse(tracker._observe_line_turn_reacquisition(centred, 640))
            self.assertFalse(tracker._observe_line_turn_reacquisition(lost, 640))
            self.assertFalse(tracker._observe_line_turn_reacquisition(centred, 640))
            self.assertFalse(tracker._observe_line_turn_reacquisition(centred, 640))
            self.assertTrue(tracker._observe_line_turn_reacquisition(centred, 640))

    def test_line_turn_is_separate_from_face_turn_and_needs_no_heartbeat(self):
        with tempfile.TemporaryDirectory() as directory:
            tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(True), tuning_path=Path(directory) / "tuning.json")
            status = tracker.request_line_center_turn("START_RIGHT")
            self.assertEqual(status["state"], "LINE_CENTER_TURN")
            self.assertEqual(status["vision_turn_target"], "WHITE_LINE")
            self.assertEqual(tracker._motion_phase, "LINE_CENTER_TURN")
            self.assertEqual(tracker._face_turn_side, "RIGHT")
            self.assertEqual(tracker._face_turn_deadline, 0.0)
            status = tracker.request_line_center_turn("STOP")
            self.assertEqual(status["state"], "LINE_TURN_STOPPED")
            self.assertIsNone(status["vision_turn_target"])

    def test_roundtrip_face_stop_advances_to_white_line_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(True), tuning_path=Path(directory) / "tuning.json")
            tracker.request_roundtrip_start("LEFT")
            tracker._roundtrip.endpoint_reached()
            tracker._schedule_roundtrip_turn(now=0.0)
            tracker.request_face_center_turn("START_LEFT")
            status = tracker.request_face_center_turn("STOP")
            self.assertEqual(status["roundtrip"]["phase"], "TURN_LINE_MIDDLE")
            self.assertEqual(status["roundtrip"]["expected_target"], "WHITE_LINE")
            self.assertEqual(tracker._motion_phase, "ROUNDTRIP_HOLD")

    def test_stale_face_stop_cannot_cancel_line_turn(self):
        with tempfile.TemporaryDirectory() as directory:
            tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(True), tuning_path=Path(directory) / "tuning.json")
            tracker.request_line_center_turn("START_LEFT")
            status = tracker.request_face_center_turn("STOP")
            self.assertEqual(tracker._motion_phase, "LINE_CENTER_TURN")
            self.assertEqual(status["vision_turn_target"], "WHITE_LINE")

    def test_update_is_visible_immediate_and_persisted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.json"
            turn_90, turn_180 = Path(directory) / "turn_90.json", Path(directory) / "turn_180.json"
            save_turn_profile(turn_90, TurnProfile(200, .3))
            save_turn_profile(turn_180, TurnProfile(200, .3))
            with patch.object(adaptor_module, "TURN_90_PATH", turn_90), patch.object(adaptor_module, "TURN_180_PATH", turn_180):
                tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(), tuning_path=path)
                status = tracker.update_tuning({
                    "straight_pwm": 91,
                    "turn_90_step_seconds": 1.4,
                    "turn_interstep_pause_seconds": 1.5,
                    "red_alignment_min_angle": 78,
                    "red_alignment_confirm_frames": 3,
                    "turn_90_max_steps": 5,
                    "correction_gain": 190,
                    "face_turn_pwm": 231,
                    "face_turn_pulse_seconds": .35,
                    "face_turn_cooldown_seconds": 1.2,
                    "face_turn_max_seconds": 24,
                    "face_turn_line_center_confirm_frames": 4,
                    "face_turn_line_center_deadband_normalized": .16,
                })
                self.assertEqual(status["tuning"]["straight_pwm"], 91)
                self.assertEqual(status["tuning"]["correction_gain"], 190.0)
                self.assertEqual(status["tuning"]["turn_90_step_seconds"], 1.4)
                self.assertEqual(status["tuning"]["turn_interstep_pause_seconds"], 1.5)
                self.assertEqual(status["tuning"]["red_alignment_min_angle"], 78.0)
                self.assertEqual(status["tuning"]["red_alignment_confirm_frames"], 3)
                self.assertEqual(status["tuning"]["turn_90_max_steps"], 5)
                self.assertEqual(status["tuning"]["face_turn_pwm"], 231)
                self.assertEqual(status["tuning"]["face_turn_pulse_seconds"], .35)
                self.assertEqual(status["tuning"]["face_turn_cooldown_seconds"], 1.2)
                self.assertEqual(status["tuning"]["face_turn_max_seconds"], 24.0)
                self.assertEqual(status["tuning"]["face_turn_line_center_confirm_frames"], 4)
                self.assertEqual(status["tuning"]["face_turn_line_center_deadband_normalized"], .16)
                self.assertTrue(path.exists())
                reloaded = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(), tuning_path=path)
                self.assertEqual(reloaded.status_dict()["tuning"]["straight_pwm"], 91)
                self.assertEqual(reloaded.status_dict()["tuning"]["face_turn_pwm"], 231)
                self.assertEqual(reloaded.status_dict()["tuning"]["face_turn_line_center_confirm_frames"], 4)

    def test_line_center_gate_uses_saved_web_tuning_without_resetting_other_actions(self):
        with tempfile.TemporaryDirectory() as directory:
            tracker = EndLineTurnAdaptorRouteTracker(
                None, None, None, _Gate(True),
                tuning_path=Path(directory) / "tuning.json",
            )
            tracker.update_tuning({
                "face_turn_line_center_deadband_normalized": .05,
                "face_turn_line_center_confirm_frames": 1,
            })
            tracker.request_line_center_turn("START_LEFT")
            off_centre = SimpleNamespace(valid=True, center_x=400.0)
            near_centre = SimpleNamespace(valid=True, center_x=330.0)
            self.assertFalse(tracker._observe_line_turn_reacquisition(off_centre, 640))
            self.assertTrue(tracker._observe_line_turn_reacquisition(near_centre, 640))

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
            # The Pi compatibility shim may load the same dataclass through a
            # second module name, so compare the profile contract rather than
            # relying on exact Python class identity.
            self.assertEqual(tracker._manual_profile.pwm, 137)
            self.assertEqual(tracker._manual_profile.step_seconds, 1.7)
            self.assertEqual((tracker._manual_max_steps, tracker._manual_steps_started), (4, 1))

    def test_face_turn_requires_m_and_uses_pi_deadman_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(False), tuning_path=Path(directory) / "tuning.json")
            with self.assertRaises(ValueError):
                tracker.request_face_center_turn("START_LEFT")
            tracker.gate.value = True
            status = tracker.request_face_center_turn("START_LEFT")
            self.assertEqual(status["state"], "FACE_CENTER_TURN")
            self.assertEqual(status["vision_turn_target"], "FACE")
            self.assertEqual(tracker._face_turn_side, "LEFT")
            self.assertEqual(adaptor_module.FACE_TURN_PWM, 255)
            self.assertTrue(tracker._face_turn_pulse_active)
            self.assertAlmostEqual(
                tracker._face_turn_phase_until - tracker._face_turn_started,
                adaptor_module.FACE_TURN_PULSE_SECONDS,
                places=3,
            )
            self.assertEqual(status["tuning"]["face_turn_pulse_seconds"], .20)
            self.assertEqual(status["tuning"]["face_turn_cooldown_seconds"], 2.0)
            self.assertEqual(status["tuning"]["face_turn_heartbeat_seconds"], 3.0)
            self.assertEqual(status["tuning"]["face_turn_max_seconds"], 15.0)
            first_deadline = tracker._face_turn_deadline
            tracker.request_face_center_turn("HEARTBEAT")
            self.assertGreaterEqual(tracker._face_turn_deadline, first_deadline)
            status = tracker.request_face_center_turn("STOP")
            self.assertEqual(status["state"], "FACE_CENTERED_STOP")
            self.assertIsNone(tracker._face_turn_side)
