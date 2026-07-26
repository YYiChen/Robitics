from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

from control.robotics_gateway import RoboticsGateway
from routes.common import AutonomousRunGate


class FakeTracker:
    def __init__(self) -> None:
        self.gate = AutonomousRunGate()
        self.calls: list[tuple[str, object]] = []
        self.tuning = {
            "process_fps": 20.0,
            "straight_pwm": 85,
            "correction_deadband": 0.035,
            "correction_gain": 155.0,
            "minimum_correction_pwm": 18,
            "maximum_correction_pwm": 180,
            "line_lost_confirm_frames": 3,
            "brake_hold_seconds": 0.18,
            "face_turn_pwm": 255,
            "face_turn_pulse_seconds": 0.2,
            "face_turn_cooldown_seconds": 2.0,
            "face_turn_max_seconds": 15.0,
            "face_turn_line_center_deadband_normalized": 0.1,
            "face_turn_line_center_confirm_frames": 3,
        }

    def status_dict(self):
        return {"available": True, "enabled": self.gate.enabled(), "tuning": dict(self.tuning)}

    def set_drive_enabled(self, enabled):
        self.calls.append(("gate", enabled))
        self.gate.set_enabled(enabled)
        return self.status_dict()

    def request_follow_to_end(self):
        self.calls.append(("follow", None))
        return {"state": "FOLLOWING_TO_END"}

    def request_face_center_turn(self, command):
        self.calls.append(("face", command))
        return {"state": command}

    def request_line_center_turn(self, command):
        self.calls.append(("line", command))
        return {"state": command}

    def request_manual_turn(self, command):
        self.calls.append(("preset", command))
        return {"state": command}

    def request_roundtrip_stop(self):
        self.calls.append(("stop", None))
        return {"state": "STOPPED"}

    def update_tuning(self, payload):
        self.calls.append(("config", dict(payload)))
        self.tuning.update(payload)
        return self.status_dict()


class FakeController:
    def __init__(self) -> None:
        self.deal_calls: list[dict] = []
        self.stop_calls = 0
        self.deal_result = {"token": "", "state": "pending"}

    def status(self):
        return {"serial": True, "arduino_online": True, "motor_output": [0, 0, 0, 0]}

    def deal_from_key_request(self, request):
        self.deal_calls.append(dict(request))
        return {**self.deal_result, "token": request["token"]}

    def deal_request_status(self, token):
        if not self.deal_calls or self.deal_calls[-1]["token"] != token:
            return None
        return {**self.deal_result, "token": token}

    def stop_now(self):
        self.stop_calls += 1


class RoboticsGatewayTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tracker = FakeTracker()
        self.controller = FakeController()
        self.gateway = RoboticsGateway(self.controller, self.tracker)

    def test_gate_is_explicit_and_idempotent(self) -> None:
        first = self.gateway.set_gate({"enabled": True})
        second = self.gateway.set_gate({"enabled": True})
        self.assertTrue(first["enabled"])
        self.assertTrue(second["enabled"])
        self.assertEqual(self.tracker.calls, [("gate", True), ("gate", True)])

    def test_motion_request_id_is_idempotent_and_cannot_change_meaning(self) -> None:
        payload = {"request_id": "turn-1", "action": "line_recenter_start", "direction": "LEFT"}
        first = self.gateway.execute(payload)
        second = self.gateway.execute(payload)
        queried = self.gateway.request_result("turn-1")
        self.assertEqual(first, second)
        self.assertEqual(queried, first)
        self.assertEqual(first["state"], "START_LEFT")
        self.assertIn("accepted_at", first)
        self.assertIn("updated_at", first)
        self.assertEqual(self.tracker.calls.count(("line", "START_LEFT")), 1)
        with self.assertRaisesRegex(ValueError, "different payload"):
            self.gateway.execute({**payload, "direction": "RIGHT"})
        self.assertIsNone(self.gateway.request_result("unknown-request"))

    def test_dispense_reuses_controller_token_and_marks_evidence_boundary(self) -> None:
        payload = {
            "request_id": "deal-1",
            "action": "dispense_one",
            "feed_pwm": -10,
            "feed_duration_ms": 20,
            "deal_pwm": 30,
            "deal_duration_ms": 40,
        }
        first = self.gateway.execute(payload)
        self.assertEqual(
            self.controller.deal_calls,
            [{
                "token": "deal-1",
                "feed_pwm": -150,
                "feed_duration_ms": 1500,
                "deal_pwm": 150,
                "deal_duration_ms": 400,
            }],
        )
        self.controller.deal_result = {"state": "completed"}
        second = self.gateway.execute(payload)
        queried = self.gateway.request_result("deal-1")
        self.assertEqual(len(self.controller.deal_calls), 1)
        self.assertEqual(first["detail"]["state"], "pending")
        self.assertEqual(second["detail"]["state"], "completed")
        self.assertEqual(queried["detail"]["state"], "completed")
        self.assertFalse(second["physical_card_exit_verified"])
        self.assertEqual(second["completion_evidence"], "arduino_command_ack_only")
        with self.assertRaisesRegex(ValueError, "different payload"):
            self.gateway.execute({**payload, "deal_pwm": 120})

    def test_old_dispense_retry_never_reissues_after_a_new_token(self) -> None:
        first_payload = {"request_id": "deal-1", "action": "dispense_one"}
        second_payload = {"request_id": "deal-2", "action": "dispense_one"}
        original = self.gateway.execute(first_payload)
        self.gateway.execute(second_payload)
        retried = self.gateway.execute(first_payload)
        self.assertEqual(len(self.controller.deal_calls), 2)
        self.assertEqual(retried, original)

    def test_preset_turn_is_validated_and_idempotent(self) -> None:
        payload = {
            "request_id": "preset-1",
            "action": "preset_turn",
            "direction": "LEFT",
            "degrees": 180,
        }
        first = self.gateway.execute(payload)
        second = self.gateway.execute(payload)
        self.assertEqual(first, second)
        self.assertEqual(self.tracker.calls.count(("preset", "LEFT_180")), 1)
        with self.assertRaisesRegex(ValueError, "degrees 90 or 180"):
            self.gateway.execute(
                {
                    "request_id": "preset-invalid",
                    "action": "preset_turn",
                    "direction": "RIGHT",
                    "degrees": 45,
                }
            )
        for request_id, invalid_degrees in (
            ("preset-fractional", 90.5),
            ("preset-boolean", True),
        ):
            with self.assertRaisesRegex(ValueError, "degrees 90 or 180"):
                self.gateway.execute(
                    {
                        "request_id": request_id,
                        "action": "preset_turn",
                        "direction": "RIGHT",
                        "degrees": invalid_degrees,
                    }
                )

    def test_grouped_config_updates_only_owned_tuning(self) -> None:
        result = self.gateway.update_config(
            {
                "visual_turn": {
                    "face_turn_line_center_deadband_normalized": 0.15,
                    "face_turn_line_center_confirm_frames": 2,
                }
            }
        )
        self.assertEqual(
            result["visual_turn"]["face_turn_line_center_deadband_normalized"], 0.15
        )
        with self.assertRaisesRegex(ValueError, "unknown line_follow fields"):
            self.gateway.update_config({"line_follow": {"red_excess_min": 20}})

if __name__ == "__main__":
    unittest.main()
