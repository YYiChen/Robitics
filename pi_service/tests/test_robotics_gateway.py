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
        self.state = "MANUAL_COMPLETE"
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
        return {
            "available": True,
            "enabled": self.gate.enabled(),
            "state": self.state,
            "tuning": dict(self.tuning),
        }

    def set_drive_enabled(self, enabled):
        self.calls.append(("gate", enabled))
        self.gate.set_enabled(enabled)
        return self.status_dict()

    def request_follow_to_end(self):
        self.calls.append(("follow", None))
        self.state = "FOLLOWING_TO_END"
        return self.status_dict()

    def request_face_center_turn(self, command):
        self.calls.append(("face", command))
        self.state = {
            "START_LEFT": "FACE_CENTER_TURN",
            "START_RIGHT": "FACE_CENTER_TURN",
            "STOP": "FACE_CENTERED_STOP",
        }.get(command, self.state)
        return self.status_dict()

    def request_line_center_turn(self, command):
        self.calls.append(("line", command))
        self.state = {
            "START_LEFT": "LINE_CENTER_TURN",
            "START_RIGHT": "LINE_CENTER_TURN",
            "STOP": "LINE_TURN_STOPPED",
        }.get(command, self.state)
        return self.status_dict()

    def request_manual_turn(self, command):
        self.calls.append(("preset", command))
        self.state = "MANUAL_STEP"
        return self.status_dict()

    def request_roundtrip_stop(self):
        self.calls.append(("stop", None))
        self.state = "ROUNDTRIP_STOPPED"
        return self.status_dict()

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
        self.tracker.state = "LINE_TURN_CENTERED"
        queried = self.gateway.request_result("turn-1")
        second = self.gateway.execute(payload)
        self.assertEqual(queried, second)
        self.assertEqual(first["request_status"], "running")
        self.assertFalse(first["terminal"])
        self.assertEqual(queried["state"], "LINE_TURN_CENTERED")
        self.assertEqual(queried["request_status"], "succeeded")
        self.assertTrue(queried["terminal"])
        self.assertIn("completed_at", queried)
        self.assertIn("accepted_at", first)
        self.assertIn("updated_at", first)
        self.assertEqual(self.tracker.calls.count(("line", "START_LEFT")), 1)
        with self.assertRaisesRegex(ValueError, "different payload"):
            self.gateway.execute({**payload, "direction": "RIGHT"})
        self.assertIsNone(self.gateway.request_result("unknown-request"))

    def test_dispense_reuses_controller_token_and_marks_evidence_boundary(self) -> None:
        payload = {"request_id": "deal-1", "action": "dispense_one"}
        first = self.gateway.execute(payload)
        self.controller.deal_result = {"state": "completed"}
        second = self.gateway.execute(payload)
        queried = self.gateway.request_result("deal-1")
        self.assertEqual(len(self.controller.deal_calls), 1)
        self.assertEqual(first["detail"]["state"], "pending")
        self.assertEqual(first["request_status"], "running")
        self.assertEqual(second["detail"]["state"], "completed")
        self.assertEqual(second["request_status"], "succeeded")
        self.assertTrue(second["terminal"])
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
        self.assertEqual(original["request_status"], "running")
        self.assertEqual(retried["request_status"], "cancelled")
        self.assertEqual(retried["completed_by_request_id"], "deal-2")

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
        self.assertEqual(first["request_status"], "running")
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

    def test_stop_cancels_active_follow_with_correlated_terminal_result(self) -> None:
        follow = self.gateway.execute(
            {"request_id": "board-follow", "action": "follow_line_to_end"}
        )
        self.assertEqual(follow["request_status"], "running")
        stopped = self.gateway.execute(
            {"request_id": "board-stop", "action": "stop"}
        )
        final_follow = self.gateway.request_result("board-follow")
        self.assertEqual(stopped["request_status"], "succeeded")
        self.assertTrue(stopped["terminal"])
        self.assertIsNotNone(final_follow)
        self.assertEqual(final_follow["request_status"], "cancelled")
        self.assertEqual(
            final_follow["completed_by_request_id"],
            "board-stop",
        )

    def test_face_stop_completes_the_matching_start_request(self) -> None:
        self.gateway.execute(
            {
                "request_id": "face-start",
                "action": "face_turn_start",
                "direction": "RIGHT",
            }
        )
        self.gateway.execute(
            {"request_id": "face-stop", "action": "face_turn_stop"}
        )
        final_start = self.gateway.request_result("face-start")
        self.assertIsNotNone(final_start)
        self.assertEqual(final_start["request_status"], "succeeded")
        self.assertEqual(
            final_start["completed_by_request_id"],
            "face-stop",
        )

if __name__ == "__main__":
    unittest.main()
