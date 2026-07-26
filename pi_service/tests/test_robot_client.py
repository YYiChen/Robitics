import unittest

from pi_service.robot_client import RobotClientConfig, RobotWebClient


class StubRobotClient(RobotWebClient):
    def __init__(self):
        super().__init__(RobotClientConfig("http://robot"))
        self.calls = []

    def _request(self, path, payload=None):
        self.calls.append((path, payload))
        if path == "/api/status":
            return {"robot": {"arduino_online": True}}
        if path == "/api/action":
            return {"ok": True, "action": payload["action"]}
        if path == "/api/drive":
            return {"ok": True, **payload}
        if path == "/api/config":
            return {"ok": True, "config": payload}
        if path == "/api/autonomous/face-turn":
            return {"ok": True, "autonomous": {"face_turn_active": payload["action"] != "CANCEL"}}
        return {"ok": True}


class RobotWebClientTests(unittest.TestCase):
    def test_send_action_uses_shared_action_endpoint(self):
        client = StubRobotClient()
        self.assertEqual(client.send_action("pr"), "PR")
        self.assertEqual(client.calls, [("/api/action", {"action": "PR"})])

    def test_invalid_action_never_reaches_network(self):
        client = StubRobotClient()
        with self.assertRaises(ValueError):
            client.send_action("FLY")
        self.assertEqual(client.calls, [])

    def test_online_check_and_stop(self):
        client = StubRobotClient()
        client.require_arduino_online()
        client.stop()
        self.assertEqual(client.calls[-1], ("/api/stop", {}))

    def test_send_drive_pwm_uses_non_persistent_drive_endpoint(self):
        client = StubRobotClient()
        self.assertEqual(client.send_drive_pwm(140, 80), (140, 80))
        self.assertEqual(
            client.calls,
            [("/api/drive", {"right_pwm": 140, "left_pwm": 80})],
        )

    def test_face_turn_start_observation_and_cancel_use_dedicated_endpoint(self):
        client = StubRobotClient()
        self.assertTrue(client.start_face_turn("left")["face_turn_active"])
        client.send_face_observation(found=True, frame_width=640, center_x=300)
        self.assertFalse(client.cancel_face_turn()["face_turn_active"])
        self.assertEqual(
            client.calls,
            [
                ("/api/autonomous/face-turn", {"action": "START", "direction": "LEFT"}),
                ("/api/autonomous/face-turn", {"action": "OBSERVE", "found": True, "frame_width": 640, "center_x": 300.0}),
                ("/api/autonomous/face-turn", {"action": "CANCEL"}),
            ],
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
