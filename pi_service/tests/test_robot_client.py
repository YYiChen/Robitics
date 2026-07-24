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
        if path == "/api/config":
            return {"ok": True, "config": payload}
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


if __name__ == "__main__":
    unittest.main(verbosity=2)
