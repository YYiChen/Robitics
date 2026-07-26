from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

if importlib.util.find_spec("flask"):
    from flask import Flask

    from api.robotics_api import register_robotics_api
else:
    Flask = None
    register_robotics_api = None

from test_robotics_gateway import FakeController, FakeTracker as BaseFakeTracker


class FakeTracker(BaseFakeTracker):
    def request_manual_turn(self, command):
        self.calls.append(("preset", command))
        return {"state": command}


@unittest.skipUnless(Flask is not None, "Flask is required for HTTP facade tests")
class RoboticsGatewayHttpTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tracker = FakeTracker()
        self.controller = FakeController()
        app = Flask(__name__)
        register_robotics_api(app, self.controller, self.tracker)
        self.client = app.test_client()

    def test_lost_action_response_can_be_queried_without_redispatch(self) -> None:
        payload = {
            "request_id": "http-turn-1",
            "action": "face_turn_start",
            "direction": "RIGHT",
        }
        posted = self.client.post("/api/robotics/v1/actions", json=payload)
        queried = self.client.get("/api/robotics/v1/requests/http-turn-1")
        self.assertEqual(posted.status_code, 200)
        self.assertEqual(queried.status_code, 200)
        self.assertEqual(
            queried.get_json()["result"]["request_id"],
            "http-turn-1",
        )
        self.assertEqual(
            self.tracker.calls.count(("face", "START_RIGHT")),
            1,
        )

    def test_unknown_request_id_is_explicit_404(self) -> None:
        response = self.client.get("/api/robotics/v1/requests/not-seen")
        self.assertEqual(response.status_code, 404)
        self.assertFalse(response.get_json()["ok"])

    def test_action_rejects_request_ids_that_are_not_url_safe(self) -> None:
        response = self.client.post(
            "/api/robotics/v1/actions",
            json={"request_id": "not url safe", "action": "face_turn_stop"},
        )
        self.assertEqual(response.status_code, 400)
        self.assertIn("URL-safe", response.get_json()["error"])

    def test_preset_turn_validates_degrees_and_is_idempotent(self) -> None:
        payload = {
            "request_id": "preset-1",
            "action": "preset_turn",
            "direction": "LEFT",
            "degrees": 180,
        }
        first = self.client.post("/api/robotics/v1/actions", json=payload)
        second = self.client.post("/api/robotics/v1/actions", json=payload)
        self.assertEqual(first.status_code, 200)
        self.assertEqual(second.status_code, 200)
        self.assertEqual(
            self.tracker.calls.count(("preset", "LEFT_180")),
            1,
        )

        invalid = self.client.post(
            "/api/robotics/v1/actions",
            json={**payload, "request_id": "preset-invalid", "degrees": 45},
        )
        self.assertEqual(invalid.status_code, 400)
        self.assertIn("degrees 90 or 180", invalid.get_json()["error"])


if __name__ == "__main__":
    unittest.main()
