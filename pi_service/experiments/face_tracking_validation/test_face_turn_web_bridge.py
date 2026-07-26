import unittest
from unittest.mock import patch
from datetime import datetime, timedelta, timezone
import json
from types import SimpleNamespace

from face_turn_web_bridge import (
    DEFAULT_FACE_DEADBAND_NORMALIZED,
    FaceTurnBridge,
    FaceStopArmer,
    decode_json_object,
    is_fresh_and_centred,
    is_fresh_payload,
    post_command,
    robotics_action_payload,
    robotics_route_status,
)


class JsonResponse:
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self):
        return b'{"ok":true,"result":{"accepted":true}}'


class FaceTurnWebBridgeTests(unittest.TestCase):
    def test_decodes_direct_and_pi_legacy_double_encoded_json(self):
        payload = {"autonomous": {"motion_phase": "FOLLOW"}}
        direct = json.dumps(payload).encode("utf-8")
        double_encoded = json.dumps(json.dumps(payload)).encode("utf-8")
        self.assertEqual(decode_json_object(direct), payload)
        self.assertEqual(decode_json_object(double_encoded), payload)

    def test_rejects_non_object_json(self):
        with self.assertRaises(ValueError):
            decode_json_object(b"[]")

    def test_default_stop_zone_is_thirty_percent(self):
        self.assertEqual(DEFAULT_FACE_DEADBAND_NORMALIZED, .30)

    def test_initially_centred_face_must_depart_before_it_can_stop_return(self):
        armer = FaceStopArmer()
        self.assertFalse(armer.should_stop(True))
        self.assertFalse(armer.should_stop(False))
        self.assertTrue(armer.should_stop(True))
        armer.reset()
        self.assertFalse(armer.should_stop(True))

    def test_only_fresh_confident_centred_face_stops_turn(self):
        now = datetime.now(timezone.utc).isoformat()
        face = {"detected": True, "score": .9, "offset_x_normalized": .06, "time": now}
        self.assertTrue(is_fresh_and_centred(face, minimum_score=.5, deadband_normalized=.08, max_age_ms=450))
        face["offset_x_normalized"] = .3
        self.assertFalse(is_fresh_and_centred(face, minimum_score=.5, deadband_normalized=.08, max_age_ms=450))
        face["offset_x_normalized"] = 0.0
        face["time"] = (datetime.now(timezone.utc) - timedelta(seconds=1)).isoformat()
        self.assertFalse(is_fresh_and_centred(face, minimum_score=.5, deadband_normalized=.08, max_age_ms=450))
        self.assertFalse(is_fresh_payload(face, max_age_ms=450))

    def test_bridge_uses_versioned_robotics_actions(self):
        self.assertEqual(
            robotics_action_payload("HEARTBEAT", "heartbeat-1"),
            {
                "request_id": "heartbeat-1",
                "action": "face_turn_heartbeat",
            },
        )
        self.assertEqual(
            robotics_action_payload("stop", "stop-1"),
            {
                "request_id": "stop-1",
                "action": "face_turn_stop",
            },
        )
        with self.assertRaises(ValueError):
            robotics_action_payload("START_LEFT", "start-1")

    def test_bridge_requires_versioned_robotics_status_envelope(self):
        route = robotics_route_status(
            {
                "ok": True,
                "status": {"route": {"motion_phase": "FOLLOW"}},
            }
        )
        self.assertEqual(route["motion_phase"], "FOLLOW")
        with self.assertRaises(ValueError):
            robotics_route_status(
                {"autonomous": {"motion_phase": "FOLLOW"}}
            )

    def test_post_command_uses_formal_endpoint_and_unique_heartbeat_ids(self):
        requests = []

        def capture(request, timeout):
            requests.append((request, timeout))
            return JsonResponse()

        uuids = [SimpleNamespace(hex="aaa"), SimpleNamespace(hex="bbb")]
        with (
            patch("face_turn_web_bridge.urlopen", side_effect=capture),
            patch("face_turn_web_bridge.uuid4", side_effect=uuids),
        ):
            first = post_command("http://pi:5000/", "HEARTBEAT")
            second = post_command("http://pi:5000", "HEARTBEAT")

        self.assertTrue(first["ok"])
        self.assertTrue(second["ok"])
        self.assertEqual(len(requests), 2)
        payloads = [json.loads(request.data) for request, _timeout in requests]
        self.assertEqual(
            [request.full_url for request, _timeout in requests],
            [
                "http://pi:5000/api/robotics/v1/actions",
                "http://pi:5000/api/robotics/v1/actions",
            ],
        )
        self.assertEqual(
            [payload["action"] for payload in payloads],
            ["face_turn_heartbeat", "face_turn_heartbeat"],
        )
        self.assertEqual(
            [payload["request_id"] for payload in payloads],
            [
                "face-bridge-heartbeat-aaa",
                "face-bridge-heartbeat-bbb",
            ],
        )

    def test_embedded_bridge_heartbeats_then_stops_after_face_returns(self):
        face = {
            "time": datetime.now(timezone.utc).isoformat(),
            "detected": False,
            "score": 0.0,
            "offset_x_normalized": None,
            "error": "",
        }
        commands = []
        bridge = FaceTurnBridge(
            face_provider=lambda: dict(face),
            pi_url="http://pi:5000",
            status_provider=lambda: {
                "ok": True,
                "status": {
                    "route": {"motion_phase": "FACE_CENTER_TURN"}
                },
            },
            command_poster=lambda pi_url, command: commands.append(
                (pi_url, command)
            )
            or {"ok": True},
        )

        first = bridge.step()
        self.assertEqual(first["action"], "heartbeat")
        self.assertTrue(first["face_stop_armed"])

        face.update(
            time=datetime.now(timezone.utc).isoformat(),
            detected=True,
            score=.9,
            offset_x_normalized=.05,
        )
        second = bridge.step()
        self.assertEqual(second["action"], "stop_centered")
        self.assertEqual(
            commands,
            [
                ("http://pi:5000", "HEARTBEAT"),
                ("http://pi:5000", "STOP"),
            ],
        )

    def test_embedded_bridge_never_heartbeats_bad_or_stale_publisher(self):
        commands = []
        face = {
            "time": datetime.now(timezone.utc).isoformat(),
            "detected": False,
            "error": "camera_busy",
        }
        bridge = FaceTurnBridge(
            face_provider=lambda: dict(face),
            pi_url="http://pi:5000",
            status_provider=lambda: {
                "ok": True,
                "status": {
                    "route": {"motion_phase": "FACE_CENTER_TURN"}
                },
            },
            command_poster=lambda pi_url, command: commands.append(
                (pi_url, command)
            )
            or {"ok": True},
        )
        with self.assertRaisesRegex(RuntimeError, "face_publisher_error"):
            bridge.step()
        self.assertEqual(commands, [])

        face["error"] = ""
        face["time"] = (
            datetime.now(timezone.utc) - timedelta(seconds=2)
        ).isoformat()
        with self.assertRaisesRegex(RuntimeError, "face_publisher_stale"):
            bridge.step()
        self.assertEqual(commands, [])

    def test_embedded_bridge_is_idle_until_state_machine_starts_turn(self):
        commands = []
        bridge = FaceTurnBridge(
            face_provider=lambda: self.fail("idle bridge must not read a face"),
            pi_url="http://pi:5000",
            status_provider=lambda: {
                "ok": True,
                "status": {"route": {"motion_phase": "FOLLOW"}},
            },
            command_poster=lambda pi_url, command: commands.append(
                (pi_url, command)
            )
            or {"ok": True},
        )
        record = bridge.step()
        self.assertEqual(record["action"], "idle")
        self.assertEqual(commands, [])


if __name__ == "__main__":
    unittest.main()
