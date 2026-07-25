from __future__ import annotations
import unittest
from protocol import parse_event


class ProtocolTests(unittest.TestCase):
    def test_accepts_fresh_authorized_event(self):
        event = parse_event({"token": "x", "event": "slow_down", "frame_seq": 3, "captured_at_ms": 1000}, token="x", now_ms=1200)
        self.assertEqual(event.event, "SLOW_DOWN")

    def test_rejects_stale_unknown_and_bad_token(self):
        with self.assertRaises(ValueError):
            parse_event({"token": "x", "event": "PWM", "frame_seq": 3, "captured_at_ms": 1000}, token="x", now_ms=1100)
        with self.assertRaises(ValueError):
            parse_event({"token": "no", "event": "SLOW_DOWN", "frame_seq": 3, "captured_at_ms": 1000}, token="x", now_ms=1100)
        with self.assertRaises(ValueError):
            parse_event({"token": "x", "event": "SLOW_DOWN", "frame_seq": 3, "captured_at_ms": 1000}, token="x", now_ms=1800)
