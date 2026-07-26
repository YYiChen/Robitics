from __future__ import annotations

import unittest
from pathlib import Path


APP_JS = (
    Path(__file__).parents[1] / "robot_web" / "static" / "app.js"
).read_text(encoding="utf-8")


class WebControlBackpressureTests(unittest.TestCase):
    def test_periodic_status_requests_are_completion_scheduled(self) -> None:
        self.assertNotIn("setInterval(refreshStatus", APP_JS)
        self.assertNotIn("setInterval(refreshFaceDetectionStatus", APP_JS)
        self.assertIn("async function runStatusRefresh()", APP_JS)
        self.assertIn(
            "setTimeout(runStatusRefresh, STATUS_REFRESH_INTERVAL_MS)",
            APP_JS,
        )
        self.assertIn("async function runFaceStatusRefresh()", APP_JS)

    def test_browser_heartbeat_has_single_inflight_guard(self) -> None:
        self.assertIn("if (browserHeartbeatBusy) return", APP_JS)
        self.assertIn("browserHeartbeatBusy = true", APP_JS)
        self.assertIn("browserHeartbeatBusy = false", APP_JS)

    def test_timeout_error_warns_that_server_result_is_unknown(self) -> None:
        self.assertIn('timeout.name = "RequestTimeoutError"', APP_JS)
        self.assertIn("结果未知，请先查询状态，不要盲目重发", APP_JS)
        self.assertIn("error instanceof TypeError", APP_JS)

    def test_one_shot_motion_uses_queryable_robotics_requests(self) -> None:
        self.assertIn('"/api/robotics/v1/actions"', APP_JS)
        self.assertIn("/api/robotics/v1/requests/", APP_JS)
        self.assertNotIn(
            'requestJson("/api/autonomous/manual-turn"',
            APP_JS,
        )
        self.assertNotIn(
            'requestJson("/api/autonomous/face-turn"',
            APP_JS,
        )
        self.assertNotIn(
            'requestJson("/api/autonomous/line-turn"',
            APP_JS,
        )
        self.assertIn("command === activeFaceTurnCommand", APP_JS)
        self.assertIn("if (stopRequestInFlight) return stopRequestInFlight", APP_JS)


if __name__ == "__main__":
    unittest.main()
