import re
import unittest
from pathlib import Path


class ApiStructureTests(unittest.TestCase):
    def test_endpoint_modules_preserve_formal_http_paths_without_duplicates(self) -> None:
        api_root = Path(__file__).parents[1] / "robot_web" / "api"
        found: list[str] = []
        for path in api_root.glob("*_api.py"):
            found.extend(
                re.findall(r'@app\.(?:get|post)\("([^"]+)"\)', path.read_text(encoding="utf-8"))
            )
        expected = {
            "/", "/video_feed", "/highres_feed", "/route_preview_feed",
            "/api/status", "/api/action", "/api/drive", "/api/keys",
            "/api/heartbeat", "/api/stop", "/api/deal", "/api/feed",
            "/api/servo", "/api/reconnect", "/api/config",
            "/api/camera/highres/latest", "/api/camera/mode",
            "/api/camera/exposure", "/api/camera/stream-profile",
            "/api/camera/highres-profile", "/api/camera/highres-fps",
            "/api/camera/color-correction", "/api/vision-adaptor/frame",
            "/api/vision-adaptor/event", "/api/vision-adaptor/preview",
            "/api/autonomous/toggle", "/api/autonomous/tuning",
            "/api/autonomous/manual-turn", "/api/autonomous/face-turn",
            "/api/autonomous/follow-to-end",
        }
        self.assertEqual(set(found), expected)
        self.assertEqual(len(found), len(set(found)))

    def test_app_is_assembly_not_endpoint_implementation(self) -> None:
        app_source = (
            Path(__file__).parents[1] / "robot_web" / "app.py"
        ).read_text(encoding="utf-8")
        self.assertNotRegex(app_source, r'@app\.(?:get|post)\(')
        for registrar in (
            "register_camera_api", "register_control_api",
            "register_route_api", "register_status_api",
        ):
            self.assertIn(registrar, app_source)


if __name__ == "__main__":
    unittest.main()
