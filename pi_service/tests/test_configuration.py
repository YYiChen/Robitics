import json
import sys
import tempfile
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))
sys.modules.setdefault("serial", types.SimpleNamespace(Serial=None))

from camera import CameraStreamer
from configuration import DEFAULT_CONFIG_PATH, UnifiedConfigStore
from controller import RobotController
from end_line_turn_adaptor import EndLineTurnAdaptorRouteTracker


class UnifiedConfigurationTests(unittest.TestCase):
    def make_store(self, directory: str, defaults: dict) -> UnifiedConfigStore:
        root = Path(directory)
        defaults_path = root / "defaults.json"
        defaults_path.write_text(
            json.dumps({"schema_version": 1, **defaults}),
            encoding="utf-8",
        )
        return UnifiedConfigStore(defaults_path, root / "local.json")

    def test_local_values_deep_merge_over_tracked_defaults(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = self.make_store(
                directory,
                {"camera": {"mode": "fast_1640", "exposure": {"auto": True, "ev": 0.0}}},
            )
            store.write_section("camera", {"exposure": {"ev": 1.5}})

            camera = store.read_section("camera")

            self.assertEqual(camera["mode"], "fast_1640")
            self.assertTrue(camera["exposure"]["auto"])
            self.assertEqual(camera["exposure"]["ev"], 1.5)

    def test_section_write_preserves_other_sections(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = self.make_store(directory, {})
            store.write_section("drive", {"target_speed": 30})
            store.write_section("routes.end_line", {"straight_pwm": 91})

            local = json.loads(store.local_path.read_text(encoding="utf-8"))

            self.assertEqual(local["drive"]["target_speed"], 30)
            self.assertEqual(local["routes"]["end_line"]["straight_pwm"], 91)
            self.assertEqual(local["schema_version"], 1)

    def test_legacy_migration_never_overwrites_existing_local_section(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = self.make_store(directory, {})
            self.assertTrue(store.migrate_section("drive", {"target_speed": 41}))
            self.assertFalse(store.migrate_section("drive", {"target_speed": 99}))
            self.assertEqual(store.read_section("drive")["target_speed"], 41)

    def test_controller_reads_and_writes_only_drive_section(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = self.make_store(
                directory,
                {
                    "drive": {"target_speed": 35, "straight_pwm": 80},
                    "camera": {"highres_fps": 2.0},
                },
            )
            controller = RobotController("unused", config_store=store)
            self.assertEqual(controller.config_source, "unified_config:drive")
            self.assertEqual(controller.config.target_speed, 35.0)

            controller.update_config({"target_speed": 52})

            self.assertEqual(store.read_section("drive")["target_speed"], 52.0)
            self.assertEqual(store.read_section("camera")["highres_fps"], 2.0)

    def test_camera_reads_and_writes_only_camera_section(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = self.make_store(
                directory,
                {
                    "drive": {"target_speed": 35},
                    "camera": {
                        "mode": "fast_1640",
                        "stream_profile": "low_latency",
                        "highres_profile": "medium_1640",
                        "highres_fps": 2.0,
                        "color_correction": {"enabled": True, "strength": 1.0},
                        "exposure": {"auto": True, "ev": 0.0, "shutter_denominator": 200},
                    },
                },
            )
            camera = CameraStreamer(config_store=store)
            camera.set_highres_fps(5)

            self.assertEqual(store.read_section("camera")["highres_fps"], 5.0)
            self.assertEqual(store.read_section("drive")["target_speed"], 35)

    def test_current_route_reads_and_writes_only_end_line_section(self) -> None:
        class Gate:
            def enabled(self) -> bool:
                return False

        with tempfile.TemporaryDirectory() as directory:
            store = UnifiedConfigStore(DEFAULT_CONFIG_PATH, Path(directory) / "local.json")
            tracker = EndLineTurnAdaptorRouteTracker(
                object(), object(), object(), Gate(), config_store=store
            )

            status = tracker.update_tuning({
                "straight_pwm": 93,
                "turn_90_step_seconds": 0.45,
            })

            self.assertEqual(status["tuning"]["straight_pwm"], 93)
            self.assertEqual(status["tuning"]["turn_90_step_seconds"], 0.45)
            self.assertEqual(store.read_section("routes.end_line")["straight_pwm"], 93)
            self.assertNotIn("drive", json.loads(store.local_path.read_text(encoding="utf-8")))


if __name__ == "__main__":
    unittest.main()
