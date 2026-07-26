import json
import sys
import tempfile
import unittest
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

from media.metrics import compact_window_stats, window_stats
from media.settings import CameraSettingsRepository, normalize_camera_settings


class MediaModuleTests(unittest.TestCase):
    def test_settings_normalization_clamps_without_camera_hardware(self) -> None:
        settings = normalize_camera_settings({
            "mode": "invalid",
            "highres_fps": 999,
            "exposure": {"ev": -99, "shutter_denominator": 0},
            "color_correction": {"strength": 9},
        })
        self.assertEqual(settings["mode"], "fast_1640")
        self.assertEqual(settings["highres_fps"], 30.0)
        self.assertEqual(settings["exposure"]["ev"], -8.0)
        self.assertEqual(settings["exposure"]["shutter_denominator"], 1)
        self.assertEqual(settings["color_correction"]["strength"], 1.5)

    def test_settings_repository_round_trip_is_atomic_and_hardware_free(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.json"
            repository = CameraSettingsRepository(path)
            repository.save({"mode": "full_3280", "highres_fps": 5})
            self.assertEqual(repository.load()["mode"], "full_3280")
            self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["highres_fps"], 5)

    def test_metric_windows_drop_stale_events(self) -> None:
        events = deque([(1.0, 100), (2.5, 200)])
        self.assertEqual(window_stats(events, 3.0), (1, 200, 1.0))
        self.assertEqual(compact_window_stats(events, 3.0), (1, 200))


if __name__ == "__main__":
    unittest.main()
