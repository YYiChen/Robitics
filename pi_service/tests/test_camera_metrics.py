import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

from camera import CameraStreamer


class CameraMetricsTests(unittest.TestCase):
    def test_reports_encoded_and_stream_bandwidth(self) -> None:
        camera = CameraStreamer(width=1280, height=720, fps=15)
        camera._record_encoded(100_000, 12.5)
        camera._record_stream(100_050)

        status = camera.status_dict()
        self.assertEqual(status["resolution"], "1280x720")
        self.assertEqual(status["capture_fps"], 1)
        self.assertEqual(status["jpeg_bytes"], 100_000)
        self.assertAlmostEqual(status["jpeg_kBps"], 100.0)
        self.assertAlmostEqual(status["jpeg_kbps"], 800.0)
        self.assertAlmostEqual(status["stream_kbps"], 800.4)
        self.assertAlmostEqual(status["encode_ms"], 12.5)

    def test_camera_mode_is_persisted(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "camera_config.json"
            first = CameraStreamer(config_path=config_path)
            first.mode_key = "full_3280"
            first._save_mode()
            second = CameraStreamer(config_path=config_path)
            self.assertEqual(second.mode_key, "full_3280")
            self.assertEqual(second.status_dict()["resolution"], "3280x2464")


if __name__ == "__main__":
    unittest.main()
