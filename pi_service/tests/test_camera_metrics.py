import sys
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


if __name__ == "__main__":
    unittest.main()
