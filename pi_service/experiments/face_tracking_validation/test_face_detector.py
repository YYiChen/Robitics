"""Unit test: FaceDetector on synthetic frames (no camera needed)."""
import unittest
import numpy as np
from face_detector import FaceDetector


class FaceDetectorTests(unittest.TestCase):
    def setUp(self):
        self.detector = FaceDetector()

    def tearDown(self):
        self.detector.close()

    def test_no_face_on_blank_frame(self):
        frame = np.full((480, 640, 3), 128, dtype=np.uint8)
        result = self.detector.detect(frame)
        self.assertFalse(result.detected)
        self.assertIsNone(result.center_x)

    def test_result_structure_when_no_face(self):
        frame = np.full((480, 640, 3), 128, dtype=np.uint8)
        result = self.detector.detect(frame)
        self.assertEqual(result.frame_width, 640)
        self.assertEqual(result.frame_height, 480)
        self.assertEqual(result.score, 0.0)

    def test_same_size_frame_twice(self):
        """Verify detector handles back-to-back frames without error."""
        frame = np.full((480, 640, 3), 128, dtype=np.uint8)
        r1 = self.detector.detect(frame)
        r2 = self.detector.detect(frame)
        self.assertFalse(r1.detected)
        self.assertFalse(r2.detected)


if __name__ == "__main__":
    unittest.main()
