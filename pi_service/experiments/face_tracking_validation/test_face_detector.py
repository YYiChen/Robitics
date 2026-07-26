import unittest

import numpy as np

from face_detector import FaceDetector


class FaceDetectorTests(unittest.TestCase):
    def test_cascade_loads_and_blank_frame_has_no_face(self):
        detector = FaceDetector()
        result = detector.detect(np.zeros((480, 640, 3), dtype=np.uint8))
        self.assertFalse(result.detected)
        self.assertEqual(result.frame_width, 640)
        self.assertEqual(result.width, 0)


if __name__ == "__main__":
    unittest.main()
