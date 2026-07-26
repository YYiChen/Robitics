import unittest

from face_detector import FaceDetectionResult
from face_position_server import face_payload


class FacePositionProtocolTests(unittest.TestCase):
    def test_payload_preserves_signed_and_normalized_offset(self):
        face = FaceDetectionResult(True, 480.0, 200.0, 160.0, -40.0, 100, 120, .91, 640, 480)
        payload = face_payload(face, frame=7, processing_ms=12.34, source_fps=25.0, source="test")
        self.assertTrue(payload["detected"])
        self.assertEqual(payload["offset_x"], 160.0)
        self.assertEqual(payload["offset_x_normalized"], 0.5)
        self.assertEqual(payload["offset_y_normalized"], round(-40.0 / 240.0, 4))

    def test_missing_face_has_no_normalized_offset(self):
        face = FaceDetectionResult(False, None, None, None, None, 0, 0, 0.0, 640, 480)
        payload = face_payload(face, frame=1, processing_ms=1.0, source_fps=1.0, source="test")
        self.assertIsNone(payload["offset_x_normalized"])


if __name__ == "__main__":
    unittest.main()
