import unittest

from multi_camera_face_position_server import choose_primary


class MultiCameraFusionTests(unittest.TestCase):
    def test_prefers_larger_live_face_without_averaging_offsets(self):
        sources = {
            "pi": {"detected": True, "box_width": 80, "box_height": 80, "score": .98, "offset_x": -120.0, "offset_y": 1.0, "offset_x_normalized": -.375, "offset_y_normalized": .01},
            "phone": {"detected": True, "box_width": 100, "box_height": 100, "score": .70, "offset_x": 25.0, "offset_y": 2.0, "offset_x_normalized": .08, "offset_y_normalized": .01},
        }
        fused = choose_primary(sources)
        self.assertEqual(fused["primary_source"], "phone")
        self.assertEqual(fused["offset_x"], 25.0)
        self.assertTrue(fused["both_detected"])

    def test_no_visible_face_has_no_position(self):
        fused = choose_primary({"pi": {"detected": False}, "phone": {"detected": False}})
        self.assertFalse(fused["detected"])
        self.assertIsNone(fused["offset_x"])


if __name__ == "__main__":
    unittest.main()
