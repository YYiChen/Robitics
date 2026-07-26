from __future__ import annotations

import unittest
from pathlib import Path

from droidcam_face_turn import parse_source


class DroidCamFaceTurnTests(unittest.TestCase):
    def test_numeric_camera_index_and_http_source_are_parsed(self):
        self.assertEqual(parse_source("1"), 1)
        self.assertEqual(parse_source(" http://192.168.1.2:4747/video "), "http://192.168.1.2:4747/video")

    def test_dedicated_window_uses_j_l_and_does_not_claim_p(self):
        source = Path(__file__).with_name("droidcam_face_turn.py").read_text(encoding="utf-8")
        self.assertIn('ord("j")', source)
        self.assertIn('ord("l")', source)
        self.assertNotIn('ord("p")', source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
