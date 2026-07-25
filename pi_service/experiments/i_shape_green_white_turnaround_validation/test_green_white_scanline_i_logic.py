import unittest

import cv2
import numpy as np

from green_white_scanline_i_logic import GreenWhiteHybridScanlineAnalyzer, GreenWhiteScanlineConfig


def green_i_frame(*, transverse=False, red_band=False):
    image = np.full((480, 640, 3), (55, 150, 45), dtype=np.uint8)
    if red_band:
        cv2.rectangle(image, (180, 310), (305, 330), (0, 0, 255), -1)
        cv2.rectangle(image, (335, 310), (460, 330), (0, 0, 255), -1)
    if transverse:
        cv2.line(image, (100, 300), (540, 300), (245, 245, 245), 20)
    cv2.line(image, (320, 479), (320, 80), (245, 245, 245), 20)
    return image


class GreenWhiteScanlineTests(unittest.TestCase):
    def setUp(self):
        self.analyzer = GreenWhiteHybridScanlineAnalyzer(GreenWhiteScanlineConfig(route_path_update_frames=1))

    def test_green_floor_white_stem_is_a_valid_line(self):
        evidence = self.analyzer.analyze(green_i_frame()).evidence
        self.assertTrue(evidence.valid_line)
        self.assertFalse(evidence.line_lost)
        self.assertGreater(evidence.confidence, 0.5)

    def test_green_floor_white_transverse_bar_is_never_the_route(self):
        evidence = self.analyzer.analyze(green_i_frame(transverse=True)).evidence
        self.assertTrue(evidence.valid_line)
        self.assertTrue(evidence.endpoint_detected or evidence.junction_detected)

    def test_split_red_band_pre_authorizes_the_white_t_junction(self):
        evidence = self.analyzer.analyze(green_i_frame(transverse=True, red_band=True)).evidence
        self.assertTrue(evidence.red_marker_detected)
        self.assertIsNotNone(evidence.red_marker_y)
        self.assertGreater(evidence.red_marker_span or 0, 100)
        self.assertTrue(evidence.junction_detected)

    def test_red_band_outside_the_white_route_is_rejected(self):
        image = green_i_frame()
        cv2.rectangle(image, (20, 250), (100, 270), (0, 0, 255), -1)
        evidence = self.analyzer.analyze(image).evidence
        self.assertFalse(evidence.red_marker_detected)

    def test_white_without_green_floor_is_rejected(self):
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        cv2.line(frame, (320, 479), (320, 80), (245, 245, 245), 20)
        evidence = self.analyzer.analyze(frame).evidence
        self.assertTrue(evidence.line_lost)


if __name__ == "__main__":
    unittest.main()
