import unittest

import cv2
import numpy as np

from green_white_scanline_i_logic import GreenWhiteHybridScanlineAnalyzer, GreenWhiteScanlineConfig
from scanline_i_logic import IShapeTurnaroundPlanner, ScanlineEvidence, TurnaroundConfig, TurnaroundState


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

    def test_pale_green_cast_floor_is_not_the_green_course(self):
        # This BGR colour occurs on the pale room floor under the camera's
        # colour cast: HSV alone can be marginally green, but RGB green
        # dominance correctly rejects it as not being the mat.
        frame = np.full((480, 640, 3), (157, 193, 169), dtype=np.uint8)
        cv2.line(frame, (320, 479), (320, 80), (245, 245, 245), 20)
        evidence = self.analyzer.analyze(frame).evidence
        self.assertTrue(evidence.line_lost)

    def test_pale_floor_patch_touching_green_on_one_side_is_rejected(self):
        image = np.full((480, 640, 3), (55, 150, 45), dtype=np.uint8)
        # Broad bright floor touches the green course only along its lower
        # edge.  It passes the permissive HSV candidate mask but does not
        # have a green-supported tape backbone.
        cv2.rectangle(image, (120, 100), (639, 385), (215, 215, 215), -1)
        result = self.analyzer.analyze(image)
        self.assertTrue(result.evidence.line_lost)
        self.assertEqual(int(np.count_nonzero(result.component_mask)), 0)

    def test_tape_can_continue_out_of_green_roi_in_far_field(self):
        image = np.full((480, 640, 3), (55, 150, 45), dtype=np.uint8)
        # Real camera view: the far tip of the same tape points past the mat
        # into the room.  Only the near course section must have green on both
        # sides; this remains a valid route.
        image[:170, :] = (215, 215, 215)
        cv2.line(image, (320, 479), (320, 60), (245, 245, 245), 24)
        evidence = self.analyzer.analyze(image).evidence
        self.assertTrue(evidence.valid_line)
        self.assertFalse(evidence.line_lost)

    def test_side_entering_tape_on_the_green_mat_is_not_clipped_by_a_centre_roi(self):
        image = np.full((480, 640, 3), (55, 150, 45), dtype=np.uint8)
        # This represents the side-entry view during a pivot.  The white tape
        # remains on the mat but is well left of image centre.
        cv2.line(image, (110, 479), (215, 80), (245, 245, 245), 24)
        result = self.analyzer.analyze(image)
        self.assertGreater(int(np.count_nonzero(result.component_mask)), 0)

    def test_confirmed_white_bar_brakes_when_near_red_band_exits_bottom(self):
        planner = IShapeTurnaroundPlanner(
            TurnaroundConfig(
                endpoint_confirm_frames=1,
                junction_confirm_frames=1,
                red_exit_enabled=True,
                red_exit_arm_y_ratio=.84,
            )
        )
        def evidence(*, endpoint=False, red_y=None):
            return ScanlineEvidence(
                confidence=.9, valid_line=True, line_lost=False,
                line_center_x=320.0, line_centers=((440, 320.0, 20),),
                endpoint_detected=endpoint, endpoint_y=340 if endpoint else None,
                endpoint_width=400 if endpoint else None, normal_tape_width=20.0,
                junction_detected=True, junction_y=300, junction_arm_count=3,
                red_marker_detected=red_y is not None, red_marker_y=red_y,
                red_marker_span=260 if red_y is not None else None, frame_height=480,
            )
        self.assertIs(planner.step(evidence(endpoint=True, red_y=420), 0.0).state, TurnaroundState.EARLY_BAR_PREDICTED)
        self.assertIs(planner.step(evidence(endpoint=True, red_y=430), .05).state, TurnaroundState.BAR_MARKED)
        decision = planner.step(evidence(endpoint=False, red_y=None), .10)
        self.assertIs(decision.state, TurnaroundState.BRAKE_BEFORE_PIVOT)
        self.assertEqual(decision.reason, "confirmed_white_bar_red_marker_exited_bottom_braking")

    def test_red_exit_does_not_brake_without_white_bar_confirmation(self):
        planner = IShapeTurnaroundPlanner(TurnaroundConfig(junction_confirm_frames=1, red_exit_enabled=True))
        def evidence(red_y):
            return ScanlineEvidence(
                confidence=.9, valid_line=True, line_lost=False,
                line_center_x=320.0, line_centers=((440, 320.0, 20),),
                endpoint_detected=False, endpoint_y=None, endpoint_width=None,
                normal_tape_width=20.0, junction_detected=True, junction_y=300,
                junction_arm_count=3, red_marker_detected=red_y is not None,
                red_marker_y=red_y, red_marker_span=260 if red_y is not None else None,
                frame_height=480,
            )
        self.assertIs(planner.step(evidence(430), 0.0).state, TurnaroundState.EARLY_BAR_PREDICTED)
        self.assertIs(planner.step(evidence(None), .05).state, TurnaroundState.EARLY_BAR_PREDICTED)


if __name__ == "__main__":
    unittest.main()
