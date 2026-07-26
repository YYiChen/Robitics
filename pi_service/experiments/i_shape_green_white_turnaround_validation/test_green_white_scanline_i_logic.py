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

    def test_near_bar_remains_endpoint_evidence_without_a_steering_component(self):
        image = np.full((480, 640, 3), (55, 150, 45), dtype=np.uint8)
        # At the turn position the near bar can fill the frame.  It is not a
        # valid longitudinal steering route, but must still be reported as a
        # confirmed endpoint rather than disappearing completely.
        cv2.rectangle(image, (20, 370), (620, 460), (245, 245, 245), -1)
        result = self.analyzer.analyze(image)
        self.assertFalse(result.evidence.valid_line)
        self.assertEqual(int(np.count_nonzero(result.component_mask)), 0)
        self.assertTrue(result.evidence.endpoint_detected)
        self.assertIsNotNone(result.evidence.endpoint_y)
        self.assertGreater(int(np.count_nonzero(self.analyzer.tape_candidate_mask)), 0)

    def test_white_and_red_course_bridges_reconnect_green_below_a_near_bar(self):
        image = np.full((480, 640, 3), (55, 150, 45), dtype=np.uint8)
        # White tape plus the red warning band completely split the raw-green
        # pixels into upper/lower islands.  The recovered course must contain
        # green on both sides, including the lower driving field.
        cv2.rectangle(image, (0, 350), (639, 420), (245, 245, 245), -1)
        cv2.rectangle(image, (180, 421), (460, 445), (0, 0, 255), -1)
        self.analyzer.analyze(image)
        field = self.analyzer.course_field_mask
        self.assertIsNotNone(field)
        self.assertGreater(int(np.count_nonzero(field[300:340, :])), 0)
        self.assertGreater(int(np.count_nonzero(field[450:479, :])), 0)

    def test_upper_green_tinted_background_cannot_join_the_bottom_course(self):
        image = np.full((480, 640, 3), (210, 210, 210), dtype=np.uint8)
        # A green-tinted upper background is deliberately separated from the
        # real lower carpet by a bright boundary.  No far-field white bridge
        # may make it part of the selected course contour.
        image[40:205, 40:600] = (55, 150, 45)
        image[265:, :] = (55, 150, 45)
        cv2.rectangle(image, (0, 220), (639, 252), (245, 245, 245), -1)
        self.analyzer.analyze(image)
        field = self.analyzer.course_field_mask
        self.assertIsNotNone(field)
        self.assertEqual(int(np.count_nonzero(field[80:180, :])), 0)
        self.assertGreater(int(np.count_nonzero(field[360:470, :])), 0)

    def test_white_wall_above_fixed_green_boundary_is_not_a_tape_candidate(self):
        image = np.full((480, 640, 3), (245, 245, 245), dtype=np.uint8)
        image[220:, :] = (55, 150, 45)
        # Real tape remains below the fixed green boundary; the white wall is
        # above it and must neither highlight nor influence recognition.
        cv2.line(image, (320, 470), (320, 270), (245, 245, 245), 20)
        result = self.analyzer.analyze(image)
        candidate = self.analyzer.tape_candidate_mask
        self.assertEqual(int(np.count_nonzero(candidate[:220, :])), 0)
        self.assertGreater(int(np.count_nonzero(candidate[270:, :])), 0)
        self.assertTrue(result.evidence.valid_line)

    def test_split_red_band_pre_authorizes_the_white_t_junction(self):
        evidence = self.analyzer.analyze(green_i_frame(transverse=True, red_band=True)).evidence
        self.assertTrue(evidence.red_marker_detected)
        self.assertIsNotNone(evidence.red_marker_y)
        self.assertGreater(evidence.red_marker_span or 0, 100)
        self.assertTrue(evidence.junction_detected)

    def test_fast_red_control_keeps_the_green_course_gate(self):
        layers = self.analyzer.detect_red_bands_fast(green_i_frame(transverse=True, red_band=True))
        self.assertEqual(len(layers), 1)
        self.assertGreater(layers[0].span, 100)

    def test_perspective_skewed_red_fragments_stay_one_physical_layer(self):
        image = green_i_frame(transverse=True)
        # The two pieces are one physical near band, but fisheye projection
        # shifts their centroids vertically by 35 px.  They must not become
        # two separate layers.
        cv2.rectangle(image, (180, 290), (305, 310), (0, 0, 255), -1)
        cv2.rectangle(image, (335, 325), (460, 345), (0, 0, 255), -1)
        evidence = self.analyzer.analyze(image).evidence
        self.assertTrue(evidence.red_marker_detected)
        self.assertEqual(len(self.analyzer.red_band_layers), 1)
        self.assertEqual(self.analyzer.red_band_layers[0].fragment_count, 2)
        self.assertGreaterEqual(self.analyzer.red_band_layers[0].y_spread, 30)

    def test_two_red_layers_remain_distinct_when_their_fragments_are_skewed(self):
        image = green_i_frame(transverse=True)
        # Far calibration band: one intact strip.  Near warning band: two
        # skewed fragments.  Their inter-layer gap exceeds the perspective
        # grouping tolerance.
        cv2.rectangle(image, (190, 205), (450, 225), (0, 0, 255), -1)
        cv2.rectangle(image, (180, 330), (305, 350), (0, 0, 255), -1)
        cv2.rectangle(image, (335, 365), (460, 385), (0, 0, 255), -1)
        self.analyzer.analyze(image)
        self.assertEqual(len(self.analyzer.red_band_layers), 2)
        self.assertEqual(self.analyzer.red_band_layers[0].fragment_count, 1)
        self.assertEqual(self.analyzer.red_band_layers[1].fragment_count, 2)

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

    def test_saturated_red_is_not_mistaken_for_green_course(self):
        frame = np.full((480, 640, 3), (0, 0, 255), dtype=np.uint8)
        cv2.line(frame, (320, 479), (320, 80), (245, 245, 245), 20)
        evidence = self.analyzer.analyze(frame).evidence
        self.assertTrue(evidence.line_lost)

    def test_red_excess_does_not_turn_bright_white_tape_into_a_red_marker(self):
        self.analyzer.analyze(green_i_frame())
        self.assertEqual(int(np.count_nonzero(self.analyzer.red_marker_mask)), 0)

    def test_permissive_tape_fit_is_available_without_control_authority(self):
        result = self.analyzer.analyze(green_i_frame())
        self.assertTrue(result.evidence.valid_line)
        self.assertIsNotNone(self.analyzer.tape_fit_line)

    def test_pale_floor_patch_touching_green_on_one_side_is_rejected(self):
        image = np.full((480, 640, 3), (55, 150, 45), dtype=np.uint8)
        # Broad bright floor touches the green course only along its lower
        # edge.  It passes the permissive HSV candidate mask but does not
        # have a green-supported tape backbone.
        cv2.rectangle(image, (120, 100), (639, 385), (215, 215, 215), -1)
        result = self.analyzer.analyze(image)
        self.assertTrue(result.evidence.line_lost)
        self.assertEqual(int(np.count_nonzero(result.component_mask)), 0)

    def test_green_field_fallback_keeps_close_tape_when_raw_side_probes_are_occluded(self):
        image = green_i_frame()
        # Simulate close-range glare/marker contamination right beside the
        # tape.  It removes the raw HSV-green side probes, while the recovered
        # connected green course correctly fills these thin carpet blemishes.
        cv2.rectangle(image, (299, 180), (303, 479), (80, 80, 80), -1)
        cv2.rectangle(image, (337, 180), (341, 479), (80, 80, 80), -1)
        # Set an intentionally impossible raw-proof ratio so the selection
        # path must exercise the recovered-course fallback, not merely the
        # ordinary raw-green proof.
        analyzer = GreenWhiteHybridScanlineAnalyzer(
            GreenWhiteScanlineConfig(route_path_update_frames=1, green_backbone_min_supported_ratio=1.01)
        )
        result = analyzer.analyze(image)
        self.assertTrue(result.evidence.valid_line)
        self.assertGreater(int(np.count_nonzero(result.component_mask)), 0)
        self.assertFalse(analyzer._green_backbone_supported(result.component_mask)[0])
        self.assertTrue(analyzer._course_backbone_supported(result.component_mask)[0])

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
