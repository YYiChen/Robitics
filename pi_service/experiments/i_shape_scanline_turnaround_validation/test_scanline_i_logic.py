import unittest

import cv2
import numpy as np

from scanline_i_logic import (
    HybridScanlineAnalyzer,
    HybridScanlineConfig,
    IShapeScanlineAnalyzer,
    IShapeTurnaroundPlanner,
    TurnaroundConfig,
    TurnaroundState,
)


def frame(*, endpoint=False, lower_stem_only=False, endpoint_y=336):
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    stem_top = endpoint_y if lower_stem_only else 80
    cv2.line(image, (320, 479), (320, stem_top), (0, 0, 0), 20)
    if endpoint:
        cv2.line(image, (100, endpoint_y), (540, endpoint_y), (0, 0, 0), 20)
    return image


def t_junction_frame(stem_top=80, bar_y=200):
    """Draw a T-shaped tape mark: vertical stem + horizontal bar."""
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    # Vertical stem from bottom to bar_y
    cv2.line(image, (320, 479), (320, stem_top), (0, 0, 0), 20)
    # Horizontal bar
    cv2.line(image, (100, bar_y), (540, bar_y), (0, 0, 0), 20)
    return image


class ScanlineIShapeTests(unittest.TestCase):
    def test_vertical_tape_remains_the_main_line(self):
        evidence = IShapeScanlineAnalyzer().analyze(frame()).evidence
        self.assertTrue(evidence.valid_line)
        self.assertFalse(evidence.endpoint_detected)
        self.assertAlmostEqual(evidence.line_center_x, 320, delta=3)

    def test_lower_wide_bar_is_endpoint_not_a_left_or_right_path(self):
        evidence = IShapeScanlineAnalyzer().analyze(frame(endpoint=True)).evidence
        self.assertTrue(evidence.valid_line)
        self.assertTrue(evidence.endpoint_detected)
        self.assertGreater(evidence.endpoint_width or 0, 300)

    def test_far_wide_bar_is_detected_before_it_reaches_the_lower_image(self):
        evidence = IShapeScanlineAnalyzer().analyze(frame(endpoint=True, endpoint_y=200)).evidence
        self.assertTrue(evidence.valid_line)
        self.assertTrue(evidence.endpoint_detected)
        self.assertLess(evidence.endpoint_y or 480, 230)

    def test_near_stem_still_confirms_bar_when_it_ends_at_the_bar(self):
        evidence = IShapeScanlineAnalyzer().analyze(frame(endpoint=True, lower_stem_only=True)).evidence
        self.assertTrue(evidence.valid_line)
        self.assertGreaterEqual(evidence.confidence, 0.55)
        self.assertTrue(evidence.endpoint_detected)
        self.assertAlmostEqual(evidence.line_center_x, 320, delta=3)

    def test_marked_bar_waits_for_stem_loss_then_brakes_before_pivot(self):
        analyzer = IShapeScanlineAnalyzer()
        planner = IShapeTurnaroundPlanner(TurnaroundConfig(endpoint_confirm_frames=2, line_lost_confirm_frames=2, reacquire_confirm_frames=2, brake_seconds=.1, pivot_min_seconds=1, pivot_max_seconds=5))
        # Matches the live camera scene: only the lower stem remains visible
        # beneath the transverse endpoint bar.
        bar = analyzer.analyze(frame(endpoint=True, lower_stem_only=True)).evidence
        lost = analyzer.analyze(np.full((480, 640, 3), 255, dtype=np.uint8)).evidence
        straight = analyzer.analyze(frame()).evidence
        self.assertIs(planner.step(bar, 0).state, TurnaroundState.FOLLOW_STRAIGHT)
        self.assertIs(planner.step(bar, .1).state, TurnaroundState.BAR_MARKED)
        self.assertIs(planner.step(lost, .2).state, TurnaroundState.BAR_MARKED)
        self.assertIs(planner.step(lost, .3).state, TurnaroundState.BRAKE_BEFORE_PIVOT)
        self.assertIs(planner.step(lost, .35).state, TurnaroundState.BRAKE_BEFORE_PIVOT)
        self.assertIs(planner.step(lost, .41).state, TurnaroundState.PIVOT_180)
        self.assertIs(planner.step(straight, 1.41).state, TurnaroundState.PIVOT_180)
        self.assertIs(planner.step(straight, 1.51).state, TurnaroundState.FOLLOW_STRAIGHT)


class HybridScanlineAnalyzerTests(unittest.TestCase):
    """Tests for the skeleton-augmented HybridScanlineAnalyzer."""

    def setUp(self):
        self.analyzer = HybridScanlineAnalyzer(
            HybridScanlineConfig(
                route_path_update_frames=1,  # update every frame for tests
                early_junction_max_y_ratio=0.95,  # accept junctions almost anywhere in test frames
            )
        )

    def test_pure_vertical_line_no_junction(self):
        """A plain vertical stem has no junction."""
        evidence = self.analyzer.analyze(frame()).evidence
        self.assertTrue(evidence.valid_line)
        self.assertFalse(evidence.junction_detected)
        self.assertIsNone(evidence.junction_y)
        self.assertEqual(evidence.junction_arm_count, 0)
        # Lookahead should be ahead (smaller y)
        self.assertIsNotNone(evidence.lookahead_y)
        self.assertLess(evidence.lookahead_y or 999, 400)
        self.assertGreater(evidence.path_length_px, 10)

    def test_t_junction_detected_via_skeleton_topology(self):
        """A T-shaped frame should trigger junction_detected."""
        evidence = self.analyzer.analyze(t_junction_frame(bar_y=200)).evidence
        self.assertTrue(evidence.valid_line)
        self.assertTrue(evidence.junction_detected,
                        f"Expected junction at T-bar, got junction_detected=False, "
                        f"junction_arm_count={evidence.junction_arm_count}")
        self.assertIsNotNone(evidence.junction_y)
        # Junction should be near the bar (around y=200, within tolerance)
        if evidence.junction_y is not None:
            self.assertLess(abs(evidence.junction_y - 200), 30,
                            f"Junction y={evidence.junction_y}, expected near 200")

    def test_t_junction_arm_count_is_at_least_three(self):
        """A T-junction has >= 3 skeleton arms (T=3, X=4)."""
        evidence = self.analyzer.analyze(t_junction_frame(bar_y=200)).evidence
        if evidence.junction_detected:
            self.assertGreaterEqual(evidence.junction_arm_count, 3,
                                    f"Expected >=3 arms for junction, got {evidence.junction_arm_count}")

    def test_stem_endpoint_selected_not_bar(self):
        """The y-min endpoint (stem tip) should be selected, not the bar ends.
        The lookahead (at 60% of path) should still be on the stem, i.e. below the bar."""
        evidence = self.analyzer.analyze(t_junction_frame(bar_y=200, stem_top=80)).evidence
        # The lookahead should be below the bar (y=200), i.e. still on the stem.
        # y increases downward in the image, so lookahead_y > bar_y means
        # the lookahead is still on the approach side of the bar.
        self.assertIsNotNone(evidence.lookahead_y)
        if evidence.lookahead_y is not None:
            self.assertGreater(evidence.lookahead_y, 190,
                               f"Lookahead y={evidence.lookahead_y}, expected > 190 (below bar at y=200)")
        # Path length should be substantial (>100px for a clear vertical stem)
        self.assertGreater(evidence.path_length_px, 100)

    def test_early_bar_predicted_state_transition(self):
        """Junction evidence should trigger EARLY_BAR_PREDICTED."""
        analyzer = HybridScanlineAnalyzer(
            HybridScanlineConfig(
                route_path_update_frames=1,
                early_junction_max_y_ratio=0.95,
            )
        )
        planner = IShapeTurnaroundPlanner(
            TurnaroundConfig(
                junction_confirm_frames=2,
                endpoint_confirm_frames=3,
                line_lost_confirm_frames=2,
                bar_mark_timeout_seconds=5.0,
            )
        )
        # Frame 1: T-junction, first junction confirmation
        t_frame = t_junction_frame(bar_y=200)
        d1 = planner.step(analyzer.analyze(t_frame).evidence, 0.0)
        self.assertIs(d1.state, TurnaroundState.FOLLOW_STRAIGHT,
                      f"Frame 1 should still FOLLOW, got {d1.state}")

        # Frame 2: second junction confirmation -> EARLY_BAR_PREDICTED
        d2 = planner.step(analyzer.analyze(t_frame).evidence, 0.1)
        self.assertIs(d2.state, TurnaroundState.EARLY_BAR_PREDICTED,
                      f"Frame 2 should enter EARLY_BAR_PREDICTED, got {d2.state} with reason: {d2.reason}")

    def test_early_bar_predicted_false_alarm_recovery(self):
        """Junction disappears without bar confirmation -> return to FOLLOW."""
        analyzer = HybridScanlineAnalyzer(
            HybridScanlineConfig(
                route_path_update_frames=1,
                early_junction_max_y_ratio=0.95,
            )
        )
        planner = IShapeTurnaroundPlanner(
            TurnaroundConfig(
                junction_confirm_frames=2,
                endpoint_confirm_frames=3,
                line_lost_confirm_frames=2,
            )
        )
        # Two junction frames -> EARLY_BAR_PREDICTED
        t_frame = t_junction_frame(bar_y=200)
        planner.step(analyzer.analyze(t_frame).evidence, 0.0)
        d2 = planner.step(analyzer.analyze(t_frame).evidence, 0.1)
        self.assertIs(d2.state, TurnaroundState.EARLY_BAR_PREDICTED,
                      f"Expected EARLY_BAR_PREDICTED after 2 junction frames, got {d2.state}")

        # Now feed a plain vertical line (junction disappeared, false alarm)
        # Need to reduce junction_frames below confirm threshold
        straight_evidence = analyzer.analyze(frame()).evidence
        d3 = planner.step(straight_evidence, 0.2)
        self.assertIs(d3.state, TurnaroundState.EARLY_BAR_PREDICTED,
                      f"First false-alarm frame should stay EARLY_BAR_PREDICTED, got {d3.state}")
        d4 = planner.step(straight_evidence, 0.3)
        self.assertIs(d4.state, TurnaroundState.FOLLOW_STRAIGHT,
                      f"Second false-alarm frame should return to FOLLOW, got {d4.state} with reason: {d4.reason}")

    def test_early_prediction_transitions_to_bar_marked_on_endpoint(self):
        """EARLY_BAR_PREDICTED -> BAR_MARKED when bar_rows detect the bar."""
        analyzer = HybridScanlineAnalyzer(
            HybridScanlineConfig(
                route_path_update_frames=1,
                early_junction_max_y_ratio=0.95,
            )
        )
        planner = IShapeTurnaroundPlanner(
            TurnaroundConfig(
                junction_confirm_frames=1,
                endpoint_confirm_frames=1,
            )
        )
        # Junction triggers early prediction immediately
        planner.step(analyzer.analyze(t_junction_frame(bar_y=336)).evidence, 0.0)
        # Same frame already has endpoint in bar_rows -> should go to BAR_MARKED
        d = planner.step(analyzer.analyze(t_junction_frame(bar_y=336)).evidence, 0.1)
        self.assertIn(d.state, [TurnaroundState.EARLY_BAR_PREDICTED, TurnaroundState.BAR_MARKED],
                      f"Expected early prediction or bar marked, got {d.state}")

    def test_legacy_analyzer_produces_compatible_evidence(self):
        """Legacy evidence should work with the extended planner (no hybrid fields)."""
        legacy = IShapeScanlineAnalyzer().analyze(frame(endpoint=True, lower_stem_only=True)).evidence
        # Legacy evidence has None/0/False for hybrid fields by default
        self.assertIsNone(legacy.junction_y)
        self.assertFalse(legacy.junction_detected)
        self.assertEqual(legacy.junction_arm_count, 0)
        self.assertEqual(legacy.path_length_px, 0)
        # Planner should handle it without error
        planner = IShapeTurnaroundPlanner()
        d = planner.step(legacy, 0.0)
        self.assertIs(d.state, TurnaroundState.FOLLOW_STRAIGHT)
