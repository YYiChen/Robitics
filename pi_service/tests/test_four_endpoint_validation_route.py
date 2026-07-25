from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[2]
ROBOT_WEB = ROOT / "pi_service" / "robot_web"
FOUR_ENDPOINT = ROOT / "pi_service" / "experiments" / "i_shape_four_endpoint_navigation_validation"
for location in (str(ROBOT_WEB), str(FOUR_ENDPOINT)):
    if location not in sys.path:
        sys.path.insert(0, location)

from four_endpoint_planner import DriveAction  # noqa: E402
from four_endpoint_validation_route import FourEndpointValidationRouteTracker, JunctionPassGate  # noqa: E402
from scanline_i_route import ScanlineIRouteConfig  # noqa: E402


class FourEndpointValidationRouteTests(unittest.TestCase):
    def test_pivot_signs_match_right_m1_left_m2_mapping(self):
        config = ScanlineIRouteConfig(pivot_pwm=200)
        evidence = SimpleNamespace(line_center_x=320.0, line_centers=((440, 320.0, 20),))
        self.assertEqual(
            FourEndpointValidationRouteTracker._motor_pair(DriveAction.PIVOT_LEFT_90, evidence, 640, config),
            (200, -200),
        )
        self.assertEqual(
            FourEndpointValidationRouteTracker._motor_pair(DriveAction.PIVOT_RIGHT_90, evidence, 640, config),
            (-200, 200),
        )

    def test_visual_observation_requires_confident_bottom_line(self):
        evidence = SimpleNamespace(junction_detected=True, valid_line=True, confidence=.6, line_center_x=320.0)
        self.assertTrue(FourEndpointValidationRouteTracker._forward_line_detected(evidence))
        self.assertFalse(FourEndpointValidationRouteTracker._forward_line_detected(SimpleNamespace(junction_detected=False, valid_line=True, confidence=.5, line_center_x=320.0)))

    def test_early_junction_cannot_trigger_a_pivot_before_the_red_band_exits(self):
        gate = JunctionPassGate()
        def evidence(*, bar=False, red_y=None, lost=False):
            return SimpleNamespace(
                endpoint_detected=bar, red_marker_detected=red_y is not None,
                red_marker_y=red_y, frame_height=480, line_lost=lost,
            )
        self.assertFalse(gate.observe(evidence(bar=True, red_y=280)))
        self.assertFalse(gate.observe(evidence(bar=True, red_y=420)))
        self.assertTrue(gate.observe(evidence(bar=True, red_y=None)))

    def test_unmarked_course_falls_back_to_stem_loss_only_after_bar_confirmation(self):
        gate = JunctionPassGate(line_lost_confirm_frames=2)
        no_bar = SimpleNamespace(endpoint_detected=False, red_marker_detected=False, red_marker_y=None, frame_height=480, line_lost=True)
        bar = SimpleNamespace(endpoint_detected=True, red_marker_detected=False, red_marker_y=None, frame_height=480, line_lost=False)
        lost = SimpleNamespace(endpoint_detected=False, red_marker_detected=False, red_marker_y=None, frame_height=480, line_lost=True)
        self.assertFalse(gate.observe(no_bar))
        self.assertFalse(gate.observe(bar))
        self.assertFalse(gate.observe(lost))
        self.assertTrue(gate.observe(lost))


if __name__ == "__main__":
    unittest.main()
