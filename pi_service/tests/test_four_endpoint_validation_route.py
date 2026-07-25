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
from four_endpoint_validation_route import FourEndpointValidationRouteTracker  # noqa: E402
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
        observation = FourEndpointValidationRouteTracker._vision_observation(evidence)
        self.assertTrue(observation.junction_detected)
        self.assertTrue(observation.forward_line_detected)
        self.assertFalse(FourEndpointValidationRouteTracker._vision_observation(SimpleNamespace(junction_detected=False, valid_line=True, confidence=.5, line_center_x=320.0)).forward_line_detected)


if __name__ == "__main__":
    unittest.main()
