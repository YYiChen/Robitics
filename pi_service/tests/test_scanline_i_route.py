import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

from scanline_i_route import (  # noqa: E402
    ScanlineIRouteConfig,
    ScanlineIShapeRouteTracker,
    load_scanline_tuning_config,
)
from scanline_i_logic import TurnaroundState  # noqa: E402


class _Gate:
    def enabled(self):
        return False


class ScanlineIRouteTests(unittest.TestCase):
    def test_web_tuning_persists_i_shape_speeds_only(self):
        with tempfile.TemporaryDirectory() as directory:
            tuning_path = Path(directory) / "scanline_web_tuning.json"
            tracker = ScanlineIShapeRouteTracker(None, None, None, _Gate(), ScanlineIRouteConfig(tuning_path=tuning_path))
            status = tracker.update_tuning({"straight_pwm": 135, "pivot_pwm": 210, "correction_gain": 140, "pivot_min_seconds": 3.5, "pivot_max_seconds": 6.0})
            self.assertEqual(status["tuning"]["straight_pwm"], 135)
            self.assertEqual(status["tuning"]["pivot_pwm"], 210)
            self.assertEqual(json.loads(tuning_path.read_text(encoding="utf-8"))["correction_gain"], 140)
            reloaded = load_scanline_tuning_config(tuning_path)
            self.assertEqual(reloaded.straight_pwm, 135)
            self.assertEqual(reloaded.pivot_pwm, 210)
            self.assertEqual(reloaded.pivot_min_seconds, 3.5)

    def test_rejects_pivot_minimum_longer_than_safety_timeout(self):
        tracker = ScanlineIShapeRouteTracker(None, None, None, _Gate())
        with self.assertRaisesRegex(ValueError, "最短掉头时间"):
            tracker.update_tuning({"pivot_min_seconds": 6.0, "pivot_max_seconds": 5.0})

    def test_straight_following_uses_differential_pwm_for_off_center_line(self):
        evidence = SimpleNamespace(line_center_x=400.0, line_centers=((1, 400.0, 20),) * 3)
        right, left = ScanlineIShapeRouteTracker._straight_pair(evidence, 640, ScanlineIRouteConfig(straight_pwm=120))
        self.assertLess(right, left)
        self.assertEqual(right + left, 240)

    def test_early_bar_prediction_keeps_line_following_active(self):
        self.assertTrue(ScanlineIShapeRouteTracker._keeps_forward_motion(TurnaroundState.EARLY_BAR_PREDICTED))
        self.assertFalse(ScanlineIShapeRouteTracker._keeps_forward_motion(TurnaroundState.BRAKE_BEFORE_PIVOT))
