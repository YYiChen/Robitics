from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ROBOT_WEB = ROOT / "robot_web"
if str(ROBOT_WEB) not in sys.path:
    sys.path.insert(0, str(ROBOT_WEB))

from end_line_turn_adaptor import EndLineTurnAdaptorRouteTracker


class _Gate:
    def enabled(self): return False
    def toggle(self): return False


class EndLineAdaptorTuningTests(unittest.TestCase):
    def test_update_is_visible_immediate_and_persisted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.json"
            tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(), tuning_path=path)
            status = tracker.update_tuning({"straight_pwm": 91, "pivot_seconds": 2.8, "correction_gain": 190})
            self.assertEqual(status["tuning"]["straight_pwm"], 91)
            self.assertEqual(status["tuning"]["correction_gain"], 190.0)
            self.assertEqual(status["tuning"]["pivot_seconds"], 2.8)
            self.assertTrue(path.exists())
            reloaded = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(), tuning_path=path)
            self.assertEqual(reloaded.status_dict()["tuning"]["straight_pwm"], 91)

    def test_rejects_invalid_correction_range(self):
        with tempfile.TemporaryDirectory() as directory:
            tracker = EndLineTurnAdaptorRouteTracker(None, None, None, _Gate(), tuning_path=Path(directory) / "tuning.json")
            with self.assertRaises(ValueError):
                tracker.update_tuning({"minimum_correction_pwm": 200, "maximum_correction_pwm": 100})
