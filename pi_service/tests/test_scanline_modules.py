import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

from routes.scanline.config import ScanlineIRouteConfig, load_scanline_tuning_config
from routes.scanline.control import StraightMotorConfig, drive_pwm_for_offset
from routes.scanline.logging import RUN_LOG_SCHEMA_VERSION


class ScanlineModuleTests(unittest.TestCase):
    def test_config_loader_isolated_from_tracker(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.json"
            path.write_text('{"straight_pwm": 133, "pivot_pwm": 999}', encoding="utf-8")
            config = load_scanline_tuning_config(path)
            self.assertEqual(config.straight_pwm, 133)
            self.assertEqual(config.pivot_pwm, ScanlineIRouteConfig().pivot_pwm)

    def test_pure_control_maps_positive_offset_to_faster_left_wheel(self) -> None:
        right, left = drive_pwm_for_offset(
            SimpleNamespace(offset=.5, valid_bands=3),
            StraightMotorConfig(straight_pwm=100, correction_gain=100),
        )
        self.assertLess(right, left)

    def test_log_schema_is_explicit(self) -> None:
        self.assertEqual(RUN_LOG_SCHEMA_VERSION, 1)


if __name__ == "__main__":
    unittest.main()
