import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

from control.card_control import CardControlMixin
from control.drive_config import Config, normalize_profiles
from control.motor_commands import raw_motor_output, speed_targets
from control.protocol import parse_protocol_line


class ControlModuleTests(unittest.TestCase):
    def test_profiles_are_clamped_without_hardware(self) -> None:
        profiles = normalize_profiles({"F": {"rf": 999, "lf": -999}})
        self.assertEqual(profiles["F"]["rf"], 255)
        self.assertEqual(profiles["F"]["lf"], -255)

    def test_motor_commands_never_assign_card_outputs(self) -> None:
        config = Config()
        self.assertEqual(raw_motor_output("F", config)[2:], (0, 0))
        self.assertEqual(speed_targets("PL", config)[:2], (-30.0, 30.0))

    def test_protocol_parser_is_pure_and_preserves_card_outputs_on_stop(self) -> None:
        self.assertEqual(
            parse_protocol_line("OUT,10,-20,30,-40", None)["motor_output"],
            [10, -20, 30, -40],
        )
        self.assertEqual(
            parse_protocol_line("STATUS:STOPPED", [10, -20, 30, -40])["motor_output"],
            [0, 0, 30, -40],
        )

    def test_card_parameters_validate_without_serial(self) -> None:
        self.assertEqual(CardControlMixin._timed_motor_parameters(-180, 2500), (-180, 2500))
        with self.assertRaises(ValueError):
            CardControlMixin._timed_motor_parameters(0, 2500)


if __name__ == "__main__":
    unittest.main()
