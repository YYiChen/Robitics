import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

from control.card_control import CardControlMixin
from control.drive_config import Config, default_profiles, normalize_profiles
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

    def test_direct_and_speed_modes_both_derive_actions_from_profiles(self) -> None:
        config = Config(target_speed=40)
        config.profiles = normalize_profiles({
            "F": {"rf": 100, "lf": 50},
            "PL": {"rf": 220, "lf": -110},
        })

        self.assertEqual(raw_motor_output("F", config), (100, 50, 0, 0))
        self.assertEqual(speed_targets("F", config), (20.0, 40.0, 0, 0))
        self.assertEqual(speed_targets("PL", config), (-20.0, 40.0, 0, 0))

    def test_tracked_drive_defaults_match_code_profile_defaults(self) -> None:
        import json

        defaults_path = Path(__file__).parents[1] / "config" / "defaults.json"
        tracked = json.loads(defaults_path.read_text(encoding="utf-8"))["drive"]
        self.assertNotIn("straight_pwm", tracked)
        self.assertNotIn("pivot_pwm", tracked)
        self.assertEqual(normalize_profiles(tracked["profiles"]), default_profiles())

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
