import sys
import tempfile
import types
import unittest
from pathlib import Path

# The controller only needs pyserial when it opens the Arduino port.  Keep the
# persistence test runnable on a development PC without installing pyserial.
sys.modules.setdefault("serial", types.SimpleNamespace(Serial=None))
sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

from controller import RobotController


class ControllerPersistenceTests(unittest.TestCase):
    def test_profiles_and_pid_survive_new_controller(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "robot_config.json"
            first = RobotController("unused", config_path)
            first.update_config({
                "profiles": {"F": {"rf": 123, "lf": 117, "lr": 117, "rr": 123}},
                "speed_mode": True,
                "target_speed": 42,
            })

            second = RobotController("unused", config_path)
            self.assertEqual(second.config.profiles["F"], {"rf": 123, "lf": 117, "lr": 117, "rr": 123})
            self.assertTrue(second.config.speed_mode)
            self.assertEqual(second.config.target_speed, 42.0)

    def test_shutdown_flushes_latest_in_memory_config(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "robot_config.json"
            controller = RobotController("unused", config_path)
            controller.config.profiles["PR"]["rf"] = 211
            controller.stop()

            reloaded = RobotController("unused", config_path)
            self.assertEqual(reloaded.config.profiles["PR"]["rf"], 211)

    def test_legacy_robot_config_is_copied_to_drive_config_once(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            legacy_path = Path(directory) / "robot_config.json"
            drive_path = Path(directory) / "drive_config.json"
            legacy_path.write_text('{"target_speed": 57, "profiles": {"F": {"rf": 101, "lf": 102, "lr": 103, "rr": 104}}}', encoding="utf-8")

            controller = RobotController("unused", drive_path, legacy_path)
            self.assertEqual(controller.config.target_speed, 57.0)
            self.assertEqual(controller.config.profiles["F"], {"rf": 101, "lf": 102, "lr": 103, "rr": 104})
            self.assertTrue(drive_path.exists())
            self.assertIn('"target_speed": 57.0', drive_path.read_text(encoding="utf-8"))
            self.assertIn('"target_speed": 57', legacy_path.read_text(encoding="utf-8"))

    def test_parses_one_front_sensor_and_servo_reply(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            controller._parse("US,31.5")
            controller._parse("OK:SV,125")
            self.assertEqual(controller.ultrasonic, 31.5)
            self.assertEqual(controller.servo_angle, 125)

    def test_servo_command_is_validated_without_touching_drive_profiles(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            commands: list[str] = []
            controller._write = commands.append
            self.assertEqual(controller.set_servo_angle(42), 42)
            self.assertEqual(commands, ["SV,42"])
            self.assertEqual(controller.servo_angle, 42)
            with self.assertRaises(ValueError): controller.set_servo_angle(181)


if __name__ == "__main__":
    unittest.main()
