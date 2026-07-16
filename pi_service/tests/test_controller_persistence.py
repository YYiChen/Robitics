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
    def test_default_camera_gimbal_settings_match_driving_defaults(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            self.assertEqual(controller.config.servo_speed_dps, 30.0)
            self.assertTrue(controller.config.servo_qe_reversed)

    def test_profiles_and_pid_survive_new_controller(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "robot_config.json"
            first = RobotController("unused", config_path)
            first.update_config({
                "profiles": {"F": {"rf": 123, "lf": 117, "lr": 117, "rr": 123}},
                "speed_mode": True,
                "target_speed": 42,
                "servo_center_angle": 88,
                "servo_speed_dps": 12,
                "servo_qe_reversed": True,
            })

            second = RobotController("unused", config_path)
            self.assertEqual(second.config.profiles["F"], {"rf": 123, "lf": 117, "lr": 117, "rr": 123})
            self.assertTrue(second.config.speed_mode)
            self.assertEqual(second.config.target_speed, 42.0)
            self.assertEqual(second.config.profiles["SF"], {"rf": 100, "lf": 100, "lr": 100, "rr": 100})
            self.assertEqual(second.config.servo_center_angle, 88)
            self.assertEqual(second.config.servo_speed_dps, 12.0)
            self.assertTrue(second.config.servo_qe_reversed)

    def test_r_is_slow_forward_and_qe_do_not_select_motor_actions(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            self.assertEqual(controller.update_keys({"keys": ["r"]}), "SF")
            self.assertEqual(controller.config.profiles["SF"], {"rf": 100, "lf": 100, "lr": 100, "rr": 100})
            self.assertEqual(controller.update_keys({"keys": ["w", "a"]}), "FL")
            self.assertEqual(controller.update_keys({"keys": ["w", "d"]}), "FR")
            self.assertEqual(controller.update_keys({"keys": [], "steering": -1}), "STOP")
            self.assertEqual(controller.steering_direction, -1)

    def test_steering_uses_pi_time_and_holds_after_heartbeat_stops(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            commands: list[str] = []
            controller._write = commands.append
            controller.servo_angle = 90
            controller.update_keys({"keys": [], "steering": 1})
            controller._last_servo_motion_at = 10.0
            controller.last_steering_seen = 10.0
            controller._advance_steering(10.5, controller.config)
            self.assertEqual(commands, ["SV,75"])
            controller._advance_steering(11.0, controller.config)
            self.assertEqual(commands, ["SV,75"])

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

    def test_legacy_scalar_pwm_config_is_converted_to_drive_profiles(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "drive_config.json"
            config_path.write_text('{"straight_pwm": 121, "pivot_pwm": 141, "curve_outer_pwm": 201, "curve_inner_pwm": 71}', encoding="utf-8")
            controller = RobotController("unused", config_path)
            self.assertEqual(controller.config_source, "drive_config")
            self.assertEqual(controller.config.profiles["F"], {"rf": 121, "lf": 121, "lr": 121, "rr": 121})
            self.assertEqual(controller.config.profiles["FL"], {"rf": 201, "lf": 71, "lr": 71, "rr": 201})
            self.assertEqual(controller.config.profiles["FR"], {"rf": 71, "lf": 201, "lr": 201, "rr": 71})

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
