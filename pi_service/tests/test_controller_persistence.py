import sys
import tempfile
import time
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
            self.assertEqual(controller.config.servo_speed_dps, 45.0)
            self.assertEqual(controller.config.servo_acceleration_dps2, 120.0)
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
                "servo_acceleration_dps2": 88,
                "servo_qe_reversed": True,
            })

            second = RobotController("unused", config_path)
            self.assertEqual(second.config.profiles["F"], {"rf": 123, "lf": 117, "lr": 117, "rr": 123})
            self.assertTrue(second.config.speed_mode)
            self.assertEqual(second.config.target_speed, 42.0)
            self.assertEqual(second.config.profiles["SF"], {"rf": 100, "lf": 100, "lr": 100, "rr": 100})
            self.assertEqual(second.config.servo_center_angle, 88)
            self.assertEqual(second.config.servo_speed_dps, 12.0)
            self.assertEqual(second.config.servo_acceleration_dps2, 88.0)
            self.assertTrue(second.config.servo_qe_reversed)

    def test_wasd_drive_and_qe_do_not_select_motor_actions(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            self.assertEqual(controller.update_keys({"keys": ["slow"]}), "SF")
            self.assertEqual(controller.update_keys({"keys": ["r"]}), "STOP")
            self.assertEqual(controller.config.profiles["SF"], {"rf": 100, "lf": 100, "lr": 100, "rr": 100})
            self.assertEqual(controller.update_keys({"keys": ["w", "a"]}), "FL")
            self.assertEqual(controller.update_keys({"keys": ["w", "d"]}), "FR")
            self.assertEqual(controller.update_keys({"keys": ["x"]}), "SPL")
            self.assertEqual(controller.update_keys({"keys": ["c"]}), "SPR")
            self.assertEqual(controller.update_keys({"keys": [], "steering": -1}), "STOP")
            self.assertEqual(controller.steering_direction, -1)

    def test_drive_uses_m1_m2_and_card_motors_accept_power_and_time(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            self.assertEqual(controller._raw("F", controller.config), (255, 255, 0, 0))
            commands: list[str] = []
            def write_with_ack(command: str) -> bool:
                commands.append(command)
                if command.startswith("DEAL"): controller._parse("OK:DEAL,180,2500")
                if command.startswith("FEED"): controller._parse("OK:FEED,-200,5000")
                return True
            controller._write = write_with_ack
            self.assertEqual(controller.deal_card(180, 2500), "running")
            self.assertEqual(controller.feed_cards(-200, 5000), "running")
            self.assertEqual(commands, ["DEAL,180,2500", "FEED,-200,5000"])
            controller._parse("OK:DEAL,180,2500")
            self.assertEqual(controller.card_deal_state, "running")
            controller._parse("DEAL:DONE")
            self.assertEqual(controller.card_deal_state, "idle")
            controller._parse("OK:FEED,-200,5000")
            self.assertEqual(controller.card_feed_state, "running")
            controller._parse("FEED:DONE")
            self.assertEqual(controller.card_feed_state, "idle")

    def test_card_motor_power_and_time_are_bounded(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            for pwm, duration in [(0, 1000), (-256, 1000), (256, 1000), (100, 99), (100, 60001)]:
                with self.assertRaises(ValueError):
                    controller.deal_card(pwm, duration)

    def test_legacy_firmware_falls_back_to_plain_deal_command(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            commands: list[str] = []
            def write_legacy_ack(command: str) -> bool:
                commands.append(command)
                controller._parse("OK:DEAL")
                return True
            controller._write = write_legacy_ack
            controller._parse("READY:MOTOR_BRIDGE,M1=RIGHT,M2=LEFT,M3=CARD_CONTINUOUS,M4=DEAL_1000MS,SERVO=23,IMU=OK")
            self.assertEqual(controller.card_motor_protocol, "legacy")
            self.assertEqual(controller.deal_card(180, 2500), "legacy")
            self.assertEqual(commands, ["DEAL"])
            with self.assertRaises(RuntimeError):
                controller.feed_cards(200, 5000)

    def test_adjustable_firmware_is_detected_from_ready_line(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            controller._parse("READY:MOTOR_BRIDGE,M1=RIGHT,M2=LEFT,M3=FEED_ADJUSTABLE,M4=DEAL_ADJUSTABLE,SERVO=23,IMU=OK")
            self.assertEqual(controller.card_motor_protocol, "adjustable")

    def test_unknown_firmware_retries_plain_deal_and_deduplicates_key_token(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            commands: list[str] = []
            def write_with_fallback(command: str) -> bool:
                commands.append(command)
                controller._parse("ERR:BAD_COMMAND" if "," in command else "OK:DEAL")
                return True
            controller._write = write_with_fallback
            request = {
                "token": "p-1",
                "feed_pwm": 200,
                "feed_duration_ms": 5000,
                "deal_pwm": 200,
                "deal_duration_ms": 1200,
            }
            first = controller.deal_from_key_request(request)
            deadline = time.monotonic() + 1.0
            while first["state"] == "pending" and time.monotonic() < deadline:
                time.sleep(.01)
                first = controller.deal_from_key_request(request)
            second = controller.deal_from_key_request(request)
            self.assertEqual(first, second)
            self.assertEqual(first["state"], "partial")
            self.assertEqual(first["feed"]["state"], "error")
            self.assertEqual(first["deal"]["state"], "legacy")
            self.assertEqual(commands, ["FEED,200,5000", "DEAL,200,1200", "DEAL"])
            self.assertEqual(controller.card_motor_protocol, "legacy")

    def test_p_request_runs_m3_and_m4_with_independent_presets(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            controller._parse("READY:MOTOR_BRIDGE,M1=RIGHT,M2=LEFT,M3=FEED_ADJUSTABLE,M4=DEAL_ADJUSTABLE,SERVO=23,IMU=OK")
            commands: list[str] = []
            def write_both_acks(command: str) -> bool:
                commands.append(command)
                controller._parse("OK:FEED,-180,2500" if command.startswith("FEED") else "OK:DEAL,-210,900")
                return True
            controller._write = write_both_acks
            request = {
                "token": "p-both",
                "feed_pwm": -180,
                "feed_duration_ms": 2500,
                "deal_pwm": -210,
                "deal_duration_ms": 900,
            }
            result = controller.deal_from_key_request(request)
            deadline = time.monotonic() + 1.0
            while result["state"] == "pending" and time.monotonic() < deadline:
                time.sleep(.01)
                result = controller.deal_from_key_request(request)
            self.assertEqual(result["state"], "running")
            self.assertEqual(result["feed"]["reply"], "OK:FEED,-180,2500")
            self.assertEqual(result["deal"]["reply"], "OK:DEAL,-210,900")
            self.assertEqual(commands, ["FEED,-180,2500", "DEAL,-210,900"])

    def test_steering_syncs_direction_and_limits_to_arduino(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            commands: list[str] = []
            controller._write = commands.append
            controller.update_keys({"keys": [], "steering": 1})
            controller.last_steering_seen = 10.0
            controller._sync_steering(10.1, controller.config)
            self.assertEqual(commands, ["SVC,45.0,120.0", "SVD,-1"])
            commands.clear()
            controller._sync_steering(11.0, controller.config)
            self.assertEqual(commands, ["SVD,0"])

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
            controller._parse("SVP,124")
            controller._parse("OK:M,120,-80,-75,110")
            controller._parse("OUT,121,-81,-76,111")
            self.assertEqual(controller.ultrasonic, 31.5)
            self.assertEqual(controller.servo_angle, 124)
            self.assertEqual(controller.motor_output, [121, -81, -76, 111])

    def test_servo_command_is_validated_without_touching_drive_profiles(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            controller = RobotController("unused", Path(directory) / "drive_config.json")
            commands: list[str] = []
            controller._write = commands.append
            self.assertEqual(controller.set_servo_angle(42), 42)
            self.assertEqual(commands, ["SV,42"])
            self.assertEqual(controller.servo_angle, 42)
            self.assertEqual(controller.set_servo_angle(90, fast=True), 90)
            self.assertEqual(commands, ["SV,42", "SVF,90"])
            with self.assertRaises(ValueError): controller.set_servo_angle(181)


if __name__ == "__main__":
    unittest.main()
