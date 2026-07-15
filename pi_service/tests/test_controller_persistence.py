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


if __name__ == "__main__":
    unittest.main()
