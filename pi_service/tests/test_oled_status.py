import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

from oled_status import OledStatusService


class FakeDisplay:
    address, i2c_port, error, online = 0x3C, 1, "", True

    def __init__(self) -> None:
        self.lines: list[str] | None = None
        self.warning = False

    def begin(self) -> bool:
        return True

    def show_text(self, lines: list[str]) -> None:
        self.lines = lines

    def show_warning(self) -> None:
        self.warning = True

    def stop(self) -> None:
        self.online = False


class OledStatusTests(unittest.TestCase):
    def test_renders_status_without_real_i2c_hardware(self) -> None:
        class Controller:
            @staticmethod
            def status(): return {"arduino_online": True, "ultrasonic": [-1.0, 18.2, -1.0], "action": "F"}

        class Camera:
            @staticmethod
            def status_dict(): return {"online": True}

        display = FakeDisplay()
        service = OledStatusService(Controller(), Camera(), interval_seconds=0.2, display=display)
        service.start()
        time.sleep(0.03)
        status = service.status_dict()
        service.stop()
        self.assertTrue(status["online"])
        self.assertEqual(display.lines, ["ROBITICS", "A:OK C:OK", "F:18cm", "F"])

    def test_renders_warning_when_a_dependency_is_offline(self) -> None:
        class Controller:
            @staticmethod
            def status(): return {"arduino_online": False, "ultrasonic": None, "action": "STOP"}

        class Camera:
            @staticmethod
            def status_dict(): return {"online": True}

        display = FakeDisplay()
        service = OledStatusService(Controller(), Camera(), interval_seconds=0.2, display=display)
        service.start()
        time.sleep(0.03)
        service.stop()
        self.assertTrue(display.warning)


if __name__ == "__main__":
    unittest.main()
