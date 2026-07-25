import unittest

from runtime_guard import require_no_competing_autonomous_route


def status(*, online=True, available=False, enabled=False):
    return {"robot": {"arduino_online": online}, "autonomous": {"available": available, "enabled": enabled}}


class RuntimeGuardTests(unittest.TestCase):
    def test_accepts_plain_robot_web_service(self):
        self.assertTrue(require_no_competing_autonomous_route(status())["robot"]["arduino_online"])


    def test_rejects_offline_arduino_before_any_motor_command(self):
        with self.assertRaisesRegex(RuntimeError, "Arduino is not online"):
            require_no_competing_autonomous_route(status(online=False))


    def test_rejects_even_paused_in_process_autonomous_route(self):
        with self.assertRaisesRegex(RuntimeError, "competing autonomous route service"):
            require_no_competing_autonomous_route(status(available=True, enabled=False))
