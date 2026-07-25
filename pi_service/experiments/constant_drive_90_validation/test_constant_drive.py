"""Pure checks for the isolated constant-drive test."""

import unittest

from constant_drive_runner import validate


class ConstantDriveValidationTests(unittest.TestCase):
    def test_accepts_ninety_pwm(self) -> None:
        validate(90, 10.0)

    def test_rejects_unsafe_pwm(self) -> None:
        with self.assertRaises(ValueError):
            validate(0, 10.0)
        with self.assertRaises(ValueError):
            validate(256, 10.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
