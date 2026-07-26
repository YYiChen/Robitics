import unittest

from pi_service.verify_paths import validate_paths


class ProjectPathTests(unittest.TestCase):
    def test_formal_module_and_launcher_paths(self) -> None:
        self.assertEqual(validate_paths(), [])


if __name__ == "__main__":
    unittest.main()
