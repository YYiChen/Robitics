import unittest
from pathlib import Path


class RouteTuningUiContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = (
            Path(__file__).parents[1] / "robot_web" / "static" / "app.js"
        ).read_text(encoding="utf-8")

    def test_status_polling_does_not_overwrite_unsaved_route_tuning(self) -> None:
        self.assertIn("const routeTuningDirtyInputs = new Set();", self.source)
        self.assertIn("!routeTuningDirtyInputs.has(input)", self.source)
        self.assertIn('input.addEventListener("input"', self.source)
        self.assertIn("routeTuningDirtyInputs.delete(input)", self.source)

    def test_route_tuning_save_is_serialized_and_has_network_margin(self) -> None:
        self.assertIn("if (routeTuningBusy) return;", self.source)
        self.assertIn("routeTuningBusy = true;", self.source)
        self.assertIn("routeTuningBusy = false;", self.source)
        self.assertIn("JSON.stringify(payload)}, 5000)", self.source)
        self.assertIn("输入值已保留", self.source)


if __name__ == "__main__":
    unittest.main()
