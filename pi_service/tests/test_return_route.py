import json
import sys
import tempfile
import unittest
from pathlib import Path


sys.path.insert(
    0,
    str(Path(__file__).parents[1] / "robot_web" / "routes" / "end_line"),
)

from return_route import (  # noqa: E402
    ReturnRouteRecorder,
    blend_return_pwm,
)


class ReturnRouteTests(unittest.TestCase):
    def test_deal_checkpoints_split_segments_and_never_store_card_commands(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "return.json"
            recorder = ReturnRouteRecorder(path, nominal_sample_seconds=.05)
            recorder.start_recording()
            recorder.record(
                80, 90, line_center_x=300, confidence=1.0,
                wheel_speed=[10, 20], now=1.0,
            )
            self.assertTrue(recorder.checkpoint("deal_complete_1"))
            recorder.record(
                100, 70, line_center_x=320, confidence=.8,
                wheel_speed=[12, 18], now=2.0,
            )

            replay = recorder.prepare_return()
            stored = json.loads(path.read_text(encoding="utf-8"))

            self.assertEqual(
                [segment["checkpoint"] for segment in stored["segments"]],
                ["deal_complete_1", "return_start"],
            )
            self.assertNotIn("M3", path.read_text(encoding="utf-8"))
            self.assertNotIn("M4", path.read_text(encoding="utf-8"))
            first_return = replay.step(now=10.0)
            self.assertEqual(first_return.segment_index, 1)
            self.assertEqual(first_return.forward_facing_pwm, (70, 100))

    def test_replay_reverses_segment_and_sample_order(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            recorder = ReturnRouteRecorder(
                Path(directory) / "return.json",
                nominal_sample_seconds=.01,
            )
            recorder.start_recording()
            recorder.record(10, 20, line_center_x=1, confidence=1, now=1.0)
            recorder.record(30, 40, line_center_x=2, confidence=1, now=1.01)
            recorder.checkpoint("deal")
            recorder.record(50, 60, line_center_x=3, confidence=1, now=2.0)
            replay = recorder.prepare_return()

            self.assertEqual(replay.step(5.0).forward_facing_pwm, (60, 50))
            self.assertEqual(replay.step(5.02).forward_facing_pwm, (40, 30))
            self.assertEqual(replay.step(5.04).forward_facing_pwm, (20, 10))
            self.assertTrue(replay.step(5.06).complete)

    def test_vision_keeps_majority_authority_over_replay(self) -> None:
        self.assertEqual(blend_return_pwm((80, 100), (120, 60), .25), (90, 90))
        self.assertEqual(blend_return_pwm((80, 100), (255, -255), 1.0), (168, -78))


if __name__ == "__main__":
    unittest.main()
