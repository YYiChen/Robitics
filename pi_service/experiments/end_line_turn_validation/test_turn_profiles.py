from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from turn_profiles import TurnProfile, load_turn_profile, save_turn_profile


class TurnProfileTests(unittest.TestCase):
    def test_saves_and_loads_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "turn.json"
            wanted = TurnProfile(190, 1.25)
            save_turn_profile(path, wanted)
            self.assertEqual(load_turn_profile(path, TurnProfile(1, .1)), wanted)

    def test_invalid_profile_falls_back(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "turn.json"
            path.write_text('{"pwm": 300, "step_seconds": -1}', encoding="utf-8")
            self.assertEqual(load_turn_profile(path, TurnProfile(200, 2.5)), TurnProfile(200, 2.5))

    def test_legacy_total_duration_migrates_to_per_step_duration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "turn.json"
            path.write_text('{"pwm": 200, "preset_seconds": 2.8}', encoding="utf-8")
            self.assertEqual(load_turn_profile(path, TurnProfile(1, .1), steps=2), TurnProfile(200, 1.4))
