from __future__ import annotations
from types import SimpleNamespace
import unittest
from red_band_planner import TwoRedBandPlanner


def layer(y, bottom, span=180):
    return SimpleNamespace(y=y, bottom_y=bottom, span=span)


class TwoRedBandPlannerTests(unittest.TestCase):
    def test_first_band_slows_and_two_layers_remain_approach(self):
        planner = TwoRedBandPlanner()
        self.assertEqual(planner.update([layer(330, 350)], 640, 480).event, "SLOW_DOWN")
        decision = planner.update([layer(210, 225), layer(355, 370)], 640, 480)
        self.assertEqual((decision.event, decision.phase), ("SLOW_DOWN", "TWO_LAYERS_APPROACH"))

    def test_turn_band_brakes_then_pivots(self):
        planner = TwoRedBandPlanner(); planner.update([layer(200, 215), layer(340, 355)], 640, 480)
        # First red band pre-authorizes earlier 35%/70% thresholds.
        self.assertEqual(planner.update([layer(170, 185)], 640, 480).event, "BRAKE_NOW")
        self.assertEqual(planner.update([layer(320, 340)], 640, 480).event, "PIVOT_REQUEST")

    def test_exit_after_bottom_arming_reverses_if_pivot_was_missed(self):
        planner = TwoRedBandPlanner(); planner.update([layer(210, 225), layer(340, 355)], 640, 480)
        planner.update([layer(280, 300)], 640, 480)  # arm exit, but below pivot threshold
        self.assertEqual(planner.update([], 640, 480).event, "REVERSE_REQUEST")
