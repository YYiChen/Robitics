import unittest

from i_turnaround_logic import IShapeTurnaroundPlanner, RouteEvidence, TurnaroundConfig, TurnaroundState


def route(*, marker=False, marker_y=300, vertical=True):
    path = ((320, 450), (320, 350), (320, 250)) if vertical else ((120, 350), (300, 350), (520, 350))
    return RouteEvidence(.9, False, path[1], path, marker, (320, marker_y) if marker else None, 4 if marker else 0, 480)


class IShapeTurnaroundPlannerTests(unittest.TestCase):
    def test_endpoint_requires_a_lower_transverse_bar(self):
        planner = IShapeTurnaroundPlanner(TurnaroundConfig(end_confirm_frames=2, pivot_min_seconds=1, pivot_max_seconds=5))
        self.assertIs(planner.step(route(marker=False), 0).state, TurnaroundState.FOLLOW_STRAIGHT)
        # A distant horizontal mark is not yet an endpoint.
        self.assertIs(planner.step(route(marker=True, marker_y=200), 1).state, TurnaroundState.FOLLOW_STRAIGHT)
        self.assertIs(planner.step(route(marker=True, marker_y=320), 2).state, TurnaroundState.FOLLOW_STRAIGHT)
        self.assertIs(planner.step(route(marker=True, marker_y=320), 3).state, TurnaroundState.PIVOT_180)


    def test_pivot_rejects_transverse_bar_then_accepts_longitudinal_line(self):
        planner = IShapeTurnaroundPlanner(TurnaroundConfig(end_confirm_frames=1, reacquire_confirm_frames=2, pivot_min_seconds=1, pivot_max_seconds=5))
        self.assertIs(planner.step(route(marker=True, marker_y=320), 0).state, TurnaroundState.PIVOT_180)
        self.assertIs(planner.step(route(marker=True, marker_y=320), 1.2).state, TurnaroundState.PIVOT_180)
        self.assertIs(planner.step(route(marker=False), 1.3).state, TurnaroundState.PIVOT_180)
        self.assertIs(planner.step(route(marker=False), 1.4).state, TurnaroundState.FOLLOW_STRAIGHT)
