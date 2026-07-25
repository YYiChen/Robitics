import unittest

from four_endpoint_planner import (
    DriveAction,
    Endpoint,
    FourEndpointPlanner,
    Heading,
    NavigationState,
    VisionObservation,
)


def seen(*, junction=False, line=False, dealt=False):
    return VisionObservation(junction_detected=junction, forward_line_detected=line, deal_complete=dealt)


def finish_pivot(planner: FourEndpointPlanner):
    """Feed the known pivot landmark sequence: blank, then centred line twice."""
    planner.step(seen(line=False))
    planner.step(seen(line=True))
    return planner.step(seen(line=True))


class FourEndpointPlannerTests(unittest.TestCase):
    def test_top_left_is_a_single_visual_left_90_then_deal(self):
        planner = FourEndpointPlanner((Endpoint.TOP_LEFT,))
        self.assertIs(planner.step(seen()).action, DriveAction.FOLLOW_STEM)
        self.assertIs(planner.step(seen(junction=True)).action, DriveAction.FOLLOW_STEM)
        self.assertIs(planner.step(seen(junction=True)).state, NavigationState.STOP_AT_JUNCTION)
        self.assertIs(planner.step(seen()).action, DriveAction.PIVOT_LEFT_90)
        result = finish_pivot(planner)
        self.assertIs(result.state, NavigationState.DEAL_CARD)
        self.assertIs(result.action, DriveAction.STOP)
        self.assertIs(result.heading, Heading.LEFT)

    def test_1_to_2_uses_two_90_degree_pivots_and_never_crossbar_drive(self):
        planner = FourEndpointPlanner((Endpoint.TOP_LEFT, Endpoint.TOP_RIGHT))
        planner.step(seen(junction=True)); planner.step(seen(junction=True)); planner.step(seen())
        finish_pivot(planner)  # face endpoint 1
        first_deal = planner.step(seen(dealt=True))
        self.assertIs(first_deal.action, DriveAction.PIVOT_LEFT_90)  # LEFT -> DOWN
        first_return = finish_pivot(planner)
        self.assertIs(first_return.action, DriveAction.PIVOT_LEFT_90)  # DOWN -> RIGHT
        second_target = finish_pivot(planner)
        self.assertIs(second_target.state, NavigationState.DEAL_CARD)
        self.assertIs(second_target.heading, Heading.RIGHT)
        self.assertNotIn(DriveAction.FOLLOW_STEM, (first_deal.action, first_return.action, second_target.action))

    def test_1_to_2_to_3_returns_to_stem_and_drives_down(self):
        planner = FourEndpointPlanner((Endpoint.TOP_LEFT, Endpoint.TOP_RIGHT, Endpoint.BOTTOM_LEFT))
        planner.step(seen(junction=True)); planner.step(seen(junction=True)); planner.step(seen()); finish_pivot(planner)
        planner.step(seen(dealt=True)); finish_pivot(planner); finish_pivot(planner)  # endpoint 2
        leave_top = planner.step(seen(dealt=True))
        self.assertIs(leave_top.action, DriveAction.PIVOT_RIGHT_90)  # RIGHT -> DOWN
        return_to_stem = finish_pivot(planner)
        self.assertIs(return_to_stem.state, NavigationState.FOLLOW_STEM)
        self.assertIs(return_to_stem.heading, Heading.DOWN)
        planner.step(seen(junction=True))
        self.assertIs(planner.step(seen(junction=True)).state, NavigationState.STOP_AT_JUNCTION)
        turn_bottom_left = planner.step(seen())
        self.assertIs(turn_bottom_left.action, DriveAction.PIVOT_RIGHT_90)  # DOWN -> LEFT

    def test_dealing_holds_the_drive_wheels_stopped(self):
        planner = FourEndpointPlanner((Endpoint.TOP_LEFT,))
        planner.step(seen(junction=True)); planner.step(seen(junction=True)); planner.step(seen()); finish_pivot(planner)
        result = planner.step(seen(dealt=False))
        self.assertIs(result.state, NavigationState.DEAL_CARD)
        self.assertIs(result.action, DriveAction.DEAL_CARD)

    def test_full_1_2_3_4_route_visits_every_endpoint_without_crossbar_drive(self):
        planner = FourEndpointPlanner((Endpoint.TOP_LEFT, Endpoint.TOP_RIGHT, Endpoint.BOTTOM_LEFT, Endpoint.BOTTOM_RIGHT))
        faced = []
        guard = 0
        while planner.state is not NavigationState.COMPLETE and guard < 80:
            guard += 1
            if planner.state is NavigationState.FOLLOW_STEM:
                planner.step(seen(junction=True))
                result = planner.step(seen(junction=True))
                self.assertIs(result.action, DriveAction.STOP)
            elif planner.state is NavigationState.STOP_AT_JUNCTION:
                planner.step(seen())
            elif planner.state is NavigationState.PIVOT_TO_HEADING:
                finish_pivot(planner)
            elif planner.state is NavigationState.DEAL_CARD:
                faced.append(planner.active_target)
                planner.step(seen(dealt=True))
        self.assertEqual(faced, [Endpoint.TOP_LEFT, Endpoint.TOP_RIGHT, Endpoint.BOTTOM_LEFT, Endpoint.BOTTOM_RIGHT])
        self.assertIs(planner.state, NavigationState.COMPLETE)


if __name__ == "__main__":
    unittest.main()
