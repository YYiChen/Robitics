from i_turnaround_logic import IShapeTurnaroundPlanner, RouteEvidence, TurnaroundConfig, TurnaroundState


def route(*, marker=False, end=False, vertical=True):
    path = ((320, 450), (320, 350), (320, 250 if not end else 300)) if vertical else ((120, 350), (300, 350), (520, 350))
    return RouteEvidence(.9, False, path[1], path, marker, (320, 300) if marker else None, 4 if marker else 0, 480)


def test_endpoint_requires_transverse_bar_and_route_termination():
    planner = IShapeTurnaroundPlanner(TurnaroundConfig(end_confirm_frames=2, pivot_min_seconds=1, pivot_max_seconds=5))
    assert planner.step(route(marker=False, end=True), 0).state is TurnaroundState.FOLLOW_STRAIGHT
    assert planner.step(route(marker=True, end=True), 1).state is TurnaroundState.FOLLOW_STRAIGHT
    assert planner.step(route(marker=True, end=True), 2).state is TurnaroundState.PIVOT_180


def test_pivot_rejects_transverse_bar_then_accepts_longitudinal_line():
    planner = IShapeTurnaroundPlanner(TurnaroundConfig(end_confirm_frames=1, reacquire_confirm_frames=2, pivot_min_seconds=1, pivot_max_seconds=5))
    assert planner.step(route(marker=True, end=True), 0).state is TurnaroundState.PIVOT_180
    assert planner.step(route(marker=True, end=True), 1.2).state is TurnaroundState.PIVOT_180
    assert planner.step(route(marker=False, end=False), 1.3).state is TurnaroundState.PIVOT_180
    assert planner.step(route(marker=False, end=False), 1.4).state is TurnaroundState.FOLLOW_STRAIGHT
