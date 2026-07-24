import unittest

from line_tracker import RightAngleTracker, State


class RightAngleTrackerTests(unittest.TestCase):
    def test_left_square_turn_searches_then_reacquires(self):
        tracker = RightAngleTracker(image_centre_x=320)
        actions = [tracker.step(value) for value in (315, 300, None, None, None, 318, 321)]
        self.assertEqual(actions, ["F", "F", "STOP", "PL", "PL", "STOP", "F"])
        self.assertEqual(tracker.state, State.FOLLOW)

    def test_right_square_turn_searches_right(self):
        tracker = RightAngleTracker(image_centre_x=320)
        actions = [tracker.step(value) for value in (330, 360, None, None)]
        self.assertEqual(actions, ["F", "FR", "STOP", "PR"])
        self.assertEqual(tracker.state, State.SEARCH_RIGHT)

    def test_never_pivots_forever_when_line_is_not_found(self):
        tracker = RightAngleTracker(image_centre_x=320)
        tracker.step(290)
        for _ in range(40):
            action = tracker.step(None)
        self.assertEqual(action, "STOP")
        self.assertEqual(tracker.state, State.LOST)


if __name__ == "__main__":
    unittest.main(verbosity=2)
