"""Offline tests for corner handling; never connects to a Thymio."""

import unittest

from local_controller import (
    CORNER_INNER_SPEED,
    CORNER_OUTER_SPEED,
    LocalNavigationController,
    NavigationState,
)
from obstacle_avoidance import (
    CORNER_BLOCKED_THRESHOLD,
    CORNER_CLEAR_THRESHOLD,
    FRONT_BLOCKED_THRESHOLD,
    is_critical_proximity,
)


def readings(left=0, front_left=0, center=0, front_right=0, right=0):
    return [left, front_left, center, front_right, right, 0, 0]


class CornerHandlingTests(unittest.TestCase):

    def test_corner_detection_enters_controlled_arc(self):
        controller = LocalNavigationController()
        controller.state = NavigationState.FOLLOW_BOUNDARY
        controller.boundary_side = "right"

        state, motors = controller.update(
            readings(center=400, front_right=CORNER_BLOCKED_THRESHOLD, right=2000)
        )

        self.assertEqual(state, NavigationState.CORNER_TURN)
        self.assertEqual(motors, (CORNER_INNER_SPEED, CORNER_OUTER_SPEED))
        self.assertGreater(motors[0], 0)
        self.assertGreater(motors[1], 0)

    def test_corner_arc_returns_to_boundary_following_after_hysteresis_clear(self):
        controller = LocalNavigationController()
        controller.state = NavigationState.CORNER_TURN
        controller.boundary_side = "right"

        state, motors = controller.update(
            readings(center=400, front_right=CORNER_CLEAR_THRESHOLD, right=1900)
        )

        self.assertEqual(state, NavigationState.FOLLOW_BOUNDARY)
        self.assertNotEqual(motors, (0, 0))

    def test_center_blockage_has_priority_over_corner_arc(self):
        controller = LocalNavigationController()
        controller.state = NavigationState.CORNER_TURN
        controller.boundary_side = "right"

        state, motors = controller.update(
            readings(center=FRONT_BLOCKED_THRESHOLD, front_right=3000, right=2000)
        )

        self.assertEqual(state, NavigationState.AVOID_OBSTACLE)
        self.assertLess(motors[0], 0)
        self.assertGreater(motors[1], 0)

    def test_avoidance_can_enter_corner_arc_without_stopping(self):
        controller = LocalNavigationController()
        controller.state = NavigationState.AVOID_OBSTACLE
        controller.boundary_side = "right"

        state, motors = controller.update(
            readings(center=1000, front_right=CORNER_BLOCKED_THRESHOLD, right=1400)
        )

        self.assertEqual(state, NavigationState.CORNER_TURN)
        self.assertEqual(motors, (CORNER_INNER_SPEED, CORNER_OUTER_SPEED))

    def test_search_safety_stop_remains_active(self):
        controller = LocalNavigationController()
        controller.state = NavigationState.SEARCH_BOUNDARY
        controller.boundary_side = "right"

        state, motors = controller.update(
            readings(center=FRONT_BLOCKED_THRESHOLD)
        )

        self.assertEqual(state, NavigationState.STOPPED)
        self.assertEqual(motors, (0, 0))

    def test_emergency_threshold_remains_independent(self):
        self.assertTrue(is_critical_proximity(readings(center=4500)))
        self.assertFalse(is_critical_proximity(readings(center=4499)))


if __name__ == "__main__":
    unittest.main()
