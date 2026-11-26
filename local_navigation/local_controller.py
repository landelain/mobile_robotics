
from enum import Enum

from obstacle_avoidance import (
    CORNER_BLOCKED_THRESHOLD,
    CORNER_CLEAR_THRESHOLD,
    FRONT_BLOCKED_THRESHOLD,
    FRONT_CLEAR_THRESHOLD,
    SIDE_LOST_THRESHOLD,
    SIDE_VISIBLE_THRESHOLD,
    detect_obstacle,
    choose_direction,
)
from path_recovery import (
    find_rejoin_point,
    calculate_return_command,
    reached_rejoin_point
)

# Controller parameters
BASE_SPEED = 100
TURN_SPEED = 80

SIDE_TARGET = 2000
SIDE_TOLERANCE = 100
SIDE_GAIN = 0.02
MAX_CORRECTION = 30

CORNER_INNER_SPEED = 35
CORNER_OUTER_SPEED = 55

SEARCH_MAX_STEPS = 8
SEARCH_OUTER_SPEED = 40
SEARCH_INNER_SPEED = 20

class NavigationState(Enum):
    FOLLOW_PATH = 1
    AVOID_OBSTACLE = 2
    FOLLOW_BOUNDARY = 3
    RETURN_TO_PATH = 4
    SEARCH_BOUNDARY = 5
    STOPPED = 6
    CORNER_TURN = 7

class LocalNavigationController:

    def __init__(self):
        self.state = NavigationState.FOLLOW_PATH
        self.turn_direction = "left"
        self.boundary_side = "right"
        self.rejoin_target = None
        self.search_steps = 0

    def update(
        self,
        proximity,
        normal_command=(100, 100),
        can_return=False,
        robot_pose=None,
        global_path=None
    ):

        if len(proximity) < 5:
            raise ValueError(
                "Expected at least 5 proximity readings"
            )

        front = proximity[1:4]
        left_side = proximity[0]
        right_side = proximity[4]

        side_sensor = (
            right_side if self.boundary_side == "right"
            else left_side
        )
        corner_sensor = (
            proximity[3] if self.boundary_side == "right"
            else proximity[1]
        )

        # ---------------------------------
        # STATE 1: FOLLOW ORIGINAL PATH
        # ---------------------------------

        if self.state == NavigationState.FOLLOW_PATH:

            if detect_obstacle(proximity):

                self.turn_direction = choose_direction(
                    proximity
                )

                self.boundary_side = (
                    "right" if self.turn_direction == "left"
                    else "left"
                )

                self.rejoin_target = None
                self.state = NavigationState.AVOID_OBSTACLE

        # ---------------------------------
        # STATE 2: AVOID OBSTACLE
        # ---------------------------------

        elif self.state == NavigationState.AVOID_OBSTACLE:

            center_clear = proximity[2] <= FRONT_CLEAR_THRESHOLD

            corner_blocked = corner_sensor >= CORNER_BLOCKED_THRESHOLD
            side_visible = side_sensor >= SIDE_VISIBLE_THRESHOLD

            # Once the center is clear and the obstacle is alongside, use a
            # forward corner arc if the obstacle-side corner is still close.
            if center_clear and side_visible:
                self.state = (
                    NavigationState.CORNER_TURN
                    if corner_blocked
                    else NavigationState.FOLLOW_BOUNDARY
                )
        
        # STATE 3: FOLLOW BOUNDARY
        elif self.state == NavigationState.FOLLOW_BOUNDARY:

            # Front obstacle takes priority
            if proximity[2] >= FRONT_BLOCKED_THRESHOLD:
                self.state = NavigationState.AVOID_OBSTACLE

            elif side_sensor < SIDE_LOST_THRESHOLD:
                self.search_steps = 0
                self.state = NavigationState.SEARCH_BOUNDARY

            elif corner_sensor >= CORNER_BLOCKED_THRESHOLD:
                self.state = NavigationState.CORNER_TURN

            # External path recovery takes priority when approved
            elif (
                can_return
                and robot_pose is not None
                and global_path
                and max(front) <= CORNER_CLEAR_THRESHOLD
            ):
                self.rejoin_target = find_rejoin_point(
                    robot_pose, global_path
                )
                self.state = NavigationState.RETURN_TO_PATH


        # STATE 4: TURN AROUND AN OBSTACLE CORNER
        elif self.state == NavigationState.CORNER_TURN:

            if proximity[2] >= FRONT_BLOCKED_THRESHOLD:
                self.state = NavigationState.AVOID_OBSTACLE

            elif side_sensor < SIDE_LOST_THRESHOLD:
                self.search_steps = 0
                self.state = NavigationState.SEARCH_BOUNDARY

            elif corner_sensor <= CORNER_CLEAR_THRESHOLD:
                self.state = NavigationState.FOLLOW_BOUNDARY

        # STATE 5: RETURN TO PATH
        elif self.state == NavigationState.RETURN_TO_PATH:

            if detect_obstacle(proximity):
                self.turn_direction = choose_direction(
                    proximity
                )
                self.boundary_side = (
                    "right" if self.turn_direction == "left"
                    else "left"
                )
                self.rejoin_target = None
                self.state = NavigationState.AVOID_OBSTACLE

            elif (
                robot_pose is not None
                and self.rejoin_target is not None
            ):

                if reached_rejoin_point(
                    robot_pose, self.rejoin_target
                ):
                    self.state = NavigationState.FOLLOW_PATH
                    self.rejoin_target = None

         # STATE 6: SEARCH FOR BOUNDARY
        elif self.state == NavigationState.SEARCH_BOUNDARY:

            # Don't continue searching into a front obstacle
            if max(front) >= FRONT_BLOCKED_THRESHOLD:
                self.state = NavigationState.STOPPED

            # Successfully found the boundary again
            elif side_sensor > SIDE_VISIBLE_THRESHOLD:
                self.search_steps = 0
                self.state = NavigationState.FOLLOW_BOUNDARY

            else:
                self.search_steps += 1

                # Stop if the boundary cannot be found
                if self.search_steps >= SEARCH_MAX_STEPS:
                    self.state = NavigationState.STOPPED

        # DETERMINE MOTOR COMMANDS
        if self.state == NavigationState.FOLLOW_PATH:
            command = normal_command

        elif self.state == NavigationState.AVOID_OBSTACLE:

            if self.turn_direction == "left":
                command = (-TURN_SPEED, TURN_SPEED)
            else:
                command = (TURN_SPEED, -TURN_SPEED)

        elif self.state == NavigationState.FOLLOW_BOUNDARY:

            side_sensor = (
                right_side if self.boundary_side == "right"
                else left_side
            )

            # Boundary lost: stop rather than drive blindly
            if side_sensor < SIDE_LOST_THRESHOLD:
                command = (0, 0)

            else:
                error = SIDE_TARGET - side_sensor

                correction = SIDE_GAIN * error
                correction = max(
                    -MAX_CORRECTION,
                    min(MAX_CORRECTION, correction)
                )

                if abs(error) < SIDE_TOLERANCE:
                    correction = 0

                if self.boundary_side == "right":
                    command = (
                        BASE_SPEED + correction,
                        BASE_SPEED - correction
                    )
                else:
                    command = (
                        BASE_SPEED - correction,
                        BASE_SPEED + correction
                    )


        elif self.state == NavigationState.SEARCH_BOUNDARY:

            # Curve gently toward the side where the
            # obstacle was last detected.
            if self.boundary_side == "right":
                command = (
                    SEARCH_OUTER_SPEED,
                    SEARCH_INNER_SPEED
                )
            else:
                command = (
                    SEARCH_INNER_SPEED,
                    SEARCH_OUTER_SPEED
                )

        elif self.state == NavigationState.CORNER_TURN:

            # Move forward while turning away from the obstacle-side corner.
            # This prevents an in-place spin from remaining trapped at a
            # corner whose front-side sensor is still reporting strongly.
            if self.boundary_side == "right":
                command = (CORNER_INNER_SPEED, CORNER_OUTER_SPEED)
            else:
                command = (CORNER_OUTER_SPEED, CORNER_INNER_SPEED)

        elif self.state == NavigationState.STOPPED:
            command = (0, 0)

        elif self.state == NavigationState.RETURN_TO_PATH:

            if (
                robot_pose is not None
                and self.rejoin_target is not None
            ):
                command = calculate_return_command(
                    robot_pose, self.rejoin_target
                )
            else:
                command = (0, 0)

        return self.state, command

# SIMPLE SOFTWARE TEST
if __name__ == "__main__":

    controller = LocalNavigationController()

    global_path = [
        (0.0, 0.0),
        (0.5, 0.0),
        (1.0, 0.0),
        (1.5, 0.0),
        (2.0, 0.0)
    ]

    robot_pose = (0.8, 0.3, 0.0)

    tests = [
        ("Clear path", [0, 0, 0, 0, 0, 0, 0], False),
        ("Obstacle", [0, 0, 3000, 0, 0, 0, 0], False),
        ("Front clear", [0, 0, 400, 0, 1900, 0, 0], False),
        ("Follow side", [0, 0, 0, 0, 1800, 0, 0], False),
        ("Boundary lost", [0, 0, 0, 0, 0, 0, 0], False),
        ("Return allowed", [0, 0, 0, 0, 0, 0, 0], True)
    ]

    for label, sensors, can_return in tests:

        state, motors = controller.update(
            proximity=sensors,
            can_return=can_return,
            robot_pose=robot_pose,
            global_path=global_path
        )

        print(f"{label}: {state.name}, motors={motors}")

    robot_pose = (1.5, 0.0, 0.0)

    state, motors = controller.update(
        proximity=[0] * 7,
        robot_pose=robot_pose,
        global_path=global_path
    )

    print(f"Path rejoined: {state.name}, motors={motors}")
