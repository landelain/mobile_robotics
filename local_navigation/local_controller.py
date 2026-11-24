
from enum import Enum

from obstacle_avoidance import detect_obstacle, choose_direction
from path_recovery import (
    find_rejoin_point,
    calculate_return_command,
    reached_rejoin_point
)


# Controller parameters
BASE_SPEED = 100
TURN_SPEED = 80

CLEAR_THRESHOLD = 1000
CORNER_CLEAR_THRESHOLD = 1500
SIDE_VISIBLE_THRESHOLD = 1000
SIDE_LOST_THRESHOLD = 300

SIDE_TARGET = 2000
SIDE_TOLERANCE = 100
SIDE_GAIN = 0.02
MAX_CORRECTION = 30

FRONT_BLOCKED_THRESHOLD = 2000


class NavigationState(Enum):
    FOLLOW_PATH = 1
    AVOID_OBSTACLE = 2
    FOLLOW_BOUNDARY = 3
    RETURN_TO_PATH = 4


class LocalNavigationController:

    def __init__(self):
        self.state = NavigationState.FOLLOW_PATH
        self.turn_direction = "left"
        self.boundary_side = "right"
        self.rejoin_target = None

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

            center_clear = (
                proximity[2] < CLEAR_THRESHOLD
            )

            if self.boundary_side == "right":
                corner_sensor = proximity[3]
            else:
                corner_sensor = proximity[1]

            corner_clear = (
                corner_sensor < CORNER_CLEAR_THRESHOLD
            )

            side_visible = (
                side_sensor > SIDE_VISIBLE_THRESHOLD
            )

            # Turn until obstacle is alongside
            # and the front corner is sufficiently clear
            if center_clear and corner_clear and side_visible:
                self.state = NavigationState.FOLLOW_BOUNDARY

        # ---------------------------------
        # STATE 3: FOLLOW BOUNDARY
        # ---------------------------------

        elif self.state == NavigationState.FOLLOW_BOUNDARY:

            # Front obstacle takes priority
            if max(front) > FRONT_BLOCKED_THRESHOLD:
                self.state = NavigationState.AVOID_OBSTACLE

            # Return only when external logic approves
            elif (
                can_return
                and robot_pose is not None
                and global_path
                and max(front) < CORNER_CLEAR_THRESHOLD
            ):

                self.rejoin_target = find_rejoin_point(
                    robot_pose, global_path
                )

                self.state = NavigationState.RETURN_TO_PATH

        # ---------------------------------
        # STATE 4: RETURN TO PATH
        # ---------------------------------

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

        # ---------------------------------
        # DETERMINE MOTOR COMMANDS
        # ---------------------------------

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


# ---------------------------------
# SIMPLE SOFTWARE TEST
# ---------------------------------

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
