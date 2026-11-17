from enum import Enum
from obstacle_avoidance import detect_obstacle, choose_direction
from path_recovery import (
    find_rejoin_point,
    calculate_return_command,
    reached_rejoin_point
)

BASE_SPEED = 100
TURN_SPEED = 80

CLEAR_THRESHOLD = 1000
SIDE_TARGET = 1800
SIDE_TOLERANCE = 400


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

    def update(self, proximity, normal_command=(100, 100),
               can_return=False, robot_pose=None, global_path=None):

        if len(proximity) < 5:
            raise ValueError("Expected at least 5 proximity readings")

        front = proximity[1:4]
        left_side = proximity[0]
        right_side = proximity[4]

        # STATE 1: Follow original path
        if self.state == NavigationState.FOLLOW_PATH:

            if detect_obstacle(proximity):
                self.turn_direction = choose_direction(proximity)

                # Turn left -> follow obstacle on right
                # Turn right -> follow obstacle on left
                self.boundary_side = (
                    "right" if self.turn_direction == "left"
                    else "left"
                )

                self.state = NavigationState.AVOID_OBSTACLE

        # STATE 2: Turn away from obstacle
        elif self.state == NavigationState.AVOID_OBSTACLE:

            if max(front) < CLEAR_THRESHOLD:
                self.state = NavigationState.FOLLOW_BOUNDARY

        
        # STATE 3: Follow obstacle boundary
        elif self.state == NavigationState.FOLLOW_BOUNDARY:

            # Front obstacle takes priority
            if max(front) > 2000:
                self.state = NavigationState.AVOID_OBSTACLE

            # Return only when path-recovery logic approves
            elif can_return and robot_pose is not None and global_path:
                self.rejoin_target = find_rejoin_point(
                    robot_pose, global_path
                )
                self.state = NavigationState.RETURN_TO_PATH

        # STATE 4: Return to planned path
        elif self.state == NavigationState.RETURN_TO_PATH:

            if max(front) > 2000:
                self.state = NavigationState.AVOID_OBSTACLE
                self.rejoin_target = None

            elif robot_pose is not None and self.rejoin_target is not None:

                if reached_rejoin_point(
                    robot_pose, self.rejoin_target
                ):
                    self.state = NavigationState.FOLLOW_PATH
                    self.rejoin_target = None

        # Determine motor commands
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

            # Proportional adjustment around a target sensor value
            error = SIDE_TARGET - side_sensor
            correction = max(-30, min(30, 0.02 * error))

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

            if robot_pose is not None and self.rejoin_target is not None:
                command = calculate_return_command(
                    robot_pose, self.rejoin_target
                )
            else:
                command = (0, 0)

        return self.state, command


if __name__ == "__main__":

    # Simulated sensor readings:
    # Five front sensors + two rear sensors
    test_sequence = [
        ("Clear path", [0, 0, 0, 0, 0, 0, 0], False),
        ("Obstacle ahead", [0, 0, 3000, 0, 0, 0, 0], False),
        ("Front cleared", [0, 0, 400, 0, 1900, 0, 0], False),
        ("Following side", [0, 0, 300, 0, 1800, 0, 0], False),
        ("Ready to return", [0, 0, 100, 0, 0, 0, 0], True),
    ]

    controller = LocalNavigationController()
    robot_pose = (0.8, 0.3, 0.0)

    global_path = [
        (0.0, 0.0),
        (0.5, 0.0),
        (1.0, 0.0),
        (1.5, 0.0),
        (2.0, 0.0)
    ]

    for label, sensors, can_return in test_sequence:

        state, motors = controller.update(
            sensors,
            can_return=can_return,
            robot_pose=robot_pose,
            global_path=global_path
        )

        print(f"{label}: {state.name}, motors={motors}")
    
    # Test whether the robot resumes normal navigation
    print("\nTesting return to original path:")

    robot_pose = (1.5, 0.0, 0.0)

    state, motors = controller.update(
        [0, 0, 0, 0, 0, 0, 0],
        robot_pose=robot_pose,
        global_path=global_path
    )

    print(f"{state.name}: motors={motors}")
