import math

def find_rejoin_point(robot_pose, global_path):
    """
    Select a suitable waypoint on the planned path.

    robot_pose: (x, y, theta)
    global_path: list of (x, y) waypoints

    Returns: (x, y) of the selected waypoint
    """

    if not global_path:
        raise ValueError("Global path cannot be empty")

    x, y, theta = robot_pose

    # Find closest waypoint
    distances = [
        math.hypot(px - x, py - y)
        for px, py in global_path
    ]

    closest_index = distances.index(min(distances))

    # Choose next waypoint if available
    next_index = min(
        closest_index + 1,
        len(global_path) - 1
    )

    return global_path[next_index]


def calculate_return_command(robot_pose, target):
    """
    Calculate motor commands to drive toward a target.

    robot_pose: (x_mm, y_mm, theta_rad)
    target: (x_mm, y_mm)

    Returns: (left_speed, right_speed)
    """
    x, y, theta = robot_pose
    tx, ty = target

    desired_angle = math.atan2(ty - y, tx - x)

    # Normalize heading error to [-pi, pi]
    angle_error = (
        desired_angle - theta + math.pi
    ) % (2 * math.pi) - math.pi

    # Preliminary controller gains
    BASE_SPEED = 100
    TURN_GAIN = 60

    left_speed = BASE_SPEED - TURN_GAIN * angle_error
    right_speed = BASE_SPEED + TURN_GAIN * angle_error

    return left_speed, right_speed

def reached_rejoin_point(robot_pose, target, tolerance=80.0):
    """
    Check whether the robot has reached its rejoin point.

    robot_pose: (x_mm, y_mm, theta_rad)
    target: (x_mm, y_mm)
    tolerance: acceptable distance in millimetres
    """
    x, y, _ = robot_pose
    tx, ty = target

    distance = math.hypot(tx - x, ty - y)

    return distance < tolerance

def is_path_clear(robot_pose, target, obstacles, safety_margin=60.0):
    """
    Check whether the straight path to a target
    intersects any circular obstacle.

    robot_pose: (x_mm, y_mm, theta_rad)
    target: (x_mm, y_mm)
    obstacles: list of (x_mm, y_mm, radius_mm)
    safety_margin: additional clearance in millimetres
    """
    x, y, _ = robot_pose
    tx, ty = target

    dx = tx - x
    dy = ty - y

    length_squared = dx**2 + dy**2

    if length_squared == 0:
        return True

    for ox, oy, radius in obstacles:

        # Project obstacle center onto the line segment
        t = ((ox - x) * dx + (oy - y) * dy) / length_squared
        t = max(0.0, min(1.0, t))

        # Closest point on the segment
        closest_x = x + t * dx
        closest_y = y + t * dy

        distance = math.hypot(
            ox - closest_x,
            oy - closest_y
        )

        if distance < radius + safety_margin:
            return False

    return True

if __name__ == "__main__":

    # All positions and distances use millimetres.
    # Orientation uses radians.

    path = [
        (0.0, 0.0),
        (500.0, 0.0),
        (1000.0, 0.0),
        (1500.0, 0.0),
        (2000.0, 0.0)
    ]

    robot_pose = (800.0, 300.0, 0.0)

    target = find_rejoin_point(robot_pose, path)

    print("Robot pose (mm):", robot_pose)
    print("Rejoin point (mm):", target)

    motors = calculate_return_command(
        robot_pose, target
    )

    print("Return motor commands:", motors)

    # Example circular obstacle
    obstacles = [(350.0, 0.0, 60.0)]

    print(
        "Path clear:",
        is_path_clear(robot_pose, target, obstacles)
    )

    # Verify arrival at the selected waypoint
    at_target = (target[0], target[1], 0.0)

    print(
        "Reached rejoin point:",
        reached_rejoin_point(at_target, target)
    )


