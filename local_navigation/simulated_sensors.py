
import math

# Approximate front sensor directions (radians)
SENSOR_ANGLES = [
    math.radians(40),
    math.radians(20),
    0,
    math.radians(-20),
    math.radians(-40)
]

MAX_RANGE = 0.15  # meters, illustrative
MAX_READING = 4000


def simulate_proximity(robot_pose, obstacles):
    """
    Generate simulated Thymio proximity readings.

    robot_pose: (x, y, theta)
    obstacles: list of (x, y, radius) circles

    Returns 7 readings: 5 front + 2 rear.
    """

    x, y, theta = robot_pose
    readings = [0] * 7

    for i, sensor_angle in enumerate(SENSOR_ANGLES):

        direction = theta + sensor_angle
        dx = math.cos(direction)
        dy = math.sin(direction)

        nearest = MAX_RANGE

        for ox, oy, radius in obstacles:

            # Ray-circle intersection
            fx = x - ox
            fy = y - oy

            b = 2 * (fx * dx + fy * dy)
            c = fx**2 + fy**2 - radius**2

            discriminant = b**2 - 4 * c

            if discriminant < 0:
                continue

            distance = (-b - math.sqrt(discriminant)) / 2

            if 0 <= distance < nearest:
                nearest = distance

        if nearest < MAX_RANGE:
            readings[i] = int(
                MAX_READING * (1 - nearest / MAX_RANGE)
            )

    return readings


if __name__ == "__main__":

    # Robot facing along positive x-axis
    robot_pose = (0.0, 0.0, 0.0)

    # Circular obstacle 10 cm directly ahead
    obstacles = [
        (0.10, 0.0, 0.03)
    ]

    sensors = simulate_proximity(robot_pose, obstacles)

    print("Simulated proximity readings:", sensors)
