# Local Navigation:
# Initial version: simulated sensor readings

THRESHOLD = 2000
BASE_SPEED = 100
TURN_SPEED = 80


def detect_obstacle(proximity):
    """Check the five front proximity sensors."""
    return max(proximity[:5]) > THRESHOLD


def choose_direction(proximity):
    """Turn toward the side with less reflection."""
    left = proximity[0] + proximity[1]
    right = proximity[3] + proximity[4]

    return "right" if left > right else "left"


def local_navigation(proximity):
    """
    Return simulated left and right motor commands.
    These values are not yet calibrated for Thymio.
    """
    if detect_obstacle(proximity):
        direction = choose_direction(proximity)

        if direction == "left":
            return -TURN_SPEED, TURN_SPEED
        else:
            return TURN_SPEED, -TURN_SPEED

    return BASE_SPEED, BASE_SPEED


# Test with simulated sensor values
if __name__ == "__main__":
    test_cases = {
        "No obstacle": [0, 0, 0, 0, 0, 0, 0],
        "Obstacle ahead": [0, 0, 3000, 0, 0, 0, 0],
        "Obstacle left": [3000, 2500, 0, 0, 0, 0, 0],
        "Obstacle right": [0, 0, 0, 2500, 3000, 0, 0],
    }

    for name, sensors in test_cases.items():
        left, right = local_navigation(sensors)

        print(f"{name}:")
        print(f"  Left motor: {left}")
        print(f"  Right motor: {right}")
