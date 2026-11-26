
from local_controller import (
    LocalNavigationController,
    NavigationState
)
from path_recovery import (
    find_rejoin_point,
    is_path_clear
)


# Example filtered pose (mm, mm, radians)
robot_pose = (800.0, 300.0, 0.0)

# Example global-navigation coordinates (mm)
global_path = [
    (0.0, 0.0),
    (500.0, 0.0),
    (1000.0, 0.0),
    (1500.0, 0.0),
    (2000.0, 0.0)
]

# Example circular obstacles (x_mm, y_mm, radius_mm)
# These are test inputs, NOT real Vision output.
obstacles = [
    (350.0, 0.0, 60.0)
]

controller = LocalNavigationController()

# Simulate that local avoidance has completed
controller.state = NavigationState.FOLLOW_BOUNDARY
controller.boundary_side = "right"

# Find a waypoint on the original path
target = find_rejoin_point(robot_pose, global_path)

# Check whether the straight line is clear
path_clear = is_path_clear(
    robot_pose,
    target,
    obstacles
)

# Simulate sensors with an obstacle still on the right
proximity = [0, 0, 0, 0, 1800, 0, 0]

# Do not activate physical recovery based only on this test
can_return = path_clear

state, motors = controller.update(
    proximity=proximity,
    normal_command=(100, 100),
    can_return=can_return,
    robot_pose=robot_pose,
    global_path=global_path
)

print("Robot pose:", robot_pose)
print("Rejoin target:", target)
print("Path clear:", path_clear)
print("Can return (simulated):", can_return)
print("Navigation state:", state.name)
print("Motor commands:", motors)

assert path_clear is True
assert state == NavigationState.RETURN_TO_PATH

print("Integration test passed.")
