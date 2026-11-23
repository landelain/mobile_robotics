from simulation import ThymioSimulator
from simulated_sensors import simulate_proximity
from local_controller import LocalNavigationController
from path_recovery import find_rejoin_point, is_path_clear


robot = ThymioSimulator(x=0.0, y=0.0, theta=0.0)
controller = LocalNavigationController()

# Simulated obstacle: (x, y, radius), meters
obstacles = [(0.35, 0.0, 0.06)]

# Straight global path
global_path = [
    (0.0, 0.0),
    (0.5, 0.0),
    (1.0, 0.0)
]

DT = 0.1
STEPS = 150

for step in range(STEPS):
    pose = (robot.x, robot.y, robot.theta)

    # Sensor readings now depend on the robot's position
    proximity = simulate_proximity(pose, obstacles)

    # Find a point on the original path
    rejoin_target = find_rejoin_point(pose, global_path)

    # Check whether the route to that point is clear
    can_return = is_path_clear(

    robot_pose=pose,
    target=rejoin_target,
    obstacles=obstacles,
    safety_margin=0.06
)

    state, motors = controller.update(
        proximity=proximity,
        normal_command=(100, 100),
        can_return=can_return,  # Return logic not enabled yet
        robot_pose=pose,
        global_path=global_path
    )

    robot.update(motors[0], motors[1], dt=DT)

    if step % 10 == 0:
        print(
            f"Step {step}: "
            f"State={state.name}, "
            f"x={robot.x:.3f}, y={robot.y:.3f}, "
            f"theta={robot.theta:.2f}, "
            f"front={proximity[2]}"
        )
