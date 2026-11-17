
from simulation import ThymioSimulator
from local_controller import LocalNavigationController


robot = ThymioSimulator()
controller = LocalNavigationController()

# Simulated global path
global_path = [
    (0.0, 0.0),
    (0.5, 0.0),
    (1.0, 0.0),
    (1.5, 0.0),
    (2.0, 0.0)
]

# Simulated proximity sensor readings
sensor_readings = [
    [0, 0, 0, 0, 0, 0, 0],
    [0, 0, 3000, 0, 0, 0, 0],
    [0, 0, 1000, 0, 1900, 0, 0],
    [0, 0, 300, 0, 1800, 0, 0],
    [0, 0, 100, 0, 0, 0, 0]
]

for step, sensors in enumerate(sensor_readings):
    pose = (robot.x, robot.y, robot.theta)

    # Temporary demonstration flag
    can_return = (step == 4)

    state, motors = controller.update(
        proximity=sensors,
        normal_command=(100, 100),
        can_return=can_return,
        robot_pose=pose,
        global_path=global_path
    )

    new_pose = robot.update(
        left_motor=motors[0],
        right_motor=motors[1],
        dt=0.1
    )

    print(
        f"Step {step}: "
        f"State={state.name}, "
        f"Pose={tuple(round(v, 3) for v in new_pose)}"
    )
