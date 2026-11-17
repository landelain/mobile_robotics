
import math


class ThymioSimulator:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

        # Approximate parameters for initial simulation
        self.wheel_base = 0.095  # meters
        self.speed_scale = 0.001  # m/s per motor command unit

    def update(self, left_motor, right_motor, dt=0.1):
        """Update robot pose using differential-drive kinematics."""

        # Convert motor commands to wheel linear velocities
        v_l = left_motor * self.speed_scale
        v_r = right_motor * self.speed_scale

        # Linear and angular velocity
        v = (v_r + v_l) / 2
        omega = (v_r - v_l) / self.wheel_base

        # Midpoint integration
        theta_mid = self.theta + omega * dt / 2

        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta += omega * dt

        # Normalize orientation to [-pi, pi]
        self.theta = (
            self.theta + math.pi
        ) % (2 * math.pi) - math.pi

        return self.x, self.y, self.theta


if __name__ == "__main__":

    robot = ThymioSimulator()

    print("Initial pose:", (robot.x, robot.y, robot.theta))

    # Drive straight for 2 seconds
    for _ in range(20):
        robot.update(100, 100, dt=0.1)

    print("After driving forward:",
          (round(robot.x, 3),
           round(robot.y, 3),
           round(robot.theta, 3)))

    # Rotate left for 1 second
    for _ in range(10):
        robot.update(-80, 80, dt=0.1)

    print("After turning left:",
          (round(robot.x, 3),
           round(robot.y, 3),
           round(robot.theta, 3)))
