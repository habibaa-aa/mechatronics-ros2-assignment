
import math

from geometry_msgs.msg import Twist


class MotionStrategy:
    """Strategy for calculating velocity commands to reach a target pose."""

    def __init__(self, max_linear_speed=2.0, max_angular_speed=4.0):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

    def set_speed_limits(self, max_linear_speed, max_angular_speed):
        """Update the speed limits while the node is running."""
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

    def calculate_command(self, pose, target):
        """Calculate the next velocity command from the current and target poses."""
        dx = target.x - pose.x
        dy = target.y - pose.y
        distance = math.hypot(dx, dy)

        cmd = Twist()

        if distance >= 0.1:
            target_angle = math.atan2(dy, dx)
            angle_error = math.atan2(
                math.sin(target_angle - pose.theta),
                math.cos(target_angle - pose.theta)
            )

            if abs(angle_error) > 0.2:
                cmd.angular.z = max(
                    -self.max_angular_speed,
                    min(self.max_angular_speed, 6.0 * angle_error)
                )
            else:
                cmd.linear.x = min(
                    1.5 * distance, self.max_linear_speed
                )
                cmd.angular.z = max(
                    -self.max_angular_speed,
                    min(self.max_angular_speed, 3.0 * angle_error)
                )
        else:
            yaw_error = math.atan2(
                math.sin(target.yaw - pose.theta),
                math.cos(target.yaw - pose.theta)
            )

            reached = abs(yaw_error) < 0.05

            if not reached:
                cmd.angular.z = max(
                    -self.max_angular_speed,
                    min(self.max_angular_speed, 4.0 * yaw_error)
                )

            return cmd, distance, reached

        return cmd, distance, False