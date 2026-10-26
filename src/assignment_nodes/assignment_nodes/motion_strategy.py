
"""Motion strategy for moving Turtlesim toward a target pose."""

import math

from geometry_msgs.msg import Twist


class MotionStrategy:
    """Calculate velocity commands to reach a target pose."""

    POSITION_TOLERANCE = 0.03
    ANGLE_TOLERANCE = 0.03
    TURN_THRESHOLD = 0.15

    def __init__(self, max_linear_speed=2.0, max_angular_speed=4.0):
        """Initialize the maximum linear and angular speeds."""
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

    def set_speed_limits(self, max_linear_speed, max_angular_speed):
        """Update the speed limits while the node is running."""
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

    @staticmethod
    def normalize_angle(angle):
        """Return an angle in the range [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def calculate_command(self, pose, target):
        """Calculate velocity and report completion only near the target pose."""
        cmd = Twist()

        dx = target.x - pose.x
        dy = target.y - pose.y
        distance = math.hypot(dx, dy)

        yaw_error = self.normalize_angle(target.yaw - pose.theta)

        # A target is complete only when position and heading both match.
        if (
            distance <= self.POSITION_TOLERANCE
            and abs(yaw_error) <= self.ANGLE_TOLERANCE
        ):
            return cmd, distance, True

        # First align with the direction of travel.
        if distance > self.POSITION_TOLERANCE:
            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(
                target_angle - pose.theta
            )

            if abs(angle_error) > self.TURN_THRESHOLD:
                cmd.angular.z = max(
                    -self.max_angular_speed,
                    min(
                        self.max_angular_speed,
                        4.0 * angle_error
                    )
                )
            else:
                cmd.linear.x = min(
                    1.2 * distance,
                    self.max_linear_speed
                )
                cmd.angular.z = max(
                    -self.max_angular_speed,
                    min(
                        self.max_angular_speed,
                        2.0 * angle_error
                    )
                )
        else:
            # At the target position: rotate to the requested heading.
            cmd.angular.z = max(
                -self.max_angular_speed,
                min(
                    self.max_angular_speed,
                    4.0 * yaw_error
                )
            )

        return cmd, distance, False
