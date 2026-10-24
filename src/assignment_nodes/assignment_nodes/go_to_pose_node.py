"""Action server for moving the Turtlesim turtle to a target pose."""

import time

from assignment_interfaces.action import GoToPose
from assignment_interfaces.srv import SetSpeed
from assignment_nodes.motion_strategy import MotionStrategy
from geometry_msgs.msg import Twist

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.msg import Pose


class GoToPoseNode(Node):
    """Node providing the goToPose action and speed service."""

    def __init__(self):
        """Initialize publishers, subscribers, and servers."""
        super().__init__('go_to_pose_node')

        self.current_pose = None

        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 4.0)

        self.max_linear_speed = self.get_parameter(
            'max_linear_speed'
        ).value

        self.max_angular_speed = self.get_parameter(
            'max_angular_speed'
        ).value

        self.motion_strategy = MotionStrategy(
            self.max_linear_speed,
            self.max_angular_speed
        )

        self.speed_service = self.create_service(
            SetSpeed,
            '/set_speed',
            self.set_speed_callback
        )

        self.pose_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )

        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.pose_callback_group
        )

        self.action_server = ActionServer(
            self,
            GoToPose,
            'goToPose',
            self.execute_callback,
            callback_group=self.action_callback_group
        )

        self.get_logger().info('goToPose action server is ready.')

    def pose_callback(self, msg):
        """Store the turtle's latest pose."""
        self.current_pose = msg

    def execute_callback(self, goal_handle):
        """Move the turtle toward the requested target pose."""
        self.get_logger().info('Received a goToPose goal.')

        target = goal_handle.request.target
        feedback = GoToPose.Feedback()
        result = GoToPose.Result()

        try:
            while rclpy.ok() and self.current_pose is None:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal canceled.'
                    return result

                time.sleep(0.05)

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal canceled.'
                    return result

                pose = self.current_pose

                # Read the latest speed parameters on every control cycle.
                self.max_linear_speed = self.get_parameter(
                    'max_linear_speed'
                ).value
                self.max_angular_speed = self.get_parameter(
                    'max_angular_speed'
                ).value

                # Apply the latest limits to the motion strategy.
                self.motion_strategy.set_speed_limits(
                    self.max_linear_speed,
                    self.max_angular_speed
                )

                cmd, distance, reached = (
                    self.motion_strategy.calculate_command(pose, target)
                )

                if reached:
                    break

                feedback.distance_remaining = distance
                goal_handle.publish_feedback(feedback)
                self.cmd_vel_publisher.publish(cmd)

                time.sleep(0.05)

            self.cmd_vel_publisher.publish(Twist())
            goal_handle.succeed()

            result.success = True
            result.message = (
                'Reached the target position and orientation.'
            )
            self.get_logger().info(result.message)
            return result

        finally:
            self.cmd_vel_publisher.publish(Twist())

    def set_speed_callback(self, request, response):
        """Validate and update the linear and angular speed limits."""
        if request.max_linear_speed <= 0.0:
            response.success = False
            response.message = 'Linear speed must be greater than zero.'
            return response

        if request.max_angular_speed <= 0.0:
            response.success = False
            response.message = 'Angular speed must be greater than zero.'
            return response

        # Update ROS parameters as well as the internal speed limits.
        parameter_results = self.set_parameters([
            Parameter(
                'max_linear_speed',
                value=request.max_linear_speed
            ),
            Parameter(
                'max_angular_speed',
                value=request.max_angular_speed
            )
        ])

        if not all(result.successful for result in parameter_results):
            response.success = False
            response.message = 'Failed to update speed parameters.'
            return response

        self.max_linear_speed = request.max_linear_speed
        self.max_angular_speed = request.max_angular_speed

        self.motion_strategy.set_speed_limits(
            self.max_linear_speed,
            self.max_angular_speed
        )

        self.get_logger().info(
            f'Speed limits updated: linear={self.max_linear_speed}, '
            f'angular={self.max_angular_speed}'
        )

        response.success = True
        response.message = 'Speed limits updated successfully.'
        return response


def main(args=None):
    """Start the movement node and its executor."""
    rclpy.init(args=args)
    node = GoToPoseNode()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_publisher.publish(Twist())
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
