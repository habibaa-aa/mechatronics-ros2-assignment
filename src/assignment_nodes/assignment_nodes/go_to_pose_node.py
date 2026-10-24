
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from assignment_interfaces.action import GoToPose
from assignment_interfaces.srv import SetSpeed

class GoToPoseNode(Node):
    def __init__(self):
        super().__init__('go_to_pose_node')

        self.current_pose = None 
        
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 4.0)

        self.max_linear_speed = self.get_parameter(
            'max_linear_speed'
        )  .value

        self.max_angular_speed = self.get_parameter(
           'max_angular_speed'
        ).value

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
        self.current_pose = msg

    def execute_callback(self, goal_handle):
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

                    if abs(yaw_error) < 0.05:
                        break

                    cmd.angular.z = max(
                        -self.max_angular_speed,
                        min(self.max_angular_speed, 4.0 * yaw_error)
                    )       

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
        if request.max_linear_speed <= 0.0:
            response.success = False
            response.message = 'Linear speed must be greater than zero.'
            return response

        if request.max_angular_speed <= 0.0:
            response.success = False
            response.message = 'Angular speed must be greater than zero.'
            return response

        self.max_linear_speed = request.max_linear_speed
        self.max_angular_speed = request.max_angular_speed

        self.get_logger().info(
            f'Speed limits updated: linear={self.max_linear_speed}, '
            f'angular={self.max_angular_speed}'
        )

        response.success = True
        response.message = 'Speed limits updated successfully.'
        return response

def main(args=None):
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
