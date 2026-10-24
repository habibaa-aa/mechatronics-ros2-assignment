
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from assignment_interfaces.action import DrawSquare, GoToPose
from std_srvs.srv import Empty


class DrawSquareNode(Node):
    def __init__(self):
        super().__init__('draw_square_node')

        self.callback_group = ReentrantCallbackGroup()

        # Client for resetting the turtle
        self.reset_client = self.create_client(
            Empty,
            '/reset_turtle',
            callback_group=self.callback_group
        )

        # Client for asking the turtle to move to a target pose
        self.go_to_pose_client = ActionClient(
            self,
            GoToPose,
            '/goToPose',
            callback_group=self.callback_group
        )

        # Server that accepts square-drawing requests
        self.square_action_server = ActionServer(
            self,
            DrawSquare,
            '/draw_square',
            self.execute_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('DrawSquare action server is ready.')
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Received a DrawSquare goal.')

        side_length = goal_handle.request.side_length
        feedback = DrawSquare.Feedback()
        result = DrawSquare.Result()

        if side_length <= 0.0:
            goal_handle.abort()
            result.success = False
            result.message = 'Side length must be greater than zero.'
            return result

        # Wait for the reset service and movement action
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            goal_handle.abort()
            result.success = False
            result.message = 'Reset service is unavailable.'
            return result

        if not self.go_to_pose_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result.success = False
            result.message = 'GoToPose action server is unavailable.'
            return result

        # Reset the turtle and clear its drawing
        reset_future = self.reset_client.call_async(Empty.Request())
        while rclpy.ok() and not reset_future.done():
            time.sleep(0.05)

        if reset_future.result() is None:
            goal_handle.abort()
            result.success = False
            result.message = 'Failed to reset the turtle.'
            return result

        # Read the starting pose after the reset
        pose = self.get_turtle_pose()
        if pose is None:
            goal_handle.abort()
            result.success = False
            result.message = 'Could not read the turtle pose.'
            return result
        
        current_x = pose.x
        current_y = pose.y
        current_yaw = pose.theta

        # Draw four sides, turning 90 degrees after each side
        for side in range(4):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Square drawing canceled.'
                return result

            feedback.current_side = side + 1
            goal_handle.publish_feedback(feedback)

            target_x = current_x + side_length * math.cos(current_yaw)
            target_y = current_y + side_length * math.sin(current_yaw)
            target_yaw = current_yaw + math.pi / 2.0

            # Keep the target inside the turtlesim canvas
            target_x = max(0.5, min(10.5, target_x))
            target_y = max(0.5, min(10.5, target_y))

            goal = GoToPose.Goal()
            goal.target.x = target_x
            goal.target.y = target_y
            goal.target.yaw = target_yaw

            goal_future = self.go_to_pose_client.send_goal_async(goal)
            while rclpy.ok() and not goal_future.done():
                time.sleep(0.05)

            goal_handle_pose = goal_future.result()
            if goal_handle_pose is None or not goal_handle_pose.accepted:
                goal_handle.abort()
                result.success = False
                result.message = f'Failed to start side {side + 1}.'
                return result

            result_future = goal_handle_pose.get_result_async()
            while rclpy.ok() and not result_future.done():
                time.sleep(0.05)

            movement_result = result_future.result()
            if movement_result is None or not movement_result.result.success:
                goal_handle.abort()
                result.success = False
                result.message = f'Failed to complete side {side + 1}.'
                return result
            
            current_x = target_x
            current_y = target_y
            current_yaw = target_yaw
            
        result.success = True
        result.message = 'Square drawn successfully.'
        goal_handle.succeed()
        self.get_logger().info(result.message)
        return result

    def get_turtle_pose(self):
        from turtlesim.msg import Pose

        # Subscribe temporarily and wait for a pose update
        pose_holder = {'pose': None}

        subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            lambda msg: pose_holder.update({'pose': msg}),
            10,
            callback_group=self.callback_group
        )

        deadline = time.monotonic() + 3.0
        while rclpy.ok() and pose_holder['pose'] is None:
            if time.monotonic() >= deadline:
                break
            time.sleep(0.05)

        self.destroy_subscription(subscription)
        return pose_holder['pose']


def main(args=None):
    rclpy.init(args=args)
    node = DrawSquareNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
