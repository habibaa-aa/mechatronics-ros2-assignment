
"""Action server for drawing squares with Turtlesim."""

import math
import threading
import time

from assignment_interfaces.action import DrawSquare, GoToPose
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.msg import Pose


class DrawSquareNode(Node):
    """Draw a square using fixed corner targets."""

    def __init__(self):
        """Initialize clients, subscriber, and action server."""
        super().__init__('draw_square_node')

        self.callback_group = ReentrantCallbackGroup()
        self.current_pose = None
        self.pose_count = 0

        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )

        self.reset_client = self.create_client(
            Empty,
            '/reset_turtle',
            callback_group=self.callback_group
        )

        self.go_to_pose_client = ActionClient(
            self,
            GoToPose,
            '/goToPose',
            callback_group=self.callback_group
        )

        self.square_action_server = ActionServer(
            self,
            DrawSquare,
            '/draw_square',
            self.execute_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('DrawSquare action server is ready.')

    def pose_callback(self, msg):
        """Store each newly received turtle pose."""
        self.current_pose = msg
        self.pose_count += 1

    def wait_for_new_pose(self, previous_count, timeout=3.0):
        """Wait until a newer turtle pose arrives."""
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            if self.pose_count > previous_count:
                return self.current_pose

            time.sleep(0.02)

        return None

    def wait_for_target_pose(self, target, timeout=3.0):
        """Wait until the measured pose is close to the target."""
        deadline = time.monotonic() + timeout
        previous_count = self.pose_count

        while rclpy.ok() and time.monotonic() < deadline:
            pose = self.wait_for_new_pose(
                previous_count,
                timeout=0.2
            )

            if pose is None:
                continue

            previous_count = self.pose_count

            distance = math.hypot(
                target.x - pose.x,
                target.y - pose.y
            )

            # Turtlesim Pose uses theta, not yaw.
            yaw_error = math.atan2(
                math.sin(target.theta - pose.theta),
                math.cos(target.theta - pose.theta)
            )

            if distance <= 0.06 and abs(yaw_error) <= 0.06:
                return pose

        return None

    @staticmethod
    def wait_for_future(future, timeout=30.0):
        """Wait for an asynchronous operation to finish."""
        finished = threading.Event()
        future.add_done_callback(lambda _: finished.set())
        return finished.wait(timeout)

    def execute_callback(self, goal_handle):
        """Reset the turtle and draw four geometrically fixed sides."""
        result = DrawSquare.Result()
        feedback = DrawSquare.Feedback()
        side_length = goal_handle.request.side_length

        if not math.isfinite(side_length) or side_length <= 0.0:
            goal_handle.abort()
            result.success = False
            result.message = (
                'Side length must be a finite positive number.'
            )
            return result

        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            goal_handle.abort()
            result.success = False
            result.message = 'Reset service is unavailable.'
            return result

        if not self.go_to_pose_client.wait_for_server(
            timeout_sec=5.0
        ):
            goal_handle.abort()
            result.success = False
            result.message = 'GoToPose action server is unavailable.'
            return result

        pose_count_before_reset = self.pose_count
        reset_future = self.reset_client.call_async(Empty.Request())

        if not self.wait_for_future(reset_future, timeout=10.0):
            goal_handle.abort()
            result.success = False
            result.message = 'Reset service timed out.'
            return result

        try:
            reset_future.result()
        except Exception as error:
            goal_handle.abort()
            result.success = False
            result.message = f'Reset failed: {error}'
            return result

        start_pose = self.wait_for_new_pose(
            pose_count_before_reset,
            timeout=3.0
        )

        if start_pose is None:
            goal_handle.abort()
            result.success = False
            result.message = 'No fresh turtle pose received after reset.'
            return result

        start_x = start_pose.x
        start_y = start_pose.y
        start_yaw = start_pose.theta

        # Calculate every corner from the same starting pose.
        unit_x = math.cos(start_yaw)
        unit_y = math.sin(start_yaw)

        perpendicular_x = math.cos(start_yaw + math.pi / 2.0)
        perpendicular_y = math.sin(start_yaw + math.pi / 2.0)

        corners = [
            (
                start_x + side_length * unit_x,
                start_y + side_length * unit_y
            ),
            (
                start_x + side_length * (unit_x + perpendicular_x),
                start_y + side_length * (unit_y + perpendicular_y)
            ),
            (
                start_x + side_length * perpendicular_x,
                start_y + side_length * perpendicular_y
            ),
            (start_x, start_y)
        ]

        targets = []

        for side, (target_x, target_y) in enumerate(corners):
            target_yaw = start_yaw + (side + 1) * math.pi / 2.0
            targets.append((target_x, target_y, target_yaw))

        # Check that all corners stay inside the safe canvas area.
        margin = 0.5

        for target_x, target_y, _ in targets:
            if not (
                margin <= target_x <= 11.0 - margin
                and margin <= target_y <= 11.0 - margin
            ):
                goal_handle.abort()
                result.success = False
                result.message = (
                    'Square is too large for the safe canvas area.'
                )
                return result

        # Move to each corner in sequence.
        for side, (target_x, target_y, target_yaw) in enumerate(
            targets
        ):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Square drawing canceled.'
                return result

            feedback.current_side = side + 1
            goal_handle.publish_feedback(feedback)

            goal = GoToPose.Goal()
            goal.target.x = target_x
            goal.target.y = target_y
            goal.target.yaw = target_yaw

            goal_future = self.go_to_pose_client.send_goal_async(
                goal
            )

            if not self.wait_for_future(goal_future, timeout=10.0):
                goal_handle.abort()
                result.success = False
                result.message = (
                    f'Timed out starting side {side + 1}.'
                )
                return result

            movement_handle = goal_future.result()

            if (
                movement_handle is None
                or not movement_handle.accepted
            ):
                goal_handle.abort()
                result.success = False
                result.message = f'Side {side + 1} was rejected.'
                return result

            result_future = movement_handle.get_result_async()

            if not self.wait_for_future(
                result_future,
                timeout=45.0
            ):
                movement_handle.cancel_goal_async()
                goal_handle.abort()
                result.success = False
                result.message = f'Side {side + 1} timed out.'
                return result

            movement_result = result_future.result()

            if (
                movement_result is None
                or not movement_result.result.success
            ):
                goal_handle.abort()
                result.success = False
                result.message = (
                    f'Side {side + 1} failed to complete.'
                )
                return result

            # Verify the actual measured position and orientation.
            target_pose = Pose()
            target_pose.x = target_x
            target_pose.y = target_y
            target_pose.theta = target_yaw

            measured_pose = self.wait_for_target_pose(
                target_pose,
                timeout=3.0
            )

            if measured_pose is None:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f'Turtle did not settle at corner {side + 1}.'
                )
                return result

        result.success = True
        result.message = 'Square drawn successfully.'
        goal_handle.succeed()

        self.get_logger().info(result.message)
        return result


def main(args=None):
    """Start the square-drawing node and its executor."""
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
