
"""Service node for resetting the Turtlesim turtle."""

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute


class ResetNode(Node):
    """Provide a service for clearing and resetting the turtle."""

    def __init__(self):
        """Initialize the reset and teleport clients."""
        super().__init__('reset_node')

        self.callback_group = ReentrantCallbackGroup()

        self.reset_client = self.create_client(
            Empty,
            '/reset',
            callback_group=self.callback_group
        )

        self.teleport_client = self.create_client(
            TeleportAbsolute,
            '/turtle1/teleport_absolute',
            callback_group=self.callback_group
        )

        self.reset_service = self.create_service(
            Empty,
            '/reset_turtle',
            self.reset_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Reset service is ready.')

    def reset_callback(self, request, response):
        """Clear the drawing and wait for the turtle to reset."""
        del request
        self.get_logger().info('Reset requested.')

        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/reset service is unavailable.')
            return response

        if not self.teleport_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/turtle1/teleport_absolute service is unavailable.'
            )
            return response

        reset_future = self.reset_client.call_async(Empty.Request())

        if not self.wait_for_future(reset_future, timeout_sec=5.0):
            self.get_logger().error('Failed to clear the drawing.')
            return response

        try:
            if reset_future.result() is None:
                self.get_logger().error('Failed to clear the drawing.')
                return response
        except Exception as error:
            self.get_logger().error(f'Reset failed: {error}')
            return response

        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = 5.544
        teleport_request.y = 5.544
        teleport_request.theta = 0.0

        teleport_future = self.teleport_client.call_async(
            teleport_request
        )

        if not self.wait_for_future(teleport_future, timeout_sec=5.0):
            self.get_logger().error('Teleport timed out.')
            return response

        try:
            if teleport_future.result() is None:
                self.get_logger().error('Failed to teleport the turtle.')
                return response
        except Exception as error:
            self.get_logger().error(f'Teleport failed: {error}')
            return response

        self.get_logger().info('Turtle reset completed successfully.')
        return response

    @staticmethod
    def wait_for_future(future, timeout_sec):
        """Wait for an asynchronous service call without blocking callbacks."""
        finished = threading.Event()
        future.add_done_callback(lambda _: finished.set())
        return finished.wait(timeout_sec)


def main(args=None):
    """Initialize and run the reset node."""
    rclpy.init(args=args)
    node = ResetNode()

    executor = MultiThreadedExecutor(num_threads=2)
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
