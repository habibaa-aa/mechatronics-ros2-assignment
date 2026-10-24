
"""Service node for resetting the Turtlesim turtle."""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute


class ResetNode(Node):
    """Provide a service to reset the turtle's drawing and position."""

    def __init__(self):
        """Initialize the reset and teleport clients and service."""
        super().__init__('reset_node')

        self.reset_client = self.create_client(Empty, '/reset')
        self.teleport_client = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute'
        )

        self.reset_service = self.create_service(
            Empty, '/reset_turtle', self.reset_callback
        )

        self.get_logger().info('Reset service is ready.')

    def reset_callback(self, request, response):
        """Clear the drawing and begin resetting the turtle."""
        self.get_logger().info('Reset requested.')

        if not self.reset_client.service_is_ready():
            self.get_logger().error('/reset service is unavailable.')
            return response

        future = self.reset_client.call_async(Empty.Request())
        future.add_done_callback(self.reset_finished)

        return response

    def reset_finished(self, future):
        """Handle the result of clearing the Turtlesim drawing."""
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(f'Reset failed: {error}')
            return

        if result is None:
            self.get_logger().error('Failed to reset turtlesim.')
            return

        if not self.teleport_client.service_is_ready():
            self.get_logger().error(
                'Teleport service is unavailable.'
            )
            return

        request = TeleportAbsolute.Request()
        request.x = 5.544
        request.y = 5.544
        request.theta = 0.0

        future = self.teleport_client.call_async(request)
        future.add_done_callback(self.teleport_finished)

    def teleport_finished(self, future):
        """Handle the result of teleporting the turtle to its start."""
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(f'Teleport failed: {error}')
            return

        if result is not None:
            self.get_logger().info(
                'Turtle reset to the starting position.'
            )
        else:
            self.get_logger().error(
                'Failed to return turtle to start.'
            )


def main(args=None):
    """Initialize and run the reset node."""
    rclpy.init(args=args)
    node = ResetNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
