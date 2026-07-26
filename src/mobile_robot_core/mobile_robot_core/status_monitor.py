"""Monitor robot status messages for Stage 3."""

from mobile_robot_interfaces.msg import RobotStatus
import rclpy
from rclpy.node import Node


class StatusMonitor(Node):
    """Subscribe to and report robot status messages."""

    def __init__(self):
        super().__init__('status_monitor')

        self.status_subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_callback,
            10,
        )

        self.get_logger().info('Status monitor has started.')

    def status_callback(self, message):
        """Report each received robot status message."""
        self.get_logger().info(
            f'Received robot status: sequence={message.sequence}, '
            f'state={message.state}'
        )


def main(args=None):
    """Initialize ROS 2, run the node, and shut it down."""
    rclpy.init(args=args)
    node = StatusMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
