"""Minimal ROS 2 node used for Stage 3 lessons."""

from mobile_robot_interfaces.msg import RobotStatus
import rclpy
from rclpy.node import Node


class BasicNode(Node):
    """Publish the robot status at a fixed interval."""

    def __init__(self):
        super().__init__('basic_node')

        self.status_publisher = self.create_publisher(
            RobotStatus,
            'robot_status',
            10,
        )
        self.sequence = 0
        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Basic node has started.')

    def publish_status(self):
        """Create and publish one robot status message."""
        self.sequence += 1

        message = RobotStatus()
        message.sequence = self.sequence
        message.state = 'READY'

        self.status_publisher.publish(message)
        self.get_logger().info(
            f'Published robot status: sequence={message.sequence}, '
            f'state={message.state}'
        )


def main(args=None):
    """Initialize ROS 2, run the node, and shut it down."""
    rclpy.init(args=args)
    node = BasicNode()

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
