"""Minimal ROS 2 node used for Stage 3 lessons."""

from mobile_robot_interfaces.msg import RobotStatus
from mobile_robot_interfaces.srv import SetRobotState
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class BasicNode(Node):
    """Publish robot status and provide a state-setting service."""

    def __init__(self):
        super().__init__('basic_node')

        self.declare_parameter('robot_state', 'READY')
        self.sequence = 0

        self.status_publisher = self.create_publisher(
            RobotStatus,
            'robot_status',
            10,
        )
        self.timer = self.create_timer(1.0, self.publish_status)

        self.set_state_service = self.create_service(
            SetRobotState,
            'set_robot_state',
            self.set_robot_state_callback,
        )

        self.get_logger().info('Basic node has started.')

    def publish_status(self):
        """Create and publish one robot status message."""
        self.sequence += 1

        message = RobotStatus()
        message.sequence = self.sequence
        message.state = self.get_parameter('robot_state').value

        self.status_publisher.publish(message)
        self.get_logger().info(
            f'Published robot status: sequence={message.sequence}, '
            f'state={message.state}'
        )

    def set_robot_state_callback(self, request, response):
        """Validate a requested state and return the result."""
        requested_state = request.state.strip().upper()

        if not requested_state:
            response.success = False
            response.message = 'State must not be empty.'
            return response

        self.set_parameters([
            Parameter('robot_state', value=requested_state),
        ])
        response.success = True
        response.message = f'State changed to {requested_state}.'
        self.get_logger().info(response.message)
        return response


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
