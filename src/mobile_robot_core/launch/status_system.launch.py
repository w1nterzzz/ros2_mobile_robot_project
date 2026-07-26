"""Launch the Stage 3 robot status system."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create the launch description for both status nodes."""
    return LaunchDescription([
        Node(
            package='mobile_robot_core',
            executable='basic_node',
            name='basic_node',
            output='screen',
            parameters=[{
                'robot_state': 'STANDBY',
            }],
        ),
        Node(
            package='mobile_robot_core',
            executable='status_monitor',
            name='status_monitor',
            output='screen',
        ),
    ])
