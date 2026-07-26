from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_config = PathJoinSubstitution([
        FindPackageShare('mobile_robot_description'),
        'config',
        'bridge.yaml',
    ])

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{
                'config_file': bridge_config,
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_retimestamp',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'source_list': ['/joint_states_raw'],
                'publish_default_positions': True,
                'publish_default_velocities': False,
                'publish_default_efforts': False,
                'rate': 50,
            }],
        ),
    ])
