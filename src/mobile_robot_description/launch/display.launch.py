from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    frame_prefix = LaunchConfiguration('frame_prefix')
    robot_name = LaunchConfiguration('robot_name')
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_joint_state_publisher = LaunchConfiguration(
        'use_joint_state_publisher'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('mobile_robot_description'),
        'urdf',
        'mobile_robot.urdf.xacro',
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' ',
            'robot_name:=',
            robot_name,
            ' ',
            'frame_prefix:=',
            frame_prefix,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='Prefix applied to this robot TF frame identifiers',
        ),

        DeclareLaunchArgument(
            'robot_name',
            default_value='mobile_robot',
            description='Unique Gazebo model name',
        ),

        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='ROS namespace for this robot',
        ),

        DeclareLaunchArgument(
            'use_joint_state_publisher',
            default_value='true',
            description=(
                'Start joint_state_publisher for standalone '
                'URDF and RViz visualization'
            ),
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the ROS simulation clock',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_namespace,
            output='screen',
            parameters=[{
                'frame_prefix': ParameterValue(
                    frame_prefix,
                    value_type=str,
                ),
                'robot_description': robot_description,
                'use_sim_time': ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=robot_namespace,
            output='screen',
            condition=IfCondition(use_joint_state_publisher),
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            }],
        ),
    ])
