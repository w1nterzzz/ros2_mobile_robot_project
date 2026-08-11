from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _spawn_robot_arguments(
    robot_name,
    spawn_x,
    spawn_y,
    spawn_z,
    spawn_yaw,
):
    return [
        '-world',
        'empty',
        '-topic',
        'robot_description',
        '-name',
        robot_name,
        '-x',
        spawn_x,
        '-y',
        spawn_y,
        '-z',
        spawn_z,
        '-Y',
        spawn_yaw,
    ]


def generate_launch_description():
    bridge_clock = LaunchConfiguration('bridge_clock')
    frame_prefix = LaunchConfiguration('frame_prefix')
    robot_name = LaunchConfiguration('robot_name')
    robot_namespace = LaunchConfiguration('robot_namespace')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    package_share = FindPackageShare('mobile_robot_description')

    localization_package_share = FindPackageShare(
        'mobile_robot_localization'
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                package_share,
                'launch',
                'display.launch.py',
            ])
        ),
        launch_arguments={
            'frame_prefix': frame_prefix,
            'robot_name': robot_name,
            'robot_namespace': robot_namespace,
            'use_joint_state_publisher': 'false',
            'use_sim_time': 'true',
        }.items(),
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                package_share,
                'launch',
                'bridge.launch.py',
            ])
        ),
        launch_arguments={
            'bridge_clock': bridge_clock,
            'frame_prefix': frame_prefix,
            'robot_name': robot_name,
            'robot_namespace': robot_namespace,
        }.items(),
    )

    state_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                localization_package_share,
                'launch',
                'state_estimation.launch.py',
            ])
        ),
        launch_arguments={
            'frame_prefix': frame_prefix,
            'robot_namespace': robot_namespace,
            'use_sim_time': 'true',
        }.items(),
    )
    spawn_robot = Node(
        namespace=robot_namespace,
        package='ros_gz_sim',
        executable='create',
        name='spawn_mobile_robot',
        output='screen',
        arguments=_spawn_robot_arguments(
            robot_name,
            spawn_x,
            spawn_y,
            spawn_z,
            spawn_yaw,
        ),
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_clock',
            default_value='true',
            description='Start the shared Gazebo clock bridge',
        ),

        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='Prefix applied to this robot TF frames',
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
            'spawn_x',
            default_value='0.0',
            description='Initial X position in meters',
        ),

        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Initial Y position in meters',
        ),

        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.02',
            description='Initial Z position in meters',
        ),

        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='Initial yaw angle in radians',
        ),

        display_launch,
        bridge_launch,
        state_estimation_launch,
        delayed_spawn,
    ])
