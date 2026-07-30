from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('mobile_robot_description')

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                package_share,
                'launch',
                'display.launch.py',
            ])
        ),
        launch_arguments={
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
    )

    state_estimator = Node(
        package='mobile_robot_localization',
        executable='state_estimator',
        name='state_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_mobile_robot',
        output='screen',
        arguments=[
            '-world',
            'empty',
            '-topic',
            '/robot_description',
            '-name',
            'mobile_robot',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.02',
        ],
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot],
    )

    return LaunchDescription([
        display_launch,
        bridge_launch,
        state_estimator,
        delayed_spawn,
    ])
