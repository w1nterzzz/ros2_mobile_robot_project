# Copyright 2026 Yang Yuexiang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _robot_simulation(
    package_share,
    robot_name,
    spawn_x,
    spawn_y,
    spawn_z,
    spawn_yaw,
    bridge_clock,
):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                package_share,
                'launch',
                'simulation.launch.py',
            ])
        ),
        launch_arguments={
            'bridge_clock': bridge_clock,
            'frame_prefix': f'{robot_name}/',
            'robot_name': robot_name,
            'robot_namespace': robot_name,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
        }.items(),
    )


def generate_launch_description():
    package_share = FindPackageShare('mobile_robot_description')

    robot_1_spawn_x = LaunchConfiguration('robot_1_spawn_x')
    robot_1_spawn_y = LaunchConfiguration('robot_1_spawn_y')
    robot_1_spawn_yaw = LaunchConfiguration('robot_1_spawn_yaw')

    robot_2_spawn_x = LaunchConfiguration('robot_2_spawn_x')
    robot_2_spawn_y = LaunchConfiguration('robot_2_spawn_y')
    robot_2_spawn_yaw = LaunchConfiguration('robot_2_spawn_yaw')

    spawn_z = LaunchConfiguration('spawn_z')

    robot_1_launch = _robot_simulation(
        package_share,
        'robot_1',
        robot_1_spawn_x,
        robot_1_spawn_y,
        spawn_z,
        robot_1_spawn_yaw,
        'true',
    )

    robot_2_launch = _robot_simulation(
        package_share,
        'robot_2',
        robot_2_spawn_x,
        robot_2_spawn_y,
        spawn_z,
        robot_2_spawn_yaw,
        'false',
    )

    delayed_robot_2_launch = TimerAction(
        period=0.5,
        actions=[robot_2_launch],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_1_spawn_x',
            default_value='-0.75',
            description='Initial robot_1 X position in meters',
        ),

        DeclareLaunchArgument(
            'robot_1_spawn_y',
            default_value='0.0',
            description='Initial robot_1 Y position in meters',
        ),

        DeclareLaunchArgument(
            'robot_1_spawn_yaw',
            default_value='0.0',
            description='Initial robot_1 yaw angle in radians',
        ),

        DeclareLaunchArgument(
            'robot_2_spawn_x',
            default_value='0.75',
            description='Initial robot_2 X position in meters',
        ),

        DeclareLaunchArgument(
            'robot_2_spawn_y',
            default_value='0.0',
            description='Initial robot_2 Y position in meters',
        ),

        DeclareLaunchArgument(
            'robot_2_spawn_yaw',
            default_value='0.0',
            description='Initial robot_2 yaw angle in radians',
        ),

        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.02',
            description='Initial robot height in meters',
        ),

        robot_1_launch,
        delayed_robot_2_launch,
    ])
