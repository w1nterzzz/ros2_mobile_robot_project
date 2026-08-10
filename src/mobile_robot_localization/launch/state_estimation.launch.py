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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mobile_robot_localization'),
            'config',
            'state_estimator.yaml',
        ]),
        description='Path to the state estimator parameter file',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the ROS simulation clock',
    )

    state_estimator = Node(
        package='mobile_robot_localization',
        executable='state_estimator',
        name='state_estimator',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription([
        declare_config_file,
        declare_use_sim_time,
        state_estimator,
    ])
