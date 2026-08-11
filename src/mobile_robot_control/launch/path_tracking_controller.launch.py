# Copyright 2026 ros2_mobile_robot_project contributors
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
    default_parameters_file = PathJoinSubstitution([
        FindPackageShare('mobile_robot_control'),
        'config',
        'path_tracking_controller.yaml',
    ])

    parameters_file = LaunchConfiguration('parameters_file')
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'parameters_file',
            default_value=default_parameters_file,
            description='Path to the controller parameter file',
        ),

        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='ROS namespace for this robot',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the ROS simulation clock',
        ),

        Node(
            package='mobile_robot_control',
            executable='path_tracking_controller',
            name='path_tracking_controller',
            namespace=robot_namespace,
            output='screen',
            parameters=[
                parameters_file,
                {
                    'use_sim_time': ParameterValue(
                        use_sim_time,
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
