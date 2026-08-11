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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    command_timeout = LaunchConfiguration('command_timeout')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    publish_rate = LaunchConfiguration('publish_rate')
    robot_namespace = LaunchConfiguration('robot_namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'command_timeout',
            default_value='0.5',
            description='Maximum command age in seconds',
        ),

        DeclareLaunchArgument(
            'input_topic',
            default_value='cmd_vel',
            description='Relative requested velocity topic',
        ),

        DeclareLaunchArgument(
            'output_topic',
            default_value='actuator/cmd_vel',
            description='Relative safety-filtered velocity topic',
        ),

        DeclareLaunchArgument(
            'publish_rate',
            default_value='20.0',
            description='Safety output frequency in hertz',
        ),

        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='ROS namespace for this robot',
        ),

        Node(
            package='mobile_robot_control',
            executable='cmd_vel_safety',
            name='cmd_vel_safety',
            namespace=robot_namespace,
            output='screen',
            parameters=[{
                'command_timeout': ParameterValue(
                    command_timeout,
                    value_type=float,
                ),
                'input_topic': ParameterValue(
                    input_topic,
                    value_type=str,
                ),
                'output_topic': ParameterValue(
                    output_topic,
                    value_type=str,
                ),
                'publish_rate': ParameterValue(
                    publish_rate,
                    value_type=float,
                ),
            }],
        ),
    ])
