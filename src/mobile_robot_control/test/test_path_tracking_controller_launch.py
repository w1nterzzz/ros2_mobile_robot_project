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

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_CONFIG_FILE = (
    PACKAGE_ROOT
    / 'config'
    / 'path_tracking_controller.yaml'
)
CONTROLLER_LAUNCH_FILE = (
    PACKAGE_ROOT
    / 'launch'
    / 'path_tracking_controller.launch.py'
)


def test_controller_launch_declares_robot_runtime_arguments():
    module_spec = importlib.util.spec_from_file_location(
        'mobile_robot_path_tracking_controller_launch',
        CONTROLLER_LAUNCH_FILE,
    )
    assert module_spec is not None
    assert module_spec.loader is not None

    launch_module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(launch_module)
    launch_description = launch_module.generate_launch_description()

    declared_arguments = {
        action.name
        for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert {
        'robot_namespace',
        'use_sim_time',
    }.issubset(declared_arguments)


def test_controller_topics_are_relative():
    with CONTROLLER_CONFIG_FILE.open(
        encoding='utf-8',
    ) as stream:
        config = yaml.safe_load(stream)

    parameters = (
        config['path_tracking_controller']['ros__parameters']
    )

    assert parameters['odom_topic'] == 'odom'
    assert parameters['cmd_vel_topic'] == 'cmd_vel'


def test_cmd_vel_safety_launch_is_available():
    safety_launch_file = (
        CONTROLLER_LAUNCH_FILE.parent
        / 'cmd_vel_safety.launch.py'
    )

    assert safety_launch_file.is_file()

    module_spec = importlib.util.spec_from_file_location(
        'mobile_robot_cmd_vel_safety_launch',
        safety_launch_file,
    )
    assert module_spec is not None
    assert module_spec.loader is not None

    launch_module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(launch_module)
    launch_description = launch_module.generate_launch_description()

    declared_arguments = {
        action.name
        for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert {
        'command_timeout',
        'robot_namespace',
    }.issubset(declared_arguments)
