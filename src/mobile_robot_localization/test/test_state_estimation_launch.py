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

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
STATE_ESTIMATOR_CONFIG_FILE = (
    PACKAGE_ROOT
    / 'config'
    / 'state_estimator.yaml'
)
STATE_ESTIMATION_LAUNCH_FILE = (
    PACKAGE_ROOT
    / 'launch'
    / 'state_estimation.launch.py'
)


def test_state_estimation_launch_declares_robot_identity_arguments():
    module_spec = importlib.util.spec_from_file_location(
        'mobile_robot_state_estimation_launch',
        STATE_ESTIMATION_LAUNCH_FILE,
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
        'frame_prefix',
        'robot_namespace',
    }.issubset(declared_arguments)


def test_state_estimator_topics_are_relative():
    with STATE_ESTIMATOR_CONFIG_FILE.open(
        encoding='utf-8',
    ) as stream:
        config = yaml.safe_load(stream)

    parameters = config['state_estimator']['ros__parameters']

    assert parameters['imu_topic'] == 'imu/data'
    assert parameters['wheel_odom_topic'] == 'wheel/odom'
    assert parameters['odom_topic'] == 'odom'
