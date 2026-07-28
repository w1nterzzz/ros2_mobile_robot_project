# Copyright 2026
# Licensed under the Apache License, Version 2.0

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

XACRO_FILE = (
    Path(__file__).resolve().parents[1]
    / 'urdf'
    / 'mobile_robot.urdf.xacro'
)


BRIDGE_CONFIG_FILE = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'bridge.yaml'
)

WORLD_FILE = (
    Path(__file__).resolve().parents[3]
    / 'worlds'
    / 'empty_no_shadows.sdf'
)


@pytest.fixture(scope='module')
def robot():
    document = xacro.process_file(str(XACRO_FILE))
    return ET.fromstring(document.toxml())


@pytest.fixture(scope='module')
def world():
    document_text = WORLD_FILE.read_text(encoding='utf-8')
    sdf_start = document_text.index('<sdf version="')
    document = ET.fromstring(document_text[sdf_start:])
    return document.find('world')


@pytest.fixture(scope='module')
def bridge_config():
    with BRIDGE_CONFIG_FILE.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_world_loads_imu_system(world):
    assert world is not None

    imu_system = world.find(
        './plugin[@filename="gz-sim-imu-system"]'
    )

    assert imu_system is not None
    assert (
        imu_system.attrib['name']
        == 'gz::sim::systems::Imu'
    )


def test_imu_bridge_is_configured(bridge_config):
    imu_bridge = next(
        (
            entry
            for entry in bridge_config
            if entry.get('ros_topic_name') == '/imu/data'
        ),
        None,
    )

    assert imu_bridge is not None
    assert (
        imu_bridge['gz_topic_name']
        == '/model/mobile_robot/imu'
    )
    assert (
        imu_bridge['ros_type_name']
        == 'sensor_msgs/msg/Imu'
    )
    assert imu_bridge['gz_type_name'] == 'gz.msgs.IMU'
    assert imu_bridge['direction'] == 'GZ_TO_ROS'
    assert imu_bridge['qos_profile'] == 'SENSOR_DATA'
    assert imu_bridge.get('frame_id') == 'base_link'


def test_required_links_and_joints_exist(robot):
    link_names = {
        link.attrib['name']
        for link in robot.findall('link')
    }
    assert {
        'base_footprint',
        'base_link',
        'left_wheel_link',
        'right_wheel_link',
        'caster_link',
    }.issubset(link_names)

    joint_names = {
        joint.attrib['name']
        for joint in robot.findall('joint')
    }
    assert {
        'base_footprint_joint',
        'left_wheel_joint',
        'right_wheel_joint',
        'caster_joint',
    }.issubset(joint_names)


@pytest.mark.parametrize(
    ('joint_name', 'child_link', 'expected_y'),
    [
        ('left_wheel_joint', 'left_wheel_link', 0.17),
        ('right_wheel_joint', 'right_wheel_link', -0.17),
    ],
)
def test_wheel_joint_geometry(
    robot,
    joint_name,
    child_link,
    expected_y,
):
    joint = robot.find(f'./joint[@name="{joint_name}"]')

    assert joint is not None
    assert joint.attrib['type'] == 'continuous'
    assert joint.find('parent').attrib['link'] == 'base_link'
    assert joint.find('child').attrib['link'] == child_link
    assert joint.find('axis').attrib['xyz'] == '0 1 0'

    xyz = [
        float(value)
        for value in joint.find('origin').attrib['xyz'].split()
    ]

    assert xyz[0] == pytest.approx(0.08)
    assert xyz[1] == pytest.approx(expected_y)
    assert xyz[2] == pytest.approx(0.08)


@pytest.mark.parametrize(
    'link_name',
    [
        'base_link',
        'left_wheel_link',
        'right_wheel_link',
        'caster_link',
    ],
)
def test_physical_links_are_complete(robot, link_name):
    link = robot.find(f'./link[@name="{link_name}"]')

    assert link is not None
    assert link.find('visual') is not None
    assert link.find('collision') is not None
    assert link.find('inertial') is not None
    assert float(link.find('inertial/mass').attrib['value']) > 0.0
    assert link.find('inertial/inertia') is not None


def test_base_link_has_imu_sensor(robot):
    imu_sensor = robot.find(
        './gazebo[@reference="base_link"]'
        '/sensor[@name="imu_sensor"]'
    )

    assert imu_sensor is not None
    assert imu_sensor.attrib['type'] == 'imu'
    assert imu_sensor.findtext('always_on') == 'true'
    assert float(
        imu_sensor.findtext('update_rate')
    ) == pytest.approx(100.0)
    assert imu_sensor.findtext('visualize') == 'false'
    assert (
        imu_sensor.findtext('topic')
        == '/model/mobile_robot/imu'
    )


def test_gazebo_plugins_are_configured(robot):
    joint_state_plugin = robot.find(
        './gazebo/plugin'
        '[@filename="gz-sim-joint-state-publisher-system"]'
    )
    assert joint_state_plugin is not None
    assert (
        joint_state_plugin.findtext('topic')
        == '/model/mobile_robot/joint_state'
    )

    joint_names = {
        element.text
        for element in joint_state_plugin.findall('joint_name')
    }
    assert joint_names == {
        'left_wheel_joint',
        'right_wheel_joint',
    }

    diff_drive = robot.find(
        './gazebo/plugin'
        '[@filename="gz-sim-diff-drive-system"]'
    )
    assert diff_drive is not None
    assert diff_drive.findtext('left_joint') == 'left_wheel_joint'
    assert diff_drive.findtext('right_joint') == 'right_wheel_joint'
    assert float(
        diff_drive.findtext('wheel_separation')
    ) == pytest.approx(0.34)
    assert float(
        diff_drive.findtext('wheel_radius')
    ) == pytest.approx(0.08)
    assert (
        diff_drive.findtext('topic')
        == '/model/mobile_robot/cmd_vel'
    )
    assert (
        diff_drive.findtext('odom_topic')
        == '/model/mobile_robot/odometry'
    )
    assert diff_drive.findtext('frame_id') == 'odom'
    assert (
        diff_drive.findtext('child_frame_id')
        == 'base_footprint'
    )
