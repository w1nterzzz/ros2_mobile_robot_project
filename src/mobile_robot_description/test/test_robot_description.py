# Copyright 2026
# Licensed under the Apache License, Version 2.0

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch.actions import DeclareLaunchArgument
import pytest
import xacro

XACRO_FILE = (
    Path(__file__).resolve().parents[1]
    / 'urdf'
    / 'mobile_robot.urdf.xacro'
)

DISPLAY_LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / 'launch'
    / 'display.launch.py'
)

BRIDGE_LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / 'launch'
    / 'bridge.launch.py'
)

WORLD_FILE = (
    Path(__file__).resolve().parents[3]
    / 'worlds'
    / 'empty_no_shadows.sdf'
)


def test_display_launch_declares_robot_identity_arguments():
    module_spec = importlib.util.spec_from_file_location(
        'mobile_robot_display_launch',
        DISPLAY_LAUNCH_FILE,
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
        'robot_name',
        'robot_namespace',
    }.issubset(declared_arguments)


def test_bridge_launch_declares_robot_identity_arguments():
    module_spec = importlib.util.spec_from_file_location(
        'mobile_robot_bridge_launch',
        BRIDGE_LAUNCH_FILE,
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
        'bridge_clock',
        'frame_prefix',
        'robot_name',
        'robot_namespace',
    }.issubset(declared_arguments)


@pytest.fixture(scope='module')
def robot():
    document = xacro.process_file(str(XACRO_FILE))
    return ET.fromstring(document.toxml())


@pytest.fixture(scope='module')
def custom_named_robot():
    document = xacro.process_file(
        str(XACRO_FILE),
        mappings={'robot_name': 'robot_1'},
    )
    return ET.fromstring(document.toxml())


@pytest.fixture(scope='module')
def prefixed_robot():
    document = xacro.process_file(
        str(XACRO_FILE),
        mappings={'frame_prefix': 'robot_1/'},
    )
    return ET.fromstring(document.toxml())


@pytest.fixture(scope='module')
def world():
    document_text = WORLD_FILE.read_text(encoding='utf-8')
    sdf_start = document_text.index('<sdf version="')
    document = ET.fromstring(document_text[sdf_start:])
    return document.find('world')


def test_custom_robot_name_parameterizes_gazebo_topics(
    custom_named_robot,
):

    imu_sensor = custom_named_robot.find(
        './gazebo[@reference="base_link"]'
        '/sensor[@name="imu_sensor"]'
    )
    assert imu_sensor is not None
    assert (
        imu_sensor.findtext('topic')
        == '/model/robot_1/imu'
    )

    joint_state_plugin = custom_named_robot.find(
        './gazebo/plugin'
        '[@filename="gz-sim-joint-state-publisher-system"]'
    )
    assert joint_state_plugin is not None
    assert (
        joint_state_plugin.findtext('topic')
        == '/model/robot_1/joint_state'
    )

    diff_drive = custom_named_robot.find(
        './gazebo/plugin'
        '[@filename="gz-sim-diff-drive-system"]'
    )
    assert diff_drive is not None
    assert (
        diff_drive.findtext('topic')
        == '/model/robot_1/cmd_vel'
    )
    assert (
        diff_drive.findtext('odom_topic')
        == '/model/robot_1/odometry'
    )
    assert (
        diff_drive.findtext('tf_topic')
        == '/model/robot_1/tf'
    )


def test_frame_prefix_parameterizes_diff_drive_frames(
    prefixed_robot,
):
    diff_drive = prefixed_robot.find(
        './gazebo/plugin'
        '[@filename="gz-sim-diff-drive-system"]'
    )
    assert diff_drive is not None
    assert (
        diff_drive.findtext('frame_id')
        == 'robot_1/odom'
    )
    assert (
        diff_drive.findtext('child_frame_id')
        == 'robot_1/base_footprint'
    )


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


@pytest.mark.parametrize(
    (
        'measurement_name',
        'expected_stddev',
        'expected_bias_mean',
        'expected_bias_stddev',
    ),
    [
        ('angular_velocity', 2.0e-4, 7.5e-6, 8.0e-7),
        ('linear_acceleration', 1.7e-2, 0.1, 1.0e-3),
    ],
)
def test_imu_noise_is_configured(
    robot,
    measurement_name,
    expected_stddev,
    expected_bias_mean,
    expected_bias_stddev,
):
    imu_sensor = robot.find(
        './gazebo[@reference="base_link"]'
        '/sensor[@name="imu_sensor"]'
    )
    assert imu_sensor is not None

    for axis in ('x', 'y', 'z'):
        noise = imu_sensor.find(
            f'imu/{measurement_name}/{axis}/noise'
        )

        assert noise is not None
        assert noise.attrib['type'] == 'gaussian'
        assert float(noise.findtext('mean')) == pytest.approx(0.0)
        assert float(
            noise.findtext('stddev')
        ) == pytest.approx(expected_stddev)
        assert float(
            noise.findtext('bias_mean')
        ) == pytest.approx(expected_bias_mean)
        assert float(
            noise.findtext('bias_stddev')
        ) == pytest.approx(expected_bias_stddev)


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


def test_robot_bridge_parameters_are_robot_specific():
    module_spec = importlib.util.spec_from_file_location(
        'mobile_robot_bridge_launch_parameters',
        BRIDGE_LAUNCH_FILE,
    )
    assert module_spec is not None
    assert module_spec.loader is not None

    launch_module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(launch_module)

    parameter_groups = launch_module._robot_bridge_parameters(
        'robot_1',
        'robot_1/',
    )

    parameters = {}
    for parameter_group in parameter_groups:
        parameters.update(parameter_group)

    assert parameters['bridge_names'] == [
        'joint_states',
        'imu',
        'cmd_vel',
        'wheel_odom',
    ]

    assert (
        parameters['bridges.joint_states.ros_topic_name']
        == 'joint_states_raw'
    )
    assert (
        parameters['bridges.joint_states.gz_topic_name']
        == '/model/robot_1/joint_state'
    )
    assert (
        parameters['bridges.imu.ros_topic_name']
        == 'imu/data'
    )
    assert (
        parameters['bridges.imu.gz_topic_name']
        == '/model/robot_1/imu'
    )
    assert (
        parameters['bridges.imu.frame_id']
        == 'robot_1/base_link'
    )
    assert (
        parameters['bridges.cmd_vel.ros_topic_name']
        == 'cmd_vel'
    )
    assert (
        parameters['bridges.cmd_vel.gz_topic_name']
        == '/model/robot_1/cmd_vel'
    )
    assert (
        parameters['bridges.wheel_odom.ros_topic_name']
        == 'wheel/odom'
    )
    assert (
        parameters['bridges.wheel_odom.gz_topic_name']
        == '/model/robot_1/odometry'
    )
    expected_metadata = {
        'bridges.joint_states.ros_type_name':
            'sensor_msgs/msg/JointState',
        'bridges.joint_states.gz_type_name':
            'gz.msgs.Model',
        'bridges.joint_states.direction':
            'GZ_TO_ROS',
        'bridges.imu.ros_type_name':
            'sensor_msgs/msg/Imu',
        'bridges.imu.gz_type_name':
            'gz.msgs.IMU',
        'bridges.imu.direction':
            'GZ_TO_ROS',
        'bridges.imu.qos_profile':
            'SENSOR_DATA',
        'bridges.cmd_vel.ros_type_name':
            'geometry_msgs/msg/Twist',
        'bridges.cmd_vel.gz_type_name':
            'gz.msgs.Twist',
        'bridges.cmd_vel.direction':
            'ROS_TO_GZ',
        'bridges.wheel_odom.ros_type_name':
            'nav_msgs/msg/Odometry',
        'bridges.wheel_odom.gz_type_name':
            'gz.msgs.Odometry',
        'bridges.wheel_odom.direction':
            'GZ_TO_ROS',
        'bridges.wheel_odom.qos_profile':
            'SENSOR_DATA',
    }

    for parameter_name, expected_value in expected_metadata.items():
        assert parameters[parameter_name] == expected_value

    assert 'tf' not in parameters['bridge_names']


def test_clock_bridge_parameters_are_global():
    module_spec = importlib.util.spec_from_file_location(
        'mobile_robot_clock_bridge_launch_parameters',
        BRIDGE_LAUNCH_FILE,
    )
    assert module_spec is not None
    assert module_spec.loader is not None

    launch_module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(launch_module)

    parameter_groups = launch_module._clock_bridge_parameters()

    parameters = {}
    for parameter_group in parameter_groups:
        parameters.update(parameter_group)

    assert parameters['bridge_names'] == ['clock']
    assert (
        parameters['bridges.clock.ros_topic_name']
        == '/clock'
    )
    assert (
        parameters['bridges.clock.gz_topic_name']
        == '/clock'
    )
    assert (
        parameters['bridges.clock.ros_type_name']
        == 'rosgraph_msgs/msg/Clock'
    )
    assert (
        parameters['bridges.clock.gz_type_name']
        == 'gz.msgs.Clock'
    )
    assert (
        parameters['bridges.clock.direction']
        == 'GZ_TO_ROS'
    )
    assert (
        parameters['bridges.clock.qos_profile']
        == 'CLOCK'
    )


def test_simulation_launch_declares_multi_robot_arguments():
    simulation_launch_file = (
        BRIDGE_LAUNCH_FILE.parent
        / 'simulation.launch.py'
    )
    module_spec = importlib.util.spec_from_file_location(
        'mobile_robot_simulation_launch',
        simulation_launch_file,
    )
    assert module_spec is not None
    assert module_spec.loader is not None

    launch_module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(launch_module)
    launch_description = launch_module.generate_launch_description()
    entity_identifiers = [
        id(action)
        for action in launch_description.entities
    ]
    assert len(entity_identifiers) == len(set(entity_identifiers))

    declared_arguments = {
        action.name
        for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert {
        'bridge_clock',
        'frame_prefix',
        'robot_name',
        'robot_namespace',
        'spawn_x',
        'spawn_y',
        'spawn_yaw',
        'spawn_z',
    }.issubset(declared_arguments)


def test_spawn_arguments_include_robot_identity_and_pose():
    simulation_launch_file = (
        BRIDGE_LAUNCH_FILE.parent
        / 'simulation.launch.py'
    )
    module_spec = importlib.util.spec_from_file_location(
        'mobile_robot_simulation_spawn_arguments',
        simulation_launch_file,
    )
    assert module_spec is not None
    assert module_spec.loader is not None

    launch_module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(launch_module)

    spawn_arguments = launch_module._spawn_robot_arguments(
        'robot_1',
        '1.0',
        '-2.0',
        '0.05',
        '0.75',
    )

    assert spawn_arguments == [
        '-world',
        'empty',
        '-topic',
        'robot_description',
        '-name',
        'robot_1',
        '-x',
        '1.0',
        '-y',
        '-2.0',
        '-z',
        '0.05',
        '-Y',
        '0.75',
    ]


def test_multi_robot_simulation_launch_is_available():
    multi_robot_launch_file = (
        BRIDGE_LAUNCH_FILE.parent
        / 'multi_robot_simulation.launch.py'
    )
    assert multi_robot_launch_file.is_file()

    module_spec = importlib.util.spec_from_file_location(
        'mobile_robot_multi_robot_simulation_launch',
        multi_robot_launch_file,
    )
    assert module_spec is not None
    assert module_spec.loader is not None

    launch_module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(launch_module)
    launch_description = launch_module.generate_launch_description()

    entity_identifiers = [
        id(action)
        for action in launch_description.entities
    ]
    assert len(entity_identifiers) == len(set(entity_identifiers))

    declared_arguments = {
        action.name
        for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert {
        'robot_1_spawn_x',
        'robot_1_spawn_y',
        'robot_1_spawn_yaw',
        'robot_2_spawn_x',
        'robot_2_spawn_y',
        'robot_2_spawn_yaw',
        'spawn_z',
    }.issubset(declared_arguments)
