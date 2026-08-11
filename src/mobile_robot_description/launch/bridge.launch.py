from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_bridge_parameters(robot_name, frame_prefix):
    return [
        {
            'bridge_names': [
                'joint_states',
                'imu',
                'cmd_vel',
                'wheel_odom',
            ],
        },
        {
            'bridges.joint_states.ros_topic_name':
                'joint_states_raw',
        },
        {
            'bridges.joint_states.gz_topic_name':
                f'/model/{robot_name}/joint_state',
        },
        {
            'bridges.joint_states.ros_type_name':
                'sensor_msgs/msg/JointState',
        },
        {
            'bridges.joint_states.gz_type_name':
                'gz.msgs.Model',
        },
        {
            'bridges.joint_states.direction':
                'GZ_TO_ROS',
        },
        {
            'bridges.imu.ros_topic_name':
                'imu/data',
        },
        {
            'bridges.imu.gz_topic_name':
                f'/model/{robot_name}/imu',
        },
        {
            'bridges.imu.ros_type_name':
                'sensor_msgs/msg/Imu',
        },
        {
            'bridges.imu.gz_type_name':
                'gz.msgs.IMU',
        },
        {
            'bridges.imu.direction':
                'GZ_TO_ROS',
        },
        {
            'bridges.imu.qos_profile':
                'SENSOR_DATA',
        },
        {
            'bridges.imu.frame_id':
                f'{frame_prefix}base_link',
        },
        {
            'bridges.cmd_vel.ros_topic_name':
            'actuator/cmd_vel',
        },
        {
            'bridges.cmd_vel.gz_topic_name':
                f'/model/{robot_name}/cmd_vel',
        },
        {
            'bridges.cmd_vel.ros_type_name':
                'geometry_msgs/msg/Twist',
        },
        {
            'bridges.cmd_vel.gz_type_name':
                'gz.msgs.Twist',
        },
        {
            'bridges.cmd_vel.direction':
                'ROS_TO_GZ',
        },
        {
            'bridges.wheel_odom.ros_topic_name':
                'wheel/odom',
        },
        {
            'bridges.wheel_odom.gz_topic_name':
                f'/model/{robot_name}/odometry',
        },
        {
            'bridges.wheel_odom.ros_type_name':
                'nav_msgs/msg/Odometry',
        },
        {
            'bridges.wheel_odom.gz_type_name':
                'gz.msgs.Odometry',
        },
        {
            'bridges.wheel_odom.direction':
                'GZ_TO_ROS',
        },
        {
            'bridges.wheel_odom.qos_profile':
                'SENSOR_DATA',
        },
    ]


def _clock_bridge_parameters():
    return [
        {
            'bridge_names': ['clock'],
        },
        {
            'bridges.clock.ros_topic_name': '/clock',
        },
        {
            'bridges.clock.gz_topic_name': '/clock',
        },
        {
            'bridges.clock.ros_type_name':
                'rosgraph_msgs/msg/Clock',
        },
        {
            'bridges.clock.gz_type_name':
                'gz.msgs.Clock',
        },
        {
            'bridges.clock.direction': 'GZ_TO_ROS',
        },
        {
            'bridges.clock.qos_profile': 'CLOCK',
        },
    ]


def _launch_setup(context):
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_namespace = LaunchConfiguration(
        'robot_namespace'
    ).perform(context)

    robot_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        namespace=robot_namespace,
        output='screen',
        parameters=_robot_bridge_parameters(
            robot_name,
            frame_prefix,
        ),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('bridge_clock')
        ),
        parameters=_clock_bridge_parameters(),
    )

    joint_state_retimestamp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_retimestamp',
        namespace=robot_namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'source_list': ['joint_states_raw'],
            'publish_default_positions': True,
            'publish_default_velocities': False,
            'publish_default_efforts': False,
            'rate': 50,
        }],
    )

    return [
        robot_bridge,
        clock_bridge,
        joint_state_retimestamp,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_clock',
            default_value='true',
            description='Start the shared Gazebo clock bridge',
        ),

        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='Prefix applied to bridged sensor frames',
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

        OpaqueFunction(function=_launch_setup),
    ])
