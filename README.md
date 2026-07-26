# ROS2 Mobile Robot Path Tracking and Multi-Sensor State Estimation System

基于 ROS2 Jazzy、Gazebo Harmonic 和 C++ 的移动机器人路径跟踪与多传感器状态估计仿真项目。

项目目前已完成 ROS2核心通信系统、差速移动机器人模型、RViz可视化、Gazebo仿真接入、速度控制、轮式里程计和完整 TF链路。

## Development Environment

- Host: Apple M4 Mac
- Virtualization: Parallels Desktop 26
- Guest OS: Ubuntu 24.04 ARM64
- ROS: ROS2 Jazzy
- Simulator: Gazebo Harmonic / Gazebo Sim 8
- Visualization: RViz2
- Main language: C++
- Data analysis: Python 3
- Build system: colcon
- Dependency management: rosdep

## Progress

- [x] Stage 0: Ubuntu ARM64 virtual machine setup
- [x] Stage 1: Linux, Git and SSH fundamentals
- [x] Stage 2: ROS2 Jazzy and Gazebo Harmonic installation
- [x] Stage 3: ROS2 core concepts
- [x] Stage 4: Differential-drive robot model and Gazebo integration
- [ ] Stage 5: Path tracking controller
- [ ] Stage 6: Multi-sensor state estimation
- [ ] Stage 7: Safety, testing and engineering
- [ ] Stage 8: Experiments and project presentation

## Implemented Packages

### `mobile_robot_interfaces`

自定义 ROS2接口：

- `RobotStatus.msg`
- `SetRobotState.srv`

### `mobile_robot_core`

ROS2核心概念示例与状态系统：

- publisher and subscriber
- custom messages
- services
- parameters
- launch files
- automated tests

### `mobile_robot_description`

差速移动机器人描述与仿真接入：

- Xacro/URDF机器人模型
- `base_footprint` 和 `base_link`
- 左右驱动轮与后部脚轮
- visual、collision 和 inertial 属性
- `robot_state_publisher`
- RViz模型和 TF可视化
- Gazebo JointStatePublisher插件
- Gazebo DiffDrive插件
- ROS–Gazebo topic bridge
- 仿真时间统一
- 自动模型回归测试

## Robot TF Structure

```text
odom
└── base_footprint
    └── base_link
        ├── left_wheel_link
        ├── right_wheel_link
        └── caster_link
```

- `odom → base_footprint` 由 Gazebo DiffDrive发布
- 固定关节和轮子 TF由 `robot_state_publisher` 发布
- 轮子角度来自 Gazebo真实关节状态

## Main ROS Topics

| ROS topic | Type | Direction or purpose |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo仿真时间 |
| `/robot_description` | `std_msgs/msg/String` | URDF机器人描述 |
| `/joint_states_raw` | `sensor_msgs/msg/JointState` | Gazebo原始关节状态 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 使用仿真时间的关节状态 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS到 Gazebo速度命令 |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo到 ROS轮式里程计 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 动态坐标变换 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 静态坐标变换 |

## Build

在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y

colcon build --symlink-install
source install/setup.bash
```

## Run the Gazebo Simulation

Gazebo GUI和 ROS仿真节点分开启动，以保留 Parallels虚拟 GPU下已经验证的稳定渲染方案。

### Terminal 1: Start Gazebo GUI

```bash
cd ~/ros2_mobile_robot_project
./scripts/run_gazebo_gui.sh
```

该脚本使用：

- Ogre renderer
- no-shadow project world
- Mesa OpenGL 3.3 compatibility mode

Gazebo窗口打开后，点击左下角的 **Play** 按钮启动物理仿真。

### Terminal 2: Start the unified ROS simulation

```bash
cd ~/ros2_mobile_robot_project
source install/setup.bash

ros2 launch mobile_robot_description simulation.launch.py
```

该 launch 会自动启动：

- `robot_state_publisher`
- `ros_gz_bridge`
- joint-state timestamp adapter
- simulation-time configuration
- delayed robot spawning

## Run RViz

仿真运行后，在新终端执行：

```bash
cd ~/ros2_mobile_robot_project
./scripts/run_rviz.sh
```

该脚本启用软件渲染和 OpenGL 3.3兼容模式，避免 Parallels `virgl` 环境下的黑屏问题。

RViz固定坐标系为：

```text
base_footprint
```

## Send Velocity Commands

### Move forward

```bash
ros2 topic pub --once \
  /cmd_vel \
  geometry_msgs/msg/Twist \
  '{linear: {x: 0.10}, angular: {z: 0.0}}'
```

### Rotate left

```bash
ros2 topic pub --once \
  /cmd_vel \
  geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.30}}'
```

### Stop

```bash
ros2 topic pub --once \
  /cmd_vel \
  geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

## Inspect the Simulation

Check odometry:

```bash
ros2 topic echo /odom --once
```

Check joint states:

```bash
ros2 topic echo /joint_states --once
```

Check the complete TF chain:

```bash
ros2 run tf2_ros tf2_echo \
  odom \
  left_wheel_link \
  --ros-args \
  -p use_sim_time:=true
```

Check the Gazebo model pose:

```bash
gz model -m mobile_robot --pose
```

## Test

Run all workspace tests:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Stage 4 completion baseline:

```text
27 tests, 0 errors, 0 failures, 1 skipped
```

The robot-description regression tests verify:

- required links and joints
- wheel joint geometry and axes
- visual, collision and inertial properties
- Gazebo JointStatePublisher configuration
- Gazebo DiffDrive configuration
- wheel radius and wheel separation
- command, odometry and TF topic names

## Rendering Notes

Gazebo Harmonic GUI is launched with the Ogre renderer, shadows disabled in the project world, and Mesa OpenGL 3.3 compatibility mode.

This configuration avoids rendering instability with the Parallels `virgl` virtual GPU. Do not replace the verified launcher with a plain `gz sim` command when using this environment.

RViz uses the separate `scripts/run_rviz.sh` software-rendering launcher.

Headless Gazebo remains available for automated and non-visual tests.

## Known Limitations

- The current model uses simplified wheel and caster contact properties.
- Wheel slip can cause differences between Gazebo ground-truth pose and wheel odometry.
- Gazebo may initially open with physics paused; click **Play** before sending motion commands.
- The current system provides basic velocity control but does not yet include a path-tracking controller.
- Sensor simulation and multi-sensor state estimation will be added in later stages.

## Next Step

Stage 5 will add the path-tracking control layer, including trajectory references, tracking-error calculation, velocity command generation and quantitative tracking evaluation.
