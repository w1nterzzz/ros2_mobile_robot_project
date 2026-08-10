# ROS2 Mobile Robot Path Tracking and Multi-Sensor State Estimation System

基于 ROS2 Jazzy、Gazebo Harmonic 和 C++ 的移动机器人路径跟踪与多传感器状态估计仿真项目。

项目目前已完成 ROS2核心通信系统、差速移动机器人模型、RViz可视化、Gazebo仿真接入、轮式里程计、完整 TF链路和基础航点路径跟踪控制器。

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
- [x] Stage 5: Path tracking controller
- [x] Stage 6: Multi-sensor state estimation
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

### `mobile_robot_control`

基础航点路径跟踪控制器：

- C++17纯计算角度归一化
- 二维位姿、航点和路径数据结构
- 当前目标航点选择与终点判定
- 距离和航向比例控制
- 线速度与角速度限幅
- 大航向误差下的平移抑制
- `/odom` 订阅和 `/cmd_vel` 发布
- `SENSOR_DATA` QoS里程计订阅
- 里程计超时安全停止与恢复
- YAML参数和 ROS2 launch
- 自动单元测试和代码规范检查

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
- `state_estimator`，融合 `/wheel/odom` 和 `/imu/data`，并发布 `/odom` 与 `odom → base_footprint` TF

## State Estimation Data Flow

统一仿真会自动启动状态估计器，不需要另外启动第二个 `state_estimator` 节点。

```text
/wheel/odom + /imu/data
  → state_estimator
  → /odom
  → path_tracking_controller
  → /cmd_vel

state_estimator
  → TF: odom → base_footprint
```

状态估计器参数位于：

```text
src/mobile_robot_localization/config/state_estimator.yaml
```

其中 `max_time_step` 用于拒绝异常的轮式里程计时间步，`imu_timeout` 用于拒绝过期或未来时间戳的 IMU 测量。

如需脱离完整仿真单独启动状态估计器，可以执行：

```bash
ros2 launch mobile_robot_localization state_estimation.launch.py
```

## Run the Path Tracking Controller

Gazebo GUI和统一仿真正常运行后，在第三个终端执行：

```bash
cd ~/ros2_mobile_robot_project
source install/setup.bash

ros2 launch mobile_robot_control path_tracking_controller.launch.py
```

默认测试路径为：

```text
(0.0, 0.0) → (1.0, 0.0) → (1.0, 1.0)
```

控制流程为：

```text
/odom（状态估计器融合输出）
  → 二维位姿提取
  → 当前航点选择
  → 距离和航向误差
  → 比例控制
  → 速度与航向限制
  → /cmd_vel
```

主要参数位于：

```text
src/mobile_robot_control/config/path_tracking_controller.yaml
```

| Parameter | Default | Purpose |
|---|---:|---|
| `linear_gain` | `0.8` | 距离误差到线速度的比例增益 |
| `angular_gain` | `2.0` | 航向误差到角速度的比例增益 |
| `max_linear_velocity` | `0.4 m/s` | 最大前进速度 |
| `max_angular_velocity` | `1.5 rad/s` | 最大旋转速度 |
| `max_heading_error_for_translation` | `1.0 rad` | 超过该航向误差时停止平移 |
| `waypoint_acceptance_radius` | `0.15 m` | 航点到达半径 |
| `odometry_timeout` | `0.5 s` | 里程计中断后的安全停止阈值 |

Gazebo bridge 以 `SENSOR_DATA` QoS 将原始轮式里程计发布到 `/wheel/odom`。状态估计器使用 `rclcpp::SensorDataQoS()` 订阅原始里程计，并以 Reliable QoS 发布融合后的 `/odom`。控制器使用 `rclcpp::SensorDataQoS()` 订阅 `/odom`，两端 QoS 兼容。

如果超过 `odometry_timeout` 没有收到新里程计，控制器会发布零速度；里程计恢复后继续跟踪。

运行控制器时，不要同时使用命令行或其他节点发布 `/cmd_vel`。结束实验时，先停止控制器，再停止统一仿真，最后关闭 Gazebo。

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

以下命令仅用于不运行路径跟踪控制器时的底盘测试。发送手动命令前，应确认 `/cmd_vel` 没有其他发布器。

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

## Stage 5 Path Tracking Evaluation

Gazebo首次闭环实验使用默认参数和三航点直角路径：

| Metric | Result |
|---|---:|
| Path | `(0,0) → (1,0) → (1,1)` |
| Approximate completion time | `6.2 s` |
| Final odometry position | `(1.013733, 0.858761) m` |
| Final position error | `0.1419 m` |
| Acceptance radius | `0.15 m` |
| Final `/cmd_vel` | zero |
| Final odometry velocity | zero |
| Odometry timeout during run | no |

最终误差计算为：

```text
sqrt((1.0 - 1.013733)^2 + (1.0 - 0.858761)^2)
≈ 0.1419 m
```

该结果满足 `0.15 m` 航点到达半径。当前参数作为 Stage 5基础闭环基线保留，后续实验阶段可进一步比较不同增益、速度上限和路径形状。

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

Stage 5 full-workspace baseline:

```text
4 packages finished
111 tests, 0 errors, 0 failures, 12 skipped
```

The path-tracking tests verify:

- angle normalization and wrap-around boundaries
- pose, waypoint and path data structures
- waypoint advancement and completion
- invalid path and configuration handling
- distance and heading control calculations
- velocity limits and translation suppression

Stage 6 full-workspace baseline:

```text
5 packages finished
228 tests, 0 errors, 0 failures, 25 skipped
```

The state-estimation tests verify:

- planar state representation and angle normalization
- state prediction and invalid time-step handling
- wheel-odometry and IMU yaw-rate fusion
- invalid measurement and variance fallback behavior
- estimator initialization and update behavior
- stale, future and non-finite IMU measurement rejection
- simulation odometry routing and TF ownership

## Rendering Notes

Gazebo Harmonic GUI is launched with the Ogre renderer, shadows disabled in the project world, and Mesa OpenGL 3.3 compatibility mode.

This configuration avoids rendering instability with the Parallels `virgl` virtual GPU. Do not replace the verified launcher with a plain `gz sim` command when using this environment.

RViz uses the separate `scripts/run_rviz.sh` software-rendering launcher.

Headless Gazebo remains available for automated and non-visual tests.

## Known Limitations

- The current model uses simplified wheel and caster contact properties.
- Wheel slip can cause differences between Gazebo ground-truth pose and wheel odometry.
- Gazebo may initially open with physics paused; click **Play** before sending motion commands.
- The current controller follows parameterized point waypoints; continuous-curve tracking and obstacle avoidance are not yet implemented.
- The Stage 6 estimator uses a lightweight planar prediction and inverse-variance yaw-rate fusion model; it is not a full covariance-propagating EKF.
- An independent Gazebo ground-truth accuracy benchmark is not yet included.

## Stage 6 State Estimation

Stage 6 provides simulated IMU measurements, raw wheel odometry on `/wheel/odom`, wheel-odometry and IMU yaw-rate fusion, fused `/odom` output, `odom → base_footprint` TF broadcasting, configurable output covariance and stale-IMU rejection.
