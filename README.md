# ROS2 Mobile Robot Path Tracking and Multi-Sensor State Estimation System

基于 ROS2 Jazzy、Gazebo Harmonic 和 C++ 的移动机器人路径跟踪与多传感器状态估计仿真项目。

项目目前已完成 ROS2核心通信系统、差速移动机器人模型、RViz可视化、Gazebo仿真接入、基础航点路径跟踪、多传感器平面状态估计，以及带独立命名空间、TF前缀和速度安全层的双机器人仿真基础。

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
- [x] Stage 7: Multi-robot foundation and velocity safety

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

差速移动机器人描述与单/多机器人仿真接入：

- Xacro/URDF机器人模型
- `base_footprint`、`base_link`、左右驱动轮和后部脚轮
- visual、collision 和 inertial 属性
- 参数化 Gazebo模型名称、ROS命名空间和 TF前缀
- `robot_state_publisher`
- RViz模型和 TF可视化
- Gazebo JointStatePublisher、IMU和 DiffDrive插件
- 每台机器人独立的 ROS–Gazebo topic bridge
- 单机器人兼容启动文件
- 双机器人统一启动文件
- 全局共享 `/clock`
- 自动模型、Xacro、launch和命名空间回归测试

### `mobile_robot_control`

基础航点路径跟踪与速度安全系统：

- C++17纯计算角度归一化
- 二维位姿、航点和路径数据结构
- 当前目标航点选择与终点判定
- 距离和航向比例控制
- 线速度与角速度限幅
- 大航向误差下的平移抑制
- 相对 `odom` 订阅和相对 `cmd_vel` 发布
- `SENSOR_DATA` QoS里程计订阅
- 里程计超时安全停止与恢复
- 独立 C++ `cmd_vel_safety` 节点
- 请求速度超时和发布者消失自动归零
- 重复速度发布者检测与安全拒绝
- 每台机器人独立的 `actuator/cmd_vel` 输出
- YAML参数、ROS2 launch、单元测试和代码规范检查

### `mobile_robot_localization`

轮式里程计与 IMU平面状态估计：

- 平面状态 `[x, y, yaw, v, yaw_rate]`
- 状态预测和异常时间步拒绝
- 轮式里程计与 IMU角速度融合
- 过期、未来和非有限 IMU测量拒绝
- 融合里程计和非零协方差发布
- `odom → base_footprint` TF发布
- 相对 ROS话题和参数化 TF前缀
- 每台机器人独立的状态估计器
- C++单元测试与 launch/config回归测试

## Robot TF Structure

默认单机器人模式使用原有 TF名称：

```text
odom
└── base_footprint
    └── base_link
        ├── left_wheel_link
        ├── right_wheel_link
        └── caster_link
```

多机器人模式为每台机器人增加唯一前缀。例如 `robot_1`：

```text
robot_1/odom
└── robot_1/base_footprint
    └── robot_1/base_link
        ├── robot_1/left_wheel_link
        ├── robot_1/right_wheel_link
        └── robot_1/caster_link
```

- `odom → base_footprint` 由 `state_estimator` 发布；
- 固定关节和轮子 TF由 `robot_state_publisher` 发布；
- 轮子角度来自 Gazebo真实关节状态；
- 所有机器人共享 `/tf` 和 `/tf_static` 话题；
- TF frame ID通过前缀保持唯一，不依赖 ROS命名空间隔离。

## Main ROS Topics

下表使用默认单机器人全局名称。多机器人模式会把机器人专属话题放入 `/robot_1`、`/robot_2` 等命名空间。

| ROS topic | Type | Direction or purpose |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | 所有机器人共享的 Gazebo仿真时间 |
| `/robot_description` | `std_msgs/msg/String` | URDF机器人描述 |
| `/joint_states_raw` | `sensor_msgs/msg/JointState` | Gazebo原始关节状态 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 使用仿真时间的关节状态 |
| `/imu/data` | `sensor_msgs/msg/Imu` | Gazebo模拟 IMU测量 |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | Gazebo原始轮式里程计 |
| `/odom` | `nav_msgs/msg/Odometry` | 状态估计器融合输出 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 控制器或手动测试请求速度 |
| `/actuator/cmd_vel` | `geometry_msgs/msg/Twist` | 安全节点过滤后发送给 Gazebo的速度 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 动态坐标变换 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 静态坐标变换 |

双机器人模式示例：

```text
/robot_1/imu/data
/robot_1/wheel/odom
/robot_1/odom
/robot_1/cmd_vel
/robot_1/actuator/cmd_vel

/robot_2/imu/data
/robot_2/wheel/odom
/robot_2/odom
/robot_2/cmd_vel
/robot_2/actuator/cmd_vel

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

不要使用普通 `gz sim` 命令启动 GUI。交互式 Gazebo只能使用项目脚本。

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

### Terminal 2: Start the Single-Robot ROS Simulation

```bash
cd ~/ros2_mobile_robot_project
source install/setup.bash

ros2 launch \
  mobile_robot_description \
  simulation.launch.py
```

该 launch 会自动启动：

- `robot_state_publisher`
- `ros_gz_bridge`
- joint-state timestamp adapter
- simulation-time configuration
- delayed robot spawning
- `state_estimator`
- `cmd_vel_safety`

默认单机器人模式保留原有全局话题和 TF frame名称。

### Terminal 2 Alternative: Start the Dual-Robot ROS Simulation

不要同时启动单机器人和双机器人 launch。需要双机器人时，在终端 2 改为执行：

```bash
cd ~/ros2_mobile_robot_project
source install/setup.bash

ros2 launch \
  mobile_robot_description \
  multi_robot_simulation.launch.py
```

默认配置会生成：

```text
robot_1 at x = -0.75 m
robot_2 at x =  0.75 m
```

每台机器人具有独立的：

- Gazebo model name
- ROS namespace
- TF frame prefix
- ROS–Gazebo bridge
- state estimator
- velocity safety node
- sensor, odometry and command topics

两台机器人共享：

```text
/clock
/tf
/tf_static
```

TF frame ID通过 `robot_1/` 和 `robot_2/` 前缀保持唯一。

## Run Headless Gazebo

不需要图形界面时，可以使用：

```bash
cd ~/ros2_mobile_robot_project
./scripts/run_gazebo_headless.sh
```

该脚本使用 Gazebo服务器模式并自动运行物理仿真，不会打开 GUI。

自动执行完整双机器人无界面冒烟测试：

```bash
cd ~/ros2_mobile_robot_project
./scripts/run_multi_robot_smoke_test.sh
```

冒烟测试会自动：

- 启动无界面 Gazebo；
- 启动双机器人 ROS仿真；
- 检查两套安全节点和状态估计器；
- 检查两套 IMU、轮式里程计和融合里程计；
- 检查两套安全速度输出；
- 关闭 ROS和 Gazebo进程组；
- 保留日志目录用于诊断。

运行冒烟测试前，应先关闭其他单机器人或双机器人仿真。

## State Estimation and Control Data Flow

统一仿真会自动启动状态估计器和速度安全节点，不需要另外启动第二个 `state_estimator` 或 `cmd_vel_safety`。

默认单机器人完整数据流：

```text
/wheel/odom + /imu/data
  → state_estimator
  → /odom
  → path_tracking_controller
  → /cmd_vel
  → cmd_vel_safety
  → /actuator/cmd_vel
  → ros_gz_bridge
  → Gazebo DiffDrive
```

TF数据流：

```text
state_estimator
  → TF: odom → base_footprint

robot_state_publisher
  → TF: base_footprint → base_link → wheel and caster links
```

双机器人模式使用相同结构，但机器人专属话题位于各自命名空间。例如 `robot_1`：

```text
/robot_1/wheel/odom + /robot_1/imu/data
  → /robot_1/state_estimator
  → /robot_1/odom
  → /robot_1/path_tracking_controller
  → /robot_1/cmd_vel
  → /robot_1/cmd_vel_safety
  → /robot_1/actuator/cmd_vel
  → Gazebo model robot_1
```

状态估计器参数位于：

```text
src/mobile_robot_localization/config/state_estimator.yaml
```

其中：

- `max_time_step` 用于拒绝异常轮式里程计时间步；
- `imu_timeout` 用于拒绝过期或未来时间戳的 IMU测量；
- `imu_topic`、`wheel_odom_topic` 和 `odom_topic` 使用相对名称；
- `odom_frame` 和 `base_frame` 可以通过机器人 TF前缀参数化。

如需脱离完整仿真单独启动状态估计器，可以执行：

```bash
ros2 launch \
  mobile_robot_localization \
  state_estimation.launch.py
```

带命名空间和 TF前缀的示例：

```bash
ros2 launch \
  mobile_robot_localization \
  state_estimation.launch.py \
  robot_namespace:=robot_1 \
  frame_prefix:=robot_1/
```

完整仿真已经自动启动状态估计器，因此不要在同一个命名空间中重复执行上述命令。

## Run the Path Tracking Controller

Gazebo和统一仿真正常运行后，在新终端启动默认单机器人控制器：

```bash
cd ~/ros2_mobile_robot_project
source install/setup.bash

ros2 launch \
  mobile_robot_control \
  path_tracking_controller.launch.py
```

双机器人模式下，只控制 `robot_1`：

```bash
ros2 launch \
  mobile_robot_control \
  path_tracking_controller.launch.py \
  robot_namespace:=robot_1
```

只控制 `robot_2`：

```bash
ros2 launch \
  mobile_robot_control \
  path_tracking_controller.launch.py \
  robot_namespace:=robot_2
```

两个控制器可以分别在不同终端启动。它们只订阅和发布各自命名空间中的话题。

默认测试路径为：

```text
(0.0, 0.0) → (1.0, 0.0) → (1.0, 1.0)
```

控制流程为：

```text
融合里程计
  → 二维位姿提取
  → 当前航点选择
  → 距离和航向误差
  → 比例控制
  → 速度与航向限制
  → 请求速度 cmd_vel
  → 速度安全节点
  → actuator/cmd_vel
```

主要参数位于：

```text
src/mobile_robot_control/config/path_tracking_controller.yaml
```

| Parameter | Default | Purpose |
|---|---:|---|
| `odom_topic` | `odom` | 相对融合里程计话题 |
| `cmd_vel_topic` | `cmd_vel` | 相对请求速度话题 |
| `linear_gain` | `0.8` | 距离误差到线速度的比例增益 |
| `angular_gain` | `2.0` | 航向误差到角速度的比例增益 |
| `max_linear_velocity` | `0.4 m/s` | 最大前进速度 |
| `max_angular_velocity` | `1.5 rad/s` | 最大旋转速度 |
| `max_heading_error_for_translation` | `1.0 rad` | 超过该航向误差时停止平移 |
| `waypoint_acceptance_radius` | `0.15 m` | 航点到达半径 |
| `odometry_timeout` | `0.5 s` | 里程计中断后的安全停止阈值 |

Gazebo bridge以 `SENSOR_DATA` QoS发布原始轮式里程计。状态估计器使用 `rclcpp::SensorDataQoS()` 订阅原始传感器数据，并发布融合后的里程计。控制器使用兼容 QoS订阅融合里程计。

如果超过 `odometry_timeout` 没有收到新里程计，控制器会发布零速度；里程计恢复后继续跟踪。

## Velocity Safety Layer

每台机器人拥有独立的 C++ `cmd_vel_safety` 节点：

```text
cmd_vel
  → freshness and publisher-count checks
  → actuator/cmd_vel
```

默认安全参数：

| Parameter | Default | Purpose |
|---|---:|---|
| `input_topic` | `cmd_vel` | 控制器或手动测试请求速度 |
| `output_topic` | `actuator/cmd_vel` | 发送给 Gazebo bridge的安全速度 |
| `command_timeout` | `0.5 s` | 请求速度最大允许时间间隔 |
| `publish_rate` | `20 Hz` | 安全输出频率 |

安全节点在以下情况发布零速度：

- 当前没有速度发布者；
- 尚未收到速度命令；
- 速度命令超过 `command_timeout`；
- 速度发布者已经退出；
- 同一机器人存在多个速度发布者；
- 输入速度包含非有限数值。

Gazebo bridge只订阅 `actuator/cmd_vel`，不会直接接收控制器的 `cmd_vel`。

当检测到多个速度发布者时，安全节点拒绝全部请求并持续输出零速度。冲突解除且只剩一个有效发布者后，安全节点可以恢复正常放行。

结束实验时按以下顺序关闭：

1. 停止路径控制器或手动速度发布器；
2. 确认 `actuator/cmd_vel` 已归零；
3. 停止统一 ROS仿真；
4. 最后关闭 Gazebo。

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

以下命令仅用于没有运行路径跟踪控制器时的底盘测试。

发送命令前检查请求速度话题：

```bash
ros2 topic info /cmd_vel
```

正常手动测试应显示：

```text
Publisher count: 0
Subscription count: 1
```

其中唯一订阅者是 `cmd_vel_safety`。如果已有发布者，不要再启动第二个速度发布器；安全节点检测到多个发布者后会输出零速度。

### Move Forward

向默认单机器人发送约 1秒前进命令：

```bash
ros2 topic pub \
  --rate 10 \
  --times 10 \
  /cmd_vel \
  geometry_msgs/msg/Twist \
  '{linear: {x: 0.10}, angular: {z: 0.0}}'
```

### Rotate Left

向默认单机器人发送约 1秒左转命令：

```bash
ros2 topic pub \
  --rate 10 \
  --times 10 \
  /cmd_vel \
  geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.30}}'
```

### Explicit Stop

```bash
ros2 topic pub \
  --rate 10 \
  --times 5 \
  /cmd_vel \
  geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

手动发布器正常退出后，安全节点也会自动向 `actuator/cmd_vel` 持续发布零速度。

### Command One Robot

双机器人模式下，只控制 `robot_1`：

```bash
ros2 topic pub \
  --rate 10 \
  --times 10 \
  /robot_1/cmd_vel \
  geometry_msgs/msg/Twist \
  '{linear: {x: 0.10}, angular: {z: 0.0}}'
```

只控制 `robot_2` 时，把话题改为：

```text
/robot_2/cmd_vel
```

不要直接向以下内部话题发布：

```text
/actuator/cmd_vel
/robot_1/actuator/cmd_vel
/robot_2/actuator/cmd_vel
```

这些话题只应由对应的 `cmd_vel_safety` 节点发布。

## Inspect the Simulation

检查默认单机器人传感器和里程计：

```bash
ros2 topic echo /imu/data --once
ros2 topic echo /wheel/odom --once
ros2 topic echo /odom --once
```

检查安全速度输出：

```bash
ros2 topic echo /actuator/cmd_vel --once
```

检查默认单机器人 TF链：

```bash
ros2 run tf2_ros tf2_echo \
  odom \
  base_footprint
```

检查多机器人 TF链：

```bash
ros2 run tf2_ros tf2_echo \
  robot_1/odom \
  robot_1/base_footprint
```

```bash
ros2 run tf2_ros tf2_echo \
  robot_2/odom \
  robot_2/base_footprint
```

检查 Gazebo模型：

```bash
gz model -m mobile_robot --pose
```

双机器人模式：

```bash
gz model -m robot_1 --pose
gz model -m robot_2 --pose
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

运行完整工作空间测试：

```bash
cd ~/ros2_mobile_robot_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon test

colcon test-result \
  --all \
  --verbose
```

当前阶段测试基线：

| Stage | Packages | Tests | Errors | Failures | Skipped |
|---|---:|---:|---:|---:|---:|
| Stage 4 | 4 | 27 | 0 | 0 | 1 |
| Stage 5 | 4 | 111 | 0 | 0 | 12 |
| Stage 6 | 5 | 228 | 0 | 0 | 25 |
| Stage 7 | 5 | 267 | 0 | 0 | 28 |

Stage 7自动测试覆盖：

- 参数化机器人名称和 Gazebo model topics；
- 单机器人默认名称向后兼容；
- TF frame前缀和唯一性；
- display、bridge、simulation和 multi-robot launch参数；
- ROS命名空间与相对话题配置；
- 状态估计器相对话题与 TF参数；
- 路径控制器相对 `odom` 和 `cmd_vel` 话题；
- 速度安全 launch配置；
- 新鲜速度命令正常放行；
- 请求速度超时自动归零；
- 发布者消失自动归零；
- 重复速度发布者自动归零；
- C++、Python、XML、CMake和代码格式检查。

运行无界面双机器人冒烟测试：

```bash
./scripts/run_multi_robot_smoke_test.sh
```

该测试验证真实 Gazebo与 ROS 2运行时，而不是只检查静态配置。

## Stage 7 Verification Results

Stage 7已在 Ubuntu 24.04 ARM64、ROS 2 Jazzy和 Gazebo Harmonic环境中完成验证。

### Dual-Robot Runtime

已验证：

- `robot_1` 和 `robot_2` 使用不同 Gazebo model name；
- 两套 ROS话题没有冲突；
- 两套 TF frame ID没有冲突；
- 两套 IMU约为 `99 Hz`；
- 两套轮式里程计与融合里程计约为 `50 Hz`；
- 两套状态估计器独立运行；
- 两套速度安全节点独立运行；
- 只有目标机器人响应其命名空间中的速度命令。

独立运动测试中：

```text
robot_1 movement ≈ 0.396 m
robot_2 movement ≈ 0.000 m
```

请求速度发布器退出后，`robot_1` 在额外 3秒内没有继续移动。

### Velocity Safety

已验证：

- 新鲜命令正常放行；
- 命令暂停超过 `0.5 s` 后自动归零；
- 发布者退出后自动归零；
- 两个发布者同时存在时自动归零；
- 冲突解除后可以恢复正常放行；
- `robot_1` 超时时，`robot_2` 仍继续接收 `0.15 m/s`；
- 路径控制器退出后，安全输出自动归零；
- 实验结束前两台机器人均收到零速度。

### Single-Robot Compatibility

原有单机器人启动命令无需新增参数即可完成默认路径：

```text
(0.0, 0.0) → (1.0, 0.0) → (1.0, 1.0)
```

Stage 7单机器人兼容性测试结果：

| Metric | Result |
|---|---:|
| Final odometry X | `1.05190 m` |
| Final odometry Y | `0.87878 m` |
| Final position error | `0.1319 m` |
| Acceptance radius | `0.15 m` |
| Final actuator velocity | zero |

最终位置误差小于航点到达半径，因此默认单机器人工作流保持兼容。

### Headless Automation

无界面冒烟测试成功验证：

- 两套安全节点；
- 两套状态估计器；
- 两套 IMU；
- 两套轮式里程计；
- 两套融合里程计；
- 两套安全执行器速度；
- 共享仿真时钟；
- ROS launch和 Gazebo进程组自动清理。

测试结束后没有残留 ROS节点或 Gazebo服务器进程。

## Rendering Notes

Gazebo Harmonic GUI使用 Ogre renderer、项目 no-shadow world和 Mesa OpenGL 3.3兼容模式启动。

该配置用于避免 Parallels `virgl` 虚拟 GPU环境中的渲染不稳定。交互式 Gazebo GUI只能使用：

```bash
./scripts/run_gazebo_gui.sh
```

不要用普通 `gz sim` 命令替代经过验证的 GUI启动器。

RViz使用独立的软件渲染启动器：

```bash
./scripts/run_rviz.sh
```

无界面测试使用：

```bash
./scripts/run_gazebo_headless.sh
```

## Known Limitations

- 当前模型使用简化的轮胎和脚轮接触参数；
- 轮胎滑移可能导致 Gazebo ground truth与轮式里程计存在差异；
- 当前路径控制器跟踪参数化离散航点，尚未实现连续曲线跟踪和避障；
- Stage 6状态估计器使用轻量平面预测与逆方差角速度融合，不是完整协方差传播 EKF；
- 尚未加入独立 Gazebo ground-truth精度基准；
- Stage 7只提供多机器人基础与速度安全层，尚未实现两篇论文的编队控制算法；
- 正常控制器退出、命令超时和正常 launch关闭已覆盖安全归零；
- 如果 `cmd_vel_safety` 被强制 `SIGKILL`、系统掉电或进程异常崩溃，它无法在退出前发布最终零速度，因此实验仍应按文档顺序关闭控制器、ROS仿真和 Gazebo；
- 无界面冒烟测试日志保留在 `/tmp/mobile_robot_stage7_smoke.*`，用于失败诊断。

## Stage 6 State Estimation

Stage 6提供模拟 IMU、原始 `/wheel/odom`、轮式里程计与 IMU角速度融合、融合 `/odom`、`odom → base_footprint` TF、可配置输出协方差和过期 IMU拒绝。

## Stage 7 Multi-Robot Foundation

Stage 7提供：

- 参数化机器人身份；
- ROS命名空间；
- 唯一 TF前缀；
- 动态 ROS–Gazebo bridge；
- 单机器人兼容 launch；
- 双机器人统一 launch；
- 每台机器人独立状态估计；
- 每台机器人独立 C++速度安全层；
- 重复发布者检测；
- 超时与发布者退出归零；
- 无界面 Gazebo自动冒烟测试。
