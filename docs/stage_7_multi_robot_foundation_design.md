# Stage 7 Multi-Robot Foundation Design

## Objective

Stage 7 establishes a safe and testable ROS 2 and Gazebo multi-robot foundation.

It does not implement either paper's formation-control algorithm. Those algorithms will reuse this foundation in later stages.

## Compatibility

The existing single-robot simulation remains supported.

When no robot namespace or frame prefix is supplied:

- ROS topics resolve to the existing global topic names;
- TF frame names remain unchanged;
- the default Gazebo model name remains `mobile_robot`;
- existing launch commands continue to work;
- the velocity safety layer starts automatically;
- the existing path-tracking controller remains compatible.

## Robot Identity

Each robot has a unique identifier such as:

```text
robot_1
robot_2
```

The identifier is used for:

- the Gazebo model name;
- the ROS namespace;
- the TF frame prefix;
- robot-specific configuration.

## ROS Namespace

A robot named `robot_1` uses:

```text
/robot_1
```

Robot-specific topic names are relative inside node configuration:

```text
cmd_vel
actuator/cmd_vel
wheel/odom
imu/data
odom
joint_states_raw
joint_states
```

Within the `robot_1` namespace, these resolve to:

```text
/robot_1/cmd_vel
/robot_1/actuator/cmd_vel
/robot_1/wheel/odom
/robot_1/imu/data
/robot_1/odom
/robot_1/joint_states_raw
/robot_1/joint_states
```

The simulation clock remains shared:

```text
/clock
```

## Gazebo Topics

For a robot named `robot_1`, Gazebo uses:

```text
/model/robot_1/cmd_vel
/model/robot_1/odometry
/model/robot_1/imu
/model/robot_1/joint_state
```

The bridge maps the namespaced ROS safety output:

```text
/robot_1/actuator/cmd_vel
```

to:

```text
/model/robot_1/cmd_vel
```

Sensor and odometry data are mapped in the opposite direction to the corresponding namespaced ROS topics.

## TF Frames

TF frame identifiers are not isolated by ROS namespaces. Every multi-robot frame therefore requires a unique prefix.

For `robot_1`, the main TF chain is:

```text
robot_1/odom
└── robot_1/base_footprint
    └── robot_1/base_link
        ├── robot_1/left_wheel_link
        ├── robot_1/right_wheel_link
        └── robot_1/caster_link
```

All robots share `/tf` and `/tf_static`, while their frame identifiers remain unique.

In single-robot compatibility mode, the prefix is empty and the existing frame names are preserved.

## Node Names

Node names remain unchanged inside separate namespaces:

```text
/robot_1/state_estimator
/robot_2/state_estimator

/robot_1/cmd_vel_safety
/robot_2/cmd_vel_safety

/robot_1/path_tracking_controller
/robot_2/path_tracking_controller
```

## Velocity Safety Architecture

External controllers and manual test publishers write requested velocity commands to:

```text
cmd_vel
```

The per-robot C++ safety node validates those commands and publishes to:

```text
actuator/cmd_vel
```

Only the safety-filtered actuator topic is bridged to Gazebo:

```text
controller or manual publisher
  → cmd_vel
  → cmd_vel_safety
  → actuator/cmd_vel
  → ros_gz_bridge
  → Gazebo DiffDrive
```

This additional layer is required because Gazebo DiffDrive retains its last velocity target until it receives a new command or is disabled.

The safety node publishes at `20 Hz` by default and uses a default command timeout of `0.5 s`.

## Safety Requirements

Each robot satisfies the following requirements independently:

- its controller publishes only to its requested `cmd_vel` topic;
- only its safety node publishes to its `actuator/cmd_vel` topic;
- stale odometry causes the path controller to publish zero velocity;
- stale velocity commands cause the safety node to publish zero velocity;
- controller shutdown causes the safety node to publish zero velocity;
- disappearance of a command publisher produces zero velocity;
- multiple command publishers are detected and produce zero velocity;
- one robot's timeout does not stop another robot;
- all robots receive zero velocity after an experiment finishes.

## Launch Architecture

The single-robot launch remains:

```bash
ros2 launch mobile_robot_description simulation.launch.py
```

It accepts optional identity and spawn arguments:

```text
robot_name
robot_namespace
frame_prefix
spawn_x
spawn_y
spawn_z
spawn_yaw
bridge_clock
```

The dual-robot launch is:

```bash
ros2 launch \
  mobile_robot_description \
  multi_robot_simulation.launch.py
```

It starts:

- `robot_1` at the default X position `-0.75 m`;
- `robot_2` at the default X position `0.75 m`;
- one shared `/clock` bridge;
- one description, bridge, safety and estimator stack per robot.

## Test Strategy

Stage 7 uses tests at three levels:

1. Pure C++, configuration and Xacro regression tests.
2. Registered ROS launch, namespace and topic-configuration tests.
3. A headless Gazebo multi-robot smoke test.

The automated headless smoke test is:

```bash
./scripts/run_multi_robot_smoke_test.sh
```

It verifies:

- both per-robot safety nodes;
- both per-robot state estimators;
- the shared simulation clock;
- independent IMU data;
- independent wheel odometry;
- independent fused odometry;
- independent safety-filtered actuator topics;
- automatic shutdown of the ROS and Gazebo process groups.

The existing single-robot regression baseline continues to pass.

## Verification Evidence

Stage 7 runtime verification was completed on Ubuntu 24.04 ARM64 with ROS 2 Jazzy and Gazebo Harmonic.

Verified results include:

- two robots spawned with unique Gazebo model names;
- independent namespaced ROS topics;
- unique prefixed TF frame chains;
- IMU output at approximately `99 Hz`;
- wheel and fused odometry at approximately `50 Hz`;
- independent state estimators for both robots;
- a robot 1 motion test of approximately `0.396 m`;
- robot 2 remaining effectively stationary during that test;
- no additional robot 1 movement after the command publisher exited;
- runtime detection of duplicate velocity publishers;
- automatic zero output when a publisher disappeared;
- automatic zero output after command timeout;
- robot 1 timeout while robot 2 continued receiving `0.15 m/s`;
- zero actuator output after path-controller shutdown;
- successful headless multi-robot smoke testing with no residual processes;
- successful single-robot path tracking using the original launch commands.

The single-robot compatibility run finished at approximately:

```text
x = 1.05190 m
y = 0.87878 m
```

The final position error was approximately:

```text
0.1319 m
```

This is within the `0.15 m` waypoint acceptance radius.

The full workspace checkpoint passed:

```text
5 packages
267 tests
0 errors
0 failures
28 skipped
```

## Stage 7 Acceptance Criteria

Stage 7 is complete when:

- at least two robots can be spawned with unique model names;
- robot-specific ROS topics do not collide;
- TF frames do not collide;
- each robot publishes independent wheel odometry and IMU data;
- each robot can run an independent state estimator;
- independent velocity commands move only the intended robot;
- timeout and shutdown behavior produce zero velocity commands;
- duplicate velocity publishers are rejected safely;
- one robot's timeout does not stop another robot;
- the headless multi-robot smoke test passes and cleans up its processes;
- the complete workspace test suite passes;
- the single-robot workflow remains backward compatible.

All Stage 7 acceptance criteria were verified on 2026-08-11.
