# Stage 7 Multi-Robot Foundation Design

## Objective

Stage 7 establishes a safe and testable ROS 2 and Gazebo multi-robot foundation.

It does not implement either paper's formation-control algorithm. Those algorithms will reuse this foundation in later stages.

## Compatibility

The existing single-robot simulation must remain supported.

When no robot namespace or frame prefix is supplied:

- ROS topics resolve to the existing global topic names;
- TF frame names remain unchanged;
- the default Gazebo model name remains `mobile_robot`;
- existing launch commands and tests continue to work.

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

Robot-specific topic names are relative inside the node configuration:

```text
cmd_vel
wheel/odom
imu/data
odom
joint_states_raw
joint_states
```

Within the `robot_1` namespace, these resolve to:

```text
/robot_1/cmd_vel
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

The bridge maps these Gazebo topics to the corresponding namespaced ROS topics.

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

Node names may remain unchanged inside separate namespaces:

```text
/robot_1/state_estimator
/robot_2/state_estimator

/robot_1/path_tracking_controller
/robot_2/path_tracking_controller
```

## Safety Requirements

Each robot must satisfy the following requirements independently:

- only its own controller publishes to its `cmd_vel` topic;
- stale odometry causes a zero velocity command;
- controller shutdown publishes a zero velocity command;
- one robot's timeout does not stop another robot;
- duplicate velocity publishers can be detected;
- all robots publish zero velocity after an experiment finishes.

## Test Strategy

Stage 7 adds tests at three levels:

1. Pure configuration and Xacro regression tests.
2. ROS launch and namespace integration tests.
3. Headless Gazebo multi-robot smoke tests.

The existing single-robot regression baseline must continue to pass.

## Stage 7 Acceptance Criteria

Stage 7 is complete when:

- at least two robots can be spawned with unique model names;
- robot-specific ROS topics do not collide;
- TF frames do not collide;
- each robot publishes independent wheel odometry and IMU data;
- each robot can run an independent state estimator;
- independent velocity commands move only the intended robot;
- timeout and shutdown behavior produce zero velocity commands;
- the complete workspace test suite passes;
- the single-robot workflow remains backward compatible.
