# Stage 6: Multi-Sensor State Estimation Design

## Scope

Stage 6 estimates planar robot motion. Roll, pitch, vertical position, and lateral velocity are not included in the estimator state.

## Coordinate Frames

### `odom`

A local, continuous reference frame. Estimated position and heading are expressed in this frame.

### `base_footprint`

The planar robot frame. Forward velocity is along its positive X axis, and positive yaw rotation is around its positive Z axis.

### `base_link`

The physical chassis frame. Its fixed relationship with `base_footprint` remains the responsibility of `robot_state_publisher`.

## State Vector

The planar state is:

`state = [x, y, yaw, v, yaw_rate]^T`

where:

- `x`: X position in `odom`, in metres;
- `y`: Y position in `odom`, in metres;
- `yaw`: heading in `odom`, in radians;
- `v`: forward velocity in `base_footprint`, in metres per second;
- `yaw_rate`: angular velocity around the positive Z axis, in radians per second.

Yaw must be normalized to `[-pi, pi]`.

## Sensor Inputs

### Wheel odometry

Planned topic: `/wheel/odom`

Message type: `nav_msgs/msg/Odometry`

The current Gazebo bridge publishes raw wheel odometry on `/odom`. It will be renamed to `/wheel/odom` before the estimator publishes its output.

### IMU

Planned topic: `/imu/data`

Message type: `sensor_msgs/msg/Imu`

The first fusion implementation will use Z-axis angular velocity.

## Estimator Output

Estimated odometry topic: `/odom`

Message type: `nav_msgs/msg/Odometry`

Frame fields:

- `header.frame_id = odom`
- `child_frame_id = base_footprint`

The estimator will eventually be the only publisher of:

`odom -> base_footprint`

Before estimator TF publication is enabled, the Gazebo odometry TF bridge must stop publishing this transform.

`robot_state_publisher` will continue publishing internal robot transforms.

## Time and Data Rules

- Use sensor message timestamps.
- Use ROS simulation time in Gazebo.
- Reject non-finite measurements.
- Reject non-positive time steps.
- Treat excessively large time steps as discontinuities.
- Normalize every updated yaw angle.
- Do not publish before initialization from valid data.
- Do not treat all-zero covariance as realistic uncertainty.

## Verified Baseline

The existing system has:

- `/odom` published by `ros_gz_bridge`;
- `BEST_EFFORT` odometry QoS;
- `frame_id = odom`;
- `child_frame_id = base_footprint`;
- an odometry rate of approximately 50 Hz;
- a working `odom -> base_footprint` transform;
- all-zero pose and twist covariance.