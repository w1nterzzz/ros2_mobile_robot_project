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

Topic: `/wheel/odom`

Message type: `nav_msgs/msg/Odometry`

The Gazebo bridge publishes raw wheel odometry on `/wheel/odom`. The estimator consumes this topic and reserves `/odom` for fused output.

### IMU

Topic: `/imu/data`

Message type: `sensor_msgs/msg/Imu`

The estimator uses Z-axis angular velocity and fuses it with wheel yaw rate using inverse-variance weighting. Wheel odometry supplies forward linear velocity.

## Estimator Output

Estimated odometry topic: `/odom`

Message type: `nav_msgs/msg/Odometry`

Frame fields:

- `header.frame_id = odom`
- `child_frame_id = base_footprint`

The estimator is the only publisher of the `odom -> base_footprint` transform. The Gazebo TF bridge is disabled, while `robot_state_publisher` continues publishing internal robot transforms.

## Time and Data Rules

- Use sensor message timestamps.
- Use ROS simulation time in Gazebo.
- Reject non-finite measurements.
- Reject non-positive time steps.
- Treat excessively large time steps as discontinuities.
- Normalize every updated yaw angle.
- Do not publish before initialization from valid data.
- Do not treat all-zero covariance as realistic uncertainty.

## Implemented and Verified Architecture

The implemented system has:

- raw wheel odometry on `/wheel/odom` at approximately 50 Hz;
- simulated IMU measurements on `/imu/data` at approximately 100 Hz;
- `BEST_EFFORT` sensor-input QoS through `rclcpp::SensorDataQoS()`;
- fused odometry on `/odom` at approximately 50 Hz;
- `RELIABLE` fused-odometry publication;
- `frame_id = odom`;
- `child_frame_id = base_footprint`;
- configurable non-zero pose and twist covariance;
- `odom -> base_footprint` TF published only by the estimator;
- invalid time-step and non-finite measurement rejection;
- stale and future IMU measurement rejection;
- configuration in `config/state_estimator.yaml`;
- independent and unified-simulation launch support.

The full Stage 6 workspace baseline is:

- 5 packages;
- 228 tests;
- 0 errors;
- 0 failures;
- 25 skipped.
