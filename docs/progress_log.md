# Progress Log

## 2026-07-25 — Stage 0 completed

- Installed Parallels Desktop on an Apple M4 Mac.
- Installed Ubuntu 24.04.4 LTS ARM64.
- Configured approximately 6 GiB of virtual-machine memory.
- Expanded the virtual disk to 128 GiB.
- Verified networking, shared folders and package sources.
- Created recoverable virtual-machine snapshots.

## 2026-07-25 — Stage 1 completed

- Practiced Linux paths, files, directories and text editing.
- Learned basic permissions, processes and environment variables.
- Installed software with APT.
- Installed and configured Git.
- Practiced staging, commits, branches, merging and remote repositories.
- Generated an Ed25519 SSH key for GitHub.
- Configured GitHub SSH access through port 443.
- Successfully pushed the practice repository to GitHub.

## 2026-07-25 — Stage 2 completed

- Verified the `en_US.UTF-8` locale.
- Enabled NTP and confirmed successful system-clock synchronization.
- Verified Ubuntu Noble package sources and the ARM64 architecture.
- Installed the official ROS2 APT source package `1.2.0~noble`.
- Installed ROS2 Jazzy Desktop `0.11.0-1noble.20260615.092556`.
- Installed ROS development tools and initialized rosdep.
- Configured new Bash terminals to load `/opt/ros/jazzy/setup.bash`.
- Verified C++ and Python ROS2 nodes using the `/chatter` topic.
- Verified RViz2 with OpenGL 4 rendering.
- Installed `ros-jazzy-ros-gz` version `1.0.22-1noble.20260615.095917`.
- Verified Gazebo Harmonic / Gazebo Sim `8.11.0`.
- Verified communication from headless Gazebo to ROS2 by bridging `/clock`.

### Gazebo GUI compatibility

The Gazebo physics server and ROS2 bridge work correctly. GUI testing identified a graphics compatibility issue with the Parallels `virgl` virtual GPU:

- The default Ogre2 renderer crashed while compiling a pixel shader.
- The Ogre renderer could flicker or become unresponsive with its default graphics settings.
- Mesa software rendering was not responsive with the current virtual-machine resources.

Stable interactive operation was verified using:

- The Ogre renderer.
- Shadows disabled in `worlds/empty_no_shadows.sdf`.
- Mesa OpenGL 3.3 compatibility mode.
- The repeatable launcher `scripts/run_gazebo_gui.sh`.

Two consecutive interactive tests completed successfully, including simulation playback, camera movement, zooming and window resizing. Headless Gazebo remains available for automated tests.

## 2026-08-10 — Stage 6 implementation and integration completed

- Added simulated IMU measurements with configurable noise and ROS bridging.
- Added the `mobile_robot_localization` package.
- Implemented the planar state model `[x, y, yaw, v, yaw_rate]`.
- Implemented angle normalization, state prediction and invalid time-step handling.
- Implemented inverse-variance fusion of wheel and IMU yaw-rate measurements.
- Added fallback behavior for invalid measurements and variances.
- Implemented estimator initialization and fused state updates.
- Routed raw Gazebo wheel odometry to `/wheel/odom`.
- Published fused odometry on `/odom` with configurable non-zero covariance.
- Published the `odom -> base_footprint` transform from the estimator.
- Removed the conflicting Gazebo odometry TF bridge.
- Added configurable topics, frames, time-step limits and output covariance.
- Added IMU timestamp tracking and stale/future measurement rejection.
- Added standalone configuration and launch files for the estimator.
- Integrated the estimator into the unified simulation launch.
- Verified the complete `/wheel/odom + /imu/data -> /odom -> controller -> /cmd_vel` chain.
- Completed the default path with a final position error of approximately `0.1413 m`, within the `0.15 m` acceptance radius.
- Verified that the controller publishes a zero velocity command after completion.
- Passed the full workspace regression baseline: 5 packages, 228 tests, 0 errors, 0 failures and 25 skipped.

## Next step

Complete the final Stage 6 repository verification and prepare the branch for integration.
