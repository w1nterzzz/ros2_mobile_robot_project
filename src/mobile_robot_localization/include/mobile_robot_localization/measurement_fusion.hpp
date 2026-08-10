// Copyright 2026 Yang Yuexiang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOBILE_ROBOT_LOCALIZATION__MEASUREMENT_FUSION_HPP_
#define MOBILE_ROBOT_LOCALIZATION__MEASUREMENT_FUSION_HPP_
#include <cmath>

namespace mobile_robot_localization
{

struct MotionEstimate
{
  double v{0.0};
  double yaw_rate{0.0};
};

inline bool is_valid_variance(const double variance)
{
  return std::isfinite(variance) && variance > 0.0;
}

inline MotionEstimate fuse_motion_measurements(
  const double wheel_linear_velocity,
  const double wheel_yaw_rate,
  const double wheel_yaw_rate_variance,
  const double imu_yaw_rate,
  const double imu_yaw_rate_variance)
{
  double safe_linear_velocity = 0.0;

  if (std::isfinite(wheel_linear_velocity)) {
    safe_linear_velocity = wheel_linear_velocity;
  }

  const bool wheel_yaw_rate_is_valid =
    std::isfinite(wheel_yaw_rate);
  const bool imu_yaw_rate_is_valid =
    std::isfinite(imu_yaw_rate);

  if (!wheel_yaw_rate_is_valid && !imu_yaw_rate_is_valid) {
    return MotionEstimate{
      safe_linear_velocity,
      0.0};
  }

  if (!wheel_yaw_rate_is_valid) {
    return MotionEstimate{
      safe_linear_velocity,
      imu_yaw_rate};
  }

  if (!imu_yaw_rate_is_valid) {
    return MotionEstimate{
      safe_linear_velocity,
      wheel_yaw_rate};
  }
  const bool wheel_variance_is_valid =
    is_valid_variance(wheel_yaw_rate_variance);
  const bool imu_variance_is_valid =
    is_valid_variance(imu_yaw_rate_variance);

  if (!wheel_variance_is_valid && !imu_variance_is_valid) {
    return MotionEstimate{
      safe_linear_velocity,
      wheel_yaw_rate};
  }

  if (!wheel_variance_is_valid) {
    return MotionEstimate{
      safe_linear_velocity,
      imu_yaw_rate};
  }

  if (!imu_variance_is_valid) {
    return MotionEstimate{
      safe_linear_velocity,
      wheel_yaw_rate};
  }

  const double wheel_weight = 1.0 / wheel_yaw_rate_variance;
  const double imu_weight = 1.0 / imu_yaw_rate_variance;

  const double fused_yaw_rate =
    (wheel_weight * wheel_yaw_rate +
    imu_weight * imu_yaw_rate) /
    (wheel_weight + imu_weight);

  return MotionEstimate{
    safe_linear_velocity,
    fused_yaw_rate};
}

}  // namespace mobile_robot_localization

#endif  // MOBILE_ROBOT_LOCALIZATION__MEASUREMENT_FUSION_HPP_
