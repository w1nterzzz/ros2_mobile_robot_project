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

#ifndef MOBILE_ROBOT_LOCALIZATION__PLANAR_ESTIMATOR_HPP_
#define MOBILE_ROBOT_LOCALIZATION__PLANAR_ESTIMATOR_HPP_

#include <cmath>

#include "mobile_robot_localization/angle_utils.hpp"
#include "mobile_robot_localization/measurement_fusion.hpp"
#include "mobile_robot_localization/planar_state.hpp"
#include "mobile_robot_localization/state_predictor.hpp"

namespace mobile_robot_localization
{

class PlanarEstimator
{
public:
  bool is_initialized() const
  {
    return initialized_;
  }

  const PlanarState & state() const
  {
    return state_;
  }

  void initialize(
    const double x,
    const double y,
    const double yaw)
  {
    const bool pose_is_finite =
      std::isfinite(x) &&
      std::isfinite(y) &&
      std::isfinite(yaw);

    if (!pose_is_finite) {
      return;
    }
    state_ = PlanarState{
      x,
      y,
      normalize_angle(yaw),
      0.0,
      0.0};

    initialized_ = true;
  }

  void update(
    const double wheel_linear_velocity,
    const double wheel_yaw_rate,
    const double wheel_yaw_rate_variance,
    const double imu_yaw_rate,
    const double imu_yaw_rate_variance,
    const double dt)
  {
    const bool time_step_is_valid =
      std::isfinite(dt) && dt > 0.0;

    if (!initialized_ || !time_step_is_valid) {
      return;
    }

    const MotionEstimate motion =
      fuse_motion_measurements(
      wheel_linear_velocity,
      wheel_yaw_rate,
      wheel_yaw_rate_variance,
      imu_yaw_rate,
      imu_yaw_rate_variance);

    state_.v = motion.v;
    state_.yaw_rate = motion.yaw_rate;
    state_ = predict_state(state_, dt);
  }

private:
  PlanarState state_{};
  bool initialized_{false};
};

}  // namespace mobile_robot_localization

#endif  // MOBILE_ROBOT_LOCALIZATION__PLANAR_ESTIMATOR_HPP_
