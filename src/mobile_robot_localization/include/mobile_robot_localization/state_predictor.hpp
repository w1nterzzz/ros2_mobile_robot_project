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

#ifndef MOBILE_ROBOT_LOCALIZATION__STATE_PREDICTOR_HPP_
#define MOBILE_ROBOT_LOCALIZATION__STATE_PREDICTOR_HPP_

#include <cmath>

#include "mobile_robot_localization/angle_utils.hpp"
#include "mobile_robot_localization/planar_state.hpp"

namespace mobile_robot_localization
{

inline PlanarState predict_state(
  const PlanarState & state,
  const double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    return state;
  }

  PlanarState predicted = state;

  predicted.x += state.v * std::cos(state.yaw) * dt;
  predicted.y += state.v * std::sin(state.yaw) * dt;
  predicted.yaw = normalize_angle(
    state.yaw + state.yaw_rate * dt);

  return predicted;
}

}  // namespace mobile_robot_localization

#endif  // MOBILE_ROBOT_LOCALIZATION__STATE_PREDICTOR_HPP_
