// Copyright 2026 ros2_mobile_robot_project contributors
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

#ifndef MOBILE_ROBOT_CONTROL__WAYPOINT_SELECTOR_HPP_
#define MOBILE_ROBOT_CONTROL__WAYPOINT_SELECTOR_HPP_

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "mobile_robot_control/path_types.hpp"

namespace mobile_robot_control
{

struct WaypointSelection
{
  std::size_t target_index{0U};
  bool path_complete{false};
};

inline WaypointSelection select_target_waypoint(
  const Path & path,
  const Pose2D & pose,
  std::size_t current_index,
  double acceptance_radius)
{
  if (path.empty()) {
    throw std::invalid_argument("path must not be empty");
  }

  if (!std::isfinite(acceptance_radius) || acceptance_radius < 0.0) {
    throw std::invalid_argument(
            "acceptance radius must be finite and non-negative");
  }

  if (current_index >= path.size()) {
    throw std::out_of_range("current waypoint index is out of range");
  }

  const double radius_squared = acceptance_radius * acceptance_radius;
  std::size_t target_index = current_index;

  while (target_index < path.size()) {
    const double dx = path[target_index].x - pose.x;
    const double dy = path[target_index].y - pose.y;
    const double distance_squared = dx * dx + dy * dy;

    if (distance_squared > radius_squared) {
      return WaypointSelection{target_index, false};
    }

    ++target_index;
  }

  return WaypointSelection{path.size() - 1U, true};
}

}  // namespace mobile_robot_control

#endif  // MOBILE_ROBOT_CONTROL__WAYPOINT_SELECTOR_HPP_
