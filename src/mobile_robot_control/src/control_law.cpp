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

#include "mobile_robot_control/control_law.hpp"

#include <cmath>
#include <stdexcept>

#include "mobile_robot_control/angle_utils.hpp"

namespace mobile_robot_control
{

ControlCommand compute_control_command(
  const Pose2D & pose,
  const Waypoint & target,
  const ControllerGains & gains)
{
  if (!std::isfinite(gains.linear) || gains.linear < 0.0) {
    throw std::invalid_argument(
            "linear gain must be finite and non-negative");
  }

  if (!std::isfinite(gains.angular) || gains.angular < 0.0) {
    throw std::invalid_argument(
            "angular gain must be finite and non-negative");
  }

  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double distance_error = std::hypot(dx, dy);

  if (distance_error == 0.0) {
    return ControlCommand{};
  }

  const double target_heading = std::atan2(dy, dx);
  const double heading_error =
    normalize_angle(target_heading - pose.yaw);

  return ControlCommand{
    gains.linear * distance_error,
    gains.angular * heading_error,
    distance_error,
    heading_error,
  };
}

}  // namespace mobile_robot_control
