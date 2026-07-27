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

#include <algorithm>
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

ControlCommand apply_command_limits(
  const ControlCommand & command,
  const CommandLimits & limits)
{
  constexpr double pi = 3.14159265358979323846;

  if (!std::isfinite(limits.max_linear_velocity) ||
    limits.max_linear_velocity < 0.0)
  {
    throw std::invalid_argument(
            "maximum linear velocity must be finite and non-negative");
  }

  if (!std::isfinite(limits.max_angular_velocity) ||
    limits.max_angular_velocity < 0.0)
  {
    throw std::invalid_argument(
            "maximum angular velocity must be finite and non-negative");
  }

  if (!std::isfinite(limits.max_heading_error_for_translation) ||
    limits.max_heading_error_for_translation < 0.0 ||
    limits.max_heading_error_for_translation > pi)
  {
    throw std::invalid_argument(
            "translation heading threshold must be in [0, pi]");
  }

  if (!std::isfinite(command.linear_velocity) ||
    !std::isfinite(command.angular_velocity) ||
    !std::isfinite(command.distance_error) ||
    !std::isfinite(command.heading_error))
  {
    throw std::invalid_argument("control command values must be finite");
  }

  ControlCommand limited = command;

  limited.linear_velocity = std::clamp(
    command.linear_velocity,
    0.0,
    limits.max_linear_velocity);

  limited.angular_velocity = std::clamp(
    command.angular_velocity,
    -limits.max_angular_velocity,
    limits.max_angular_velocity);

  if (std::abs(command.heading_error) >
    limits.max_heading_error_for_translation)
  {
    limited.linear_velocity = 0.0;
  }

  return limited;
}

}  // namespace mobile_robot_control
