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

#ifndef MOBILE_ROBOT_CONTROL__CONTROL_LAW_HPP_
#define MOBILE_ROBOT_CONTROL__CONTROL_LAW_HPP_

#include "mobile_robot_control/path_types.hpp"

namespace mobile_robot_control
{

struct ControllerGains
{
  double linear{1.0};
  double angular{1.0};
};

struct CommandLimits
{
  double max_linear_velocity{0.5};
  double max_angular_velocity{1.0};
  double max_heading_error_for_translation{1.0};
};

struct ControlCommand
{
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double distance_error{0.0};
  double heading_error{0.0};
};

ControlCommand compute_control_command(
  const Pose2D & pose,
  const Waypoint & target,
  const ControllerGains & gains);

ControlCommand apply_command_limits(
  const ControlCommand & command,
  const CommandLimits & limits);

}  // namespace mobile_robot_control

#endif  // MOBILE_ROBOT_CONTROL__CONTROL_LAW_HPP_
