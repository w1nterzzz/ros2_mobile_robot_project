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

#ifndef MOBILE_ROBOT_CONTROL__PATH_TYPES_HPP_
#define MOBILE_ROBOT_CONTROL__PATH_TYPES_HPP_

#include <vector>

namespace mobile_robot_control
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Waypoint
{
  double x{0.0};
  double y{0.0};
};

using Path = std::vector<Waypoint>;

}  // namespace mobile_robot_control

#endif  // MOBILE_ROBOT_CONTROL__PATH_TYPES_HPP_
