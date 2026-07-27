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

#include "mobile_robot_control/angle_utils.hpp"

#include <cmath>

namespace mobile_robot_control
{

double normalize_angle(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double two_pi = 2.0 * pi;

  double normalized = std::fmod(angle + pi, two_pi);

  if (normalized < 0.0) {
    normalized += two_pi;
  }

  return normalized - pi;
}

}  // namespace mobile_robot_control
