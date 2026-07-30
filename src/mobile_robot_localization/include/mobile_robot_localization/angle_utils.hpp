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

#ifndef MOBILE_ROBOT_LOCALIZATION__ANGLE_UTILS_HPP_
#define MOBILE_ROBOT_LOCALIZATION__ANGLE_UTILS_HPP_

#include <cmath>

namespace mobile_robot_localization
{

inline double normalize_angle(const double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;

  double normalized = std::fmod(angle + kPi, kTwoPi);

  if (normalized < 0.0) {
    normalized += kTwoPi;
  }

  return normalized - kPi;
}

}  // namespace mobile_robot_localization

#endif  // MOBILE_ROBOT_LOCALIZATION__ANGLE_UTILS_HPP_
