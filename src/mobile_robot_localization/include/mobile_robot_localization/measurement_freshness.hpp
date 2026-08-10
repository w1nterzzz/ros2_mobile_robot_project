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

#ifndef MOBILE_ROBOT_LOCALIZATION__MEASUREMENT_FRESHNESS_HPP_
#define MOBILE_ROBOT_LOCALIZATION__MEASUREMENT_FRESHNESS_HPP_

#include <cmath>

namespace mobile_robot_localization
{

inline bool is_measurement_fresh(
  const double measurement_time,
  const double reference_time,
  const double timeout)
{
  if (
    !std::isfinite(measurement_time) ||
    !std::isfinite(reference_time) ||
    !std::isfinite(timeout) ||
    timeout <= 0.0)
  {
    return false;
  }

  const double measurement_age =
    reference_time - measurement_time;

  return
    measurement_age >= 0.0 &&
    measurement_age <= timeout;
}

}  // namespace mobile_robot_localization

#endif  // MOBILE_ROBOT_LOCALIZATION__MEASUREMENT_FRESHNESS_HPP_
