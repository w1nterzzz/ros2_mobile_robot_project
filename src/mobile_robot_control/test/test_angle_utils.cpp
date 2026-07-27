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

#include <gtest/gtest.h>

#include "mobile_robot_control/angle_utils.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-12;

double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

}  // namespace

TEST(NormalizeAngle, KeepsAnglesInsideRange)
{
  EXPECT_NEAR(mobile_robot_control::normalize_angle(0.0), 0.0, kTolerance);
  EXPECT_NEAR(mobile_robot_control::normalize_angle(0.5), 0.5, kTolerance);
  EXPECT_NEAR(mobile_robot_control::normalize_angle(-0.5), -0.5, kTolerance);
}

TEST(NormalizeAngle, HandlesRangeBoundaries)
{
  EXPECT_NEAR(
    mobile_robot_control::normalize_angle(kPi),
    -kPi,
    kTolerance);
  EXPECT_NEAR(
    mobile_robot_control::normalize_angle(-kPi),
    -kPi,
    kTolerance);
}

TEST(NormalizeAngle, ChoosesShortestDirectionAcrossBoundary)
{
  const double positive_crossing =
    degrees_to_radians(-179.0) - degrees_to_radians(179.0);
  const double negative_crossing =
    degrees_to_radians(179.0) - degrees_to_radians(-179.0);

  EXPECT_NEAR(
    mobile_robot_control::normalize_angle(positive_crossing),
    degrees_to_radians(2.0),
    kTolerance);
  EXPECT_NEAR(
    mobile_robot_control::normalize_angle(negative_crossing),
    degrees_to_radians(-2.0),
    kTolerance);
}

TEST(NormalizeAngle, HandlesMultipleRevolutions)
{
  EXPECT_NEAR(
    mobile_robot_control::normalize_angle(4.0 * kPi + 0.25),
    0.25,
    kTolerance);
  EXPECT_NEAR(
    mobile_robot_control::normalize_angle(-4.0 * kPi - 0.25),
    -0.25,
    kTolerance);
}
