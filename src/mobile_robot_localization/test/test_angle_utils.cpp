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

#include <gtest/gtest.h>

#include "mobile_robot_localization/angle_utils.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-12;

}  // namespace

TEST(AngleUtilsTest, KeepsZeroUnchanged)
{
  EXPECT_NEAR(
    mobile_robot_localization::normalize_angle(0.0),
    0.0,
    kTolerance);
}

TEST(AngleUtilsTest, WrapsPositiveFullTurnToZero)
{
  EXPECT_NEAR(
    mobile_robot_localization::normalize_angle(2.0 * kPi),
    0.0,
    kTolerance);
}

TEST(AngleUtilsTest, WrapsNegativeFullTurnToZero)
{
  EXPECT_NEAR(
    mobile_robot_localization::normalize_angle(-2.0 * kPi),
    0.0,
    kTolerance);
}

TEST(AngleUtilsTest, MapsPositivePiToNegativePi)
{
  EXPECT_NEAR(
    mobile_robot_localization::normalize_angle(kPi),
    -kPi,
    kTolerance);
}

TEST(AngleUtilsTest, WrapsAngleAbovePi)
{
  EXPECT_NEAR(
    mobile_robot_localization::normalize_angle(1.5 * kPi),
    -0.5 * kPi,
    kTolerance);
}
