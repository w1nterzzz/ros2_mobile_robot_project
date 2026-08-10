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

#include <limits>

#include "mobile_robot_localization/measurement_fusion.hpp"

namespace
{

constexpr double kTolerance = 1.0e-12;

}  // namespace

TEST(MeasurementFusionTest, UsesWheelLinearVelocity)
{
  const auto fused =
    mobile_robot_localization::fuse_motion_measurements(
    1.2, 0.2, 1.0, 0.4, 1.0);

  EXPECT_DOUBLE_EQ(fused.v, 1.2);
}

TEST(MeasurementFusionTest, AveragesEqualVarianceYawRates)
{
  const auto fused =
    mobile_robot_localization::fuse_motion_measurements(
    1.2, 0.2, 0.04, 0.4, 0.04);

  EXPECT_NEAR(fused.yaw_rate, 0.3, kTolerance);
}

TEST(MeasurementFusionTest, FavorsLowerVarianceYawRate)
{
  const auto fused =
    mobile_robot_localization::fuse_motion_measurements(
    1.2, 1.0, 4.0, 3.0, 1.0);

  EXPECT_NEAR(fused.yaw_rate, 2.6, kTolerance);
}

TEST(MeasurementFusionTest, FallsBackToImuForZeroWheelVariance)
{
  const auto fused =
    mobile_robot_localization::fuse_motion_measurements(
    1.2, 0.2, 0.0, 0.4, 0.04);

  EXPECT_NEAR(fused.yaw_rate, 0.4, kTolerance);
}

TEST(MeasurementFusionTest, FallsBackToWheelForNegativeImuVariance)
{
  const auto fused =
    mobile_robot_localization::fuse_motion_measurements(
    1.2, 0.2, 0.04, 0.4, -0.04);

  EXPECT_NEAR(fused.yaw_rate, 0.2, kTolerance);
}

TEST(MeasurementFusionTest, FallsBackToImuForNanWheelVariance)
{
  const auto fused =
    mobile_robot_localization::fuse_motion_measurements(
    1.2,
    0.2,
    std::numeric_limits<double>::quiet_NaN(),
    0.4,
    0.04);

  EXPECT_NEAR(fused.yaw_rate, 0.4, kTolerance);
}

TEST(MeasurementFusionTest, FallsBackToWheelWhenBothVariancesInvalid)
{
  const auto fused =
    mobile_robot_localization::fuse_motion_measurements(
    1.2,
    0.2,
    0.0,
    0.4,
    std::numeric_limits<double>::quiet_NaN());

  EXPECT_NEAR(fused.yaw_rate, 0.2, kTolerance);
}

TEST(MeasurementFusionTest, UsesZeroForNanWheelLinearVelocity)
{
  const auto fused =
    mobile_robot_localization::fuse_motion_measurements(
    std::numeric_limits<double>::quiet_NaN(),
    0.2,
    0.04,
    0.4,
    0.04);

  EXPECT_DOUBLE_EQ(fused.v, 0.0);
}

TEST(MeasurementFusionTest, FallsBackToImuForNanWheelYawRate)
{
  const auto fused =
    mobile_robot_localization::fuse_motion_measurements(
    1.2,
    std::numeric_limits<double>::quiet_NaN(),
    0.04,
    0.4,
    0.04);

  EXPECT_NEAR(fused.yaw_rate, 0.4, kTolerance);
}

TEST(MeasurementFusionTest, FallsBackToWheelForInfiniteImuYawRate)
{
  const auto fused =
    mobile_robot_localization::fuse_motion_measurements(
    1.2,
    0.2,
    0.04,
    std::numeric_limits<double>::infinity(),
    0.04);

  EXPECT_NEAR(fused.yaw_rate, 0.2, kTolerance);
}

TEST(MeasurementFusionTest, UsesZeroWhenBothYawRatesInvalid)
{
  const auto fused =
    mobile_robot_localization::fuse_motion_measurements(
    1.2,
    std::numeric_limits<double>::quiet_NaN(),
    0.04,
    std::numeric_limits<double>::infinity(),
    0.04);

  EXPECT_DOUBLE_EQ(fused.yaw_rate, 0.0);
}
