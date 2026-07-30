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

#include "mobile_robot_localization/planar_estimator.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-12;

}  // namespace

TEST(PlanarEstimatorTest, StartsUninitializedWithZeroState)
{
  const mobile_robot_localization::PlanarEstimator estimator;

  EXPECT_FALSE(estimator.is_initialized());

  const auto & state = estimator.state();

  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.y, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.0);
  EXPECT_DOUBLE_EQ(state.v, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw_rate, 0.0);
}

TEST(PlanarEstimatorTest, InitializationSetsPoseAndZeroMotion)
{
  mobile_robot_localization::PlanarEstimator estimator;

  estimator.initialize(1.0, -2.0, 0.5);

  EXPECT_TRUE(estimator.is_initialized());

  const auto & state = estimator.state();

  EXPECT_DOUBLE_EQ(state.x, 1.0);
  EXPECT_DOUBLE_EQ(state.y, -2.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.5);
  EXPECT_DOUBLE_EQ(state.v, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw_rate, 0.0);
}

TEST(PlanarEstimatorTest, InitializationNormalizesYaw)
{
  mobile_robot_localization::PlanarEstimator estimator;

  estimator.initialize(0.0, 0.0, 1.5 * kPi);

  const auto & state = estimator.state();

  EXPECT_NEAR(state.yaw, -0.5 * kPi, kTolerance);
}

TEST(PlanarEstimatorTest, RejectsInvalidInitialState)
{
  mobile_robot_localization::PlanarEstimator estimator;

  estimator.initialize(
    std::numeric_limits<double>::quiet_NaN(),
    0.0,
    0.0);

  EXPECT_FALSE(estimator.is_initialized());

  const auto & state = estimator.state();

  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.y, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.0);
}

TEST(PlanarEstimatorTest, InvalidReinitializationPreservesState)
{
  mobile_robot_localization::PlanarEstimator estimator;

  estimator.initialize(1.0, -2.0, 0.5);
  estimator.initialize(
    9.0,
    8.0,
    std::numeric_limits<double>::infinity());

  EXPECT_TRUE(estimator.is_initialized());

  const auto & state = estimator.state();

  EXPECT_DOUBLE_EQ(state.x, 1.0);
  EXPECT_DOUBLE_EQ(state.y, -2.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.5);
}

TEST(PlanarEstimatorTest, UpdateBeforeInitializationDoesNothing)
{
  mobile_robot_localization::PlanarEstimator estimator;

  estimator.update(
    1.0,
    0.2,
    0.04,
    0.4,
    0.04,
    0.5);

  EXPECT_FALSE(estimator.is_initialized());

  const auto & state = estimator.state();

  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.y, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.0);
  EXPECT_DOUBLE_EQ(state.v, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw_rate, 0.0);
}

TEST(PlanarEstimatorTest, UpdateFusesMotionAndPredictsState)
{
  mobile_robot_localization::PlanarEstimator estimator;

  estimator.initialize(0.0, 0.0, 0.0);
  estimator.update(
    2.0,
    0.0,
    1.0,
    2.0,
    1.0,
    0.5);

  const auto & state = estimator.state();

  EXPECT_NEAR(state.x, 1.0, kTolerance);
  EXPECT_NEAR(state.y, 0.0, kTolerance);
  EXPECT_NEAR(state.yaw, 0.5, kTolerance);
  EXPECT_NEAR(state.v, 2.0, kTolerance);
  EXPECT_NEAR(state.yaw_rate, 1.0, kTolerance);
}

TEST(PlanarEstimatorTest, NegativeTimeStepPreservesEntireState)
{
  mobile_robot_localization::PlanarEstimator estimator;

  estimator.initialize(1.0, -2.0, 0.3);
  estimator.update(
    4.0,
    1.0,
    1.0,
    3.0,
    1.0,
    -0.1);

  const auto & state = estimator.state();

  EXPECT_DOUBLE_EQ(state.x, 1.0);
  EXPECT_DOUBLE_EQ(state.y, -2.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.3);
  EXPECT_DOUBLE_EQ(state.v, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw_rate, 0.0);
}

TEST(PlanarEstimatorTest, NanTimeStepPreservesExistingState)
{
  mobile_robot_localization::PlanarEstimator estimator;

  estimator.initialize(0.0, 0.0, 0.0);
  estimator.update(
    2.0,
    0.0,
    1.0,
    2.0,
    1.0,
    0.5);

  const auto state_before_invalid_update = estimator.state();

  estimator.update(
    9.0,
    1.0,
    1.0,
    3.0,
    1.0,
    std::numeric_limits<double>::quiet_NaN());

  const auto & state = estimator.state();

  EXPECT_DOUBLE_EQ(state.x, state_before_invalid_update.x);
  EXPECT_DOUBLE_EQ(state.y, state_before_invalid_update.y);
  EXPECT_DOUBLE_EQ(state.yaw, state_before_invalid_update.yaw);
  EXPECT_DOUBLE_EQ(state.v, state_before_invalid_update.v);
  EXPECT_DOUBLE_EQ(
    state.yaw_rate,
    state_before_invalid_update.yaw_rate);
}
