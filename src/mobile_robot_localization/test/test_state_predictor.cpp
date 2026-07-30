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

#include "mobile_robot_localization/state_predictor.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-12;

}  // namespace

TEST(StatePredictorTest, ZeroTimeStepLeavesStateUnchanged)
{
  const mobile_robot_localization::PlanarState state{
    1.0, -2.0, 0.3, 2.0, 0.5};

  const auto predicted =
    mobile_robot_localization::predict_state(state, 0.0);

  EXPECT_DOUBLE_EQ(predicted.x, state.x);
  EXPECT_DOUBLE_EQ(predicted.y, state.y);
  EXPECT_DOUBLE_EQ(predicted.yaw, state.yaw);
  EXPECT_DOUBLE_EQ(predicted.v, state.v);
  EXPECT_DOUBLE_EQ(predicted.yaw_rate, state.yaw_rate);
}

TEST(StatePredictorTest, ScalesMotionByTimeStep)
{
  const mobile_robot_localization::PlanarState state{
    1.0, -2.0, 0.0, 2.0, 0.5};

  const auto predicted =
    mobile_robot_localization::predict_state(state, 0.5);

  EXPECT_NEAR(predicted.x, 2.0, kTolerance);
  EXPECT_NEAR(predicted.y, -2.0, kTolerance);
  EXPECT_NEAR(predicted.yaw, 0.25, kTolerance);
  EXPECT_DOUBLE_EQ(predicted.v, state.v);
  EXPECT_DOUBLE_EQ(predicted.yaw_rate, state.yaw_rate);
}

TEST(StatePredictorTest, UsesCurrentHeadingForPlanarMotion)
{
  const mobile_robot_localization::PlanarState state{
    0.0, 0.0, 0.5 * kPi, 2.0, 0.0};

  const auto predicted =
    mobile_robot_localization::predict_state(state, 0.5);

  EXPECT_NEAR(predicted.x, 0.0, kTolerance);
  EXPECT_NEAR(predicted.y, 1.0, kTolerance);
  EXPECT_NEAR(predicted.yaw, 0.5 * kPi, kTolerance);
}

TEST(StatePredictorTest, NormalizesPredictedYaw)
{
  const mobile_robot_localization::PlanarState state{
    0.0, 0.0, 0.75 * kPi, 0.0, kPi};

  const auto predicted =
    mobile_robot_localization::predict_state(state, 0.5);

  EXPECT_NEAR(predicted.yaw, -0.75 * kPi, kTolerance);
}

TEST(StatePredictorTest, NegativeTimeStepLeavesStateUnchanged)
{
  const mobile_robot_localization::PlanarState state{
    1.0, -2.0, 0.3, 2.0, 0.5};

  const auto predicted =
    mobile_robot_localization::predict_state(state, -0.1);

  EXPECT_DOUBLE_EQ(predicted.x, state.x);
  EXPECT_DOUBLE_EQ(predicted.y, state.y);
  EXPECT_DOUBLE_EQ(predicted.yaw, state.yaw);
  EXPECT_DOUBLE_EQ(predicted.v, state.v);
  EXPECT_DOUBLE_EQ(predicted.yaw_rate, state.yaw_rate);
}

TEST(StatePredictorTest, NanTimeStepLeavesStateUnchanged)
{
  const mobile_robot_localization::PlanarState state{
    1.0, -2.0, 0.3, 2.0, 0.5};

  const auto predicted = mobile_robot_localization::predict_state(
    state, std::numeric_limits<double>::quiet_NaN());

  EXPECT_DOUBLE_EQ(predicted.x, state.x);
  EXPECT_DOUBLE_EQ(predicted.y, state.y);
  EXPECT_DOUBLE_EQ(predicted.yaw, state.yaw);
  EXPECT_DOUBLE_EQ(predicted.v, state.v);
  EXPECT_DOUBLE_EQ(predicted.yaw_rate, state.yaw_rate);
}

TEST(StatePredictorTest, InfiniteTimeStepLeavesStateUnchanged)
{
  const mobile_robot_localization::PlanarState state{
    1.0, -2.0, 0.3, 2.0, 0.5};

  const auto predicted = mobile_robot_localization::predict_state(
    state, std::numeric_limits<double>::infinity());

  EXPECT_DOUBLE_EQ(predicted.x, state.x);
  EXPECT_DOUBLE_EQ(predicted.y, state.y);
  EXPECT_DOUBLE_EQ(predicted.yaw, state.yaw);
  EXPECT_DOUBLE_EQ(predicted.v, state.v);
  EXPECT_DOUBLE_EQ(predicted.yaw_rate, state.yaw_rate);
}
