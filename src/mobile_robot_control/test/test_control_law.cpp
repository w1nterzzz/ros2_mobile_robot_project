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

#include <cmath>
#include <limits>
#include <stdexcept>

#include "mobile_robot_control/control_law.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-12;

double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

}  // namespace

TEST(ControlLaw, DrivesStraightTowardTargetAhead)
{
  const mobile_robot_control::Pose2D pose{0.0, 0.0, 0.0};
  const mobile_robot_control::Waypoint target{2.0, 0.0};
  const mobile_robot_control::ControllerGains gains{0.5, 2.0};

  const auto command =
    mobile_robot_control::compute_control_command(pose, target, gains);

  EXPECT_NEAR(command.distance_error, 2.0, kTolerance);
  EXPECT_NEAR(command.heading_error, 0.0, kTolerance);
  EXPECT_NEAR(command.linear_velocity, 1.0, kTolerance);
  EXPECT_NEAR(command.angular_velocity, 0.0, kTolerance);
}

TEST(ControlLaw, TurnsLeftTowardTarget)
{
  const mobile_robot_control::Pose2D pose{0.0, 0.0, 0.0};
  const mobile_robot_control::Waypoint target{0.0, 1.0};
  const mobile_robot_control::ControllerGains gains{1.0, 2.0};

  const auto command =
    mobile_robot_control::compute_control_command(pose, target, gains);

  EXPECT_NEAR(command.distance_error, 1.0, kTolerance);
  EXPECT_NEAR(command.heading_error, kPi / 2.0, kTolerance);
  EXPECT_NEAR(command.linear_velocity, 1.0, kTolerance);
  EXPECT_NEAR(command.angular_velocity, kPi, kTolerance);
}

TEST(ControlLaw, NormalizesHeadingAcrossAngleBoundary)
{
  const double target_angle = degrees_to_radians(-179.0);
  const mobile_robot_control::Pose2D pose{
    0.0, 0.0, degrees_to_radians(179.0)};
  const mobile_robot_control::Waypoint target{
    std::cos(target_angle), std::sin(target_angle)};
  const mobile_robot_control::ControllerGains gains{1.0, 1.0};

  const auto command =
    mobile_robot_control::compute_control_command(pose, target, gains);

  EXPECT_NEAR(
    command.heading_error,
    degrees_to_radians(2.0),
    kTolerance);
  EXPECT_NEAR(
    command.angular_velocity,
    degrees_to_radians(2.0),
    kTolerance);
}

TEST(ControlLaw, ReturnsZeroAtTarget)
{
  const mobile_robot_control::Pose2D pose{1.0, 2.0, 0.7};
  const mobile_robot_control::Waypoint target{1.0, 2.0};
  const mobile_robot_control::ControllerGains gains{1.0, 2.0};

  const auto command =
    mobile_robot_control::compute_control_command(pose, target, gains);

  EXPECT_DOUBLE_EQ(command.distance_error, 0.0);
  EXPECT_DOUBLE_EQ(command.heading_error, 0.0);
  EXPECT_DOUBLE_EQ(command.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_velocity, 0.0);
}

TEST(ControlLaw, RejectsInvalidGains)
{
  const mobile_robot_control::Pose2D pose;
  const mobile_robot_control::Waypoint target{1.0, 0.0};

  EXPECT_THROW(
    mobile_robot_control::compute_control_command(
      pose, target, {-1.0, 1.0}),
    std::invalid_argument);

  EXPECT_THROW(
    mobile_robot_control::compute_control_command(
      pose, target,
      {1.0, std::numeric_limits<double>::infinity()}),
    std::invalid_argument);
}
