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

#include <stdexcept>

#include "mobile_robot_control/waypoint_selector.hpp"

TEST(WaypointSelector, KeepsCurrentWaypointWhenOutsideRadius)
{
  const mobile_robot_control::Path path{{0.0, 0.0}, {1.0, 0.0}};
  const mobile_robot_control::Pose2D pose{-0.5, 0.0, 0.0};

  const auto selection =
    mobile_robot_control::select_target_waypoint(path, pose, 0U, 0.1);

  EXPECT_EQ(selection.target_index, 0U);
  EXPECT_FALSE(selection.path_complete);
}

TEST(WaypointSelector, AdvancesAfterReachingCurrentWaypoint)
{
  const mobile_robot_control::Path path{{0.0, 0.0}, {1.0, 0.0}};
  const mobile_robot_control::Pose2D pose{0.0, 0.0, 0.0};

  const auto selection =
    mobile_robot_control::select_target_waypoint(path, pose, 0U, 0.1);

  EXPECT_EQ(selection.target_index, 1U);
  EXPECT_FALSE(selection.path_complete);
}

TEST(WaypointSelector, SkipsConsecutiveReachedWaypoints)
{
  const mobile_robot_control::Path path{
    {0.0, 0.0},
    {0.05, 0.0},
    {1.0, 0.0},
  };
  const mobile_robot_control::Pose2D pose{0.0, 0.0, 0.0};

  const auto selection =
    mobile_robot_control::select_target_waypoint(path, pose, 0U, 0.1);

  EXPECT_EQ(selection.target_index, 2U);
  EXPECT_FALSE(selection.path_complete);
}

TEST(WaypointSelector, ReportsCompletionAtFinalWaypoint)
{
  const mobile_robot_control::Path path{{0.0, 0.0}, {1.0, 0.0}};
  const mobile_robot_control::Pose2D pose{1.0, 0.0, 0.0};

  const auto selection =
    mobile_robot_control::select_target_waypoint(path, pose, 1U, 0.1);

  EXPECT_EQ(selection.target_index, 1U);
  EXPECT_TRUE(selection.path_complete);
}

TEST(WaypointSelector, RejectsEmptyPath)
{
  const mobile_robot_control::Path path;
  const mobile_robot_control::Pose2D pose;

  EXPECT_THROW(
    mobile_robot_control::select_target_waypoint(path, pose, 0U, 0.1),
    std::invalid_argument);
}

TEST(WaypointSelector, RejectsInvalidConfiguration)
{
  const mobile_robot_control::Path path{{0.0, 0.0}};
  const mobile_robot_control::Pose2D pose;

  EXPECT_THROW(
    mobile_robot_control::select_target_waypoint(path, pose, 1U, 0.1),
    std::out_of_range);
  EXPECT_THROW(
    mobile_robot_control::select_target_waypoint(path, pose, 0U, -0.1),
    std::invalid_argument);
}
