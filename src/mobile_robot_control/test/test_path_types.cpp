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

#include "mobile_robot_control/path_types.hpp"

TEST(PathTypes, PoseDefaultsToOrigin)
{
  const mobile_robot_control::Pose2D pose;

  EXPECT_DOUBLE_EQ(pose.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.yaw, 0.0);
}

TEST(PathTypes, SupportsAggregateInitialization)
{
  const mobile_robot_control::Pose2D pose{1.0, -2.0, 0.5};
  const mobile_robot_control::Waypoint waypoint{3.0, 4.0};

  EXPECT_DOUBLE_EQ(pose.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.y, -2.0);
  EXPECT_DOUBLE_EQ(pose.yaw, 0.5);
  EXPECT_DOUBLE_EQ(waypoint.x, 3.0);
  EXPECT_DOUBLE_EQ(waypoint.y, 4.0);
}

TEST(PathTypes, PathPreservesWaypointOrder)
{
  const mobile_robot_control::Path path{
    {0.0, 0.0},
    {1.0, 0.0},
    {1.0, 2.0},
  };

  ASSERT_EQ(path.size(), 3U);
  EXPECT_DOUBLE_EQ(path[0].x, 0.0);
  EXPECT_DOUBLE_EQ(path[0].y, 0.0);
  EXPECT_DOUBLE_EQ(path[1].x, 1.0);
  EXPECT_DOUBLE_EQ(path[1].y, 0.0);
  EXPECT_DOUBLE_EQ(path[2].x, 1.0);
  EXPECT_DOUBLE_EQ(path[2].y, 2.0);
}
