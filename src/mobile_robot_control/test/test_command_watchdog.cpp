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

#include "mobile_robot_control/command_watchdog.hpp"

namespace mobile_robot_control
{
namespace
{

void expect_zero_command(const WatchdogResult & result)
{
  EXPECT_DOUBLE_EQ(result.command.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(result.command.angular_velocity, 0.0);
}

TEST(CommandWatchdog, StartsStoppedWithoutCommand)
{
  CommandWatchdog watchdog(0.5);

  const auto result = watchdog.evaluate(0.0, 1U);

  EXPECT_EQ(result.state, WatchdogState::kNoCommand);
  expect_zero_command(result);
}

TEST(CommandWatchdog, PassesFreshCommandFromSinglePublisher)
{
  CommandWatchdog watchdog(0.5);
  watchdog.update_command(
    VelocityCommand{0.2, -0.1},
    1.0);

  const auto result = watchdog.evaluate(1.4, 1U);

  EXPECT_EQ(result.state, WatchdogState::kActive);
  EXPECT_DOUBLE_EQ(result.command.linear_velocity, 0.2);
  EXPECT_DOUBLE_EQ(result.command.angular_velocity, -0.1);
}

TEST(CommandWatchdog, StopsAfterCommandTimeout)
{
  CommandWatchdog watchdog(0.5);
  watchdog.update_command(
    VelocityCommand{0.2, -0.1},
    1.0);

  const auto result = watchdog.evaluate(1.6, 1U);

  EXPECT_EQ(result.state, WatchdogState::kTimedOut);
  expect_zero_command(result);
}

TEST(CommandWatchdog, StopsWhenPublisherDisappears)
{
  CommandWatchdog watchdog(0.5);
  watchdog.update_command(
    VelocityCommand{0.2, -0.1},
    1.0);

  const auto result = watchdog.evaluate(1.1, 0U);

  EXPECT_EQ(result.state, WatchdogState::kNoPublisher);
  expect_zero_command(result);
}

TEST(CommandWatchdog, StopsForMultiplePublishers)
{
  CommandWatchdog watchdog(0.5);
  watchdog.update_command(
    VelocityCommand{0.2, -0.1},
    1.0);

  const auto result = watchdog.evaluate(1.1, 2U);

  EXPECT_EQ(result.state, WatchdogState::kMultiplePublishers);
  expect_zero_command(result);
}

}  // namespace
}  // namespace mobile_robot_control
