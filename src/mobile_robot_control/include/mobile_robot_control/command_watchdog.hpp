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

#ifndef MOBILE_ROBOT_CONTROL__COMMAND_WATCHDOG_HPP_
#define MOBILE_ROBOT_CONTROL__COMMAND_WATCHDOG_HPP_

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace mobile_robot_control
{

struct VelocityCommand
{
  double linear_velocity{0.0};
  double angular_velocity{0.0};
};

enum class WatchdogState
{
  kNoCommand,
  kActive,
  kTimedOut,
  kNoPublisher,
  kMultiplePublishers,
};

struct WatchdogResult
{
  VelocityCommand command;
  WatchdogState state{WatchdogState::kNoCommand};
};

class CommandWatchdog
{
public:
  explicit CommandWatchdog(double command_timeout)
  : command_timeout_(command_timeout)
  {
    if (!std::isfinite(command_timeout_) ||
      command_timeout_ <= 0.0)
    {
      throw std::invalid_argument(
              "command_timeout must be finite and positive");
    }
  }

  void update_command(
    const VelocityCommand & command,
    double receipt_time)
  {
    if (!std::isfinite(command.linear_velocity) ||
      !std::isfinite(command.angular_velocity) ||
      !std::isfinite(receipt_time))
    {
      has_command_ = false;
      latest_command_ = VelocityCommand{};
      return;
    }

    latest_command_ = command;
    last_command_time_ = receipt_time;
    has_command_ = true;
  }

  WatchdogResult evaluate(
    double current_time,
    std::size_t publisher_count) const
  {
    if (publisher_count == 0U) {
      return stopped(WatchdogState::kNoPublisher);
    }

    if (publisher_count > 1U) {
      return stopped(WatchdogState::kMultiplePublishers);
    }

    if (!has_command_) {
      return stopped(WatchdogState::kNoCommand);
    }

    if (!std::isfinite(current_time) ||
      current_time < last_command_time_ ||
      current_time - last_command_time_ > command_timeout_)
    {
      return stopped(WatchdogState::kTimedOut);
    }

    return WatchdogResult{
      latest_command_,
      WatchdogState::kActive,
    };
  }

private:
  static WatchdogResult stopped(WatchdogState state)
  {
    return WatchdogResult{
      VelocityCommand{},
      state,
    };
  }

  double command_timeout_;
  VelocityCommand latest_command_;
  double last_command_time_{0.0};
  bool has_command_{false};
};

}  // namespace mobile_robot_control

#endif  // MOBILE_ROBOT_CONTROL__COMMAND_WATCHDOG_HPP_
