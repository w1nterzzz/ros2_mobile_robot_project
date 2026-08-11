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

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "mobile_robot_control/command_watchdog.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_robot_control
{

class CmdVelSafetyNode : public rclcpp::Node
{
public:
  CmdVelSafetyNode()
  : Node("cmd_vel_safety")
  {
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "cmd_vel");

    const auto output_topic =
      declare_parameter<std::string>(
      "output_topic",
      "actuator/cmd_vel");

    const auto command_timeout =
      declare_parameter<double>("command_timeout", 0.5);

    const auto publish_rate =
      declare_parameter<double>("publish_rate", 20.0);

    if (input_topic.empty() || output_topic.empty()) {
      throw std::invalid_argument(
              "input_topic and output_topic must not be empty");
    }

    if (
      !std::isfinite(command_timeout) ||
      command_timeout <= 0.0)
    {
      throw std::invalid_argument(
              "command_timeout must be finite and positive");
    }

    if (
      !std::isfinite(publish_rate) ||
      publish_rate <= 0.0)
    {
      throw std::invalid_argument(
              "publish_rate must be finite and positive");
    }

    watchdog_ =
      std::make_unique<CommandWatchdog>(command_timeout);

    command_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(
      output_topic,
      10);

    command_subscription_ =
      create_subscription<geometry_msgs::msg::Twist>(
      input_topic,
      10,
      [this](geometry_msgs::msg::Twist::SharedPtr message) {
        receive_command(*message);
      });

    const auto timer_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));

    publish_timer_ = create_wall_timer(
      timer_period,
      [this]() {
        publish_safe_command();
      });

    publish_stop_command();

    RCLCPP_INFO(
      get_logger(),
      "Velocity safety node started: %s -> %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

  void publish_stop_command()
  {
    if (!command_publisher_) {
      return;
    }

    geometry_msgs::msg::Twist stop_command;
    command_publisher_->publish(stop_command);
  }

private:
  static double steady_time_seconds()
  {
    const auto elapsed =
      std::chrono::steady_clock::now().time_since_epoch();

    return std::chrono::duration<double>(elapsed).count();
  }

  void receive_command(
    const geometry_msgs::msg::Twist & message)
  {
    const VelocityCommand command{
      message.linear.x,
      message.angular.z,
    };

    watchdog_->update_command(
      command,
      steady_time_seconds());
  }

  void publish_safe_command()
  {
    const auto publisher_count =
      command_subscription_->get_publisher_count();

    const auto result = watchdog_->evaluate(
      steady_time_seconds(),
      publisher_count);

    geometry_msgs::msg::Twist output;
    output.linear.x = result.command.linear_velocity;
    output.angular.z = result.command.angular_velocity;

    command_publisher_->publish(output);
    log_state_transition(result.state);
  }

  void log_state_transition(WatchdogState state)
  {
    if (has_previous_state_ && state == previous_state_) {
      return;
    }

    previous_state_ = state;
    has_previous_state_ = true;

    switch (state) {
      case WatchdogState::kActive:
        RCLCPP_INFO(
          get_logger(),
          "Passing fresh velocity commands");
        break;

      case WatchdogState::kNoCommand:
        RCLCPP_WARN(
          get_logger(),
          "No velocity command received; publishing zero");
        break;

      case WatchdogState::kTimedOut:
        RCLCPP_WARN(
          get_logger(),
          "Velocity command timed out; publishing zero");
        break;

      case WatchdogState::kNoPublisher:
        RCLCPP_INFO(
          get_logger(),
          "No velocity command publisher; publishing zero");
        break;

      case WatchdogState::kMultiplePublishers:
        RCLCPP_ERROR(
          get_logger(),
          "Multiple velocity publishers detected; publishing zero");
        break;
    }
  }

  std::unique_ptr<CommandWatchdog> watchdog_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    command_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    command_subscription_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  WatchdogState previous_state_{WatchdogState::kNoCommand};
  bool has_previous_state_{false};
};

}  // namespace mobile_robot_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<mobile_robot_control::CmdVelSafetyNode>();

  std::weak_ptr<mobile_robot_control::CmdVelSafetyNode> weak_node = node;

  rclcpp::on_shutdown(
    [weak_node]() {
      if (const auto locked_node = weak_node.lock()) {
        locked_node->publish_stop_command();
      }
    });

  rclcpp::spin(node);

  if (rclcpp::ok()) {
    node->publish_stop_command();
    rclcpp::shutdown();
  }

  return 0;
}
