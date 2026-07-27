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
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "mobile_robot_control/control_law.hpp"
#include "mobile_robot_control/path_types.hpp"
#include "mobile_robot_control/waypoint_selector.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

namespace mobile_robot_control
{

class PathTrackingControllerNode : public rclcpp::Node
{
public:
  PathTrackingControllerNode()
  : Node("path_tracking_controller")
  {
    declare_and_validate_parameters();

    cmd_vel_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(
        &PathTrackingControllerNode::odom_callback,
        this,
        std::placeholders::_1));

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(
        &PathTrackingControllerNode::control_timer_callback,
        this));

    RCLCPP_INFO(
      get_logger(),
      "Path tracking controller started; waiting for odometry");

    RCLCPP_INFO(
      get_logger(),
      "Gains: linear=%.3f, angular=%.3f",
      gains_.linear,
      gains_.angular);

    RCLCPP_INFO(
      get_logger(),
      "Limits: linear=%.3f m/s, angular=%.3f rad/s, "
      "translation heading threshold=%.3f rad",
      limits_.max_linear_velocity,
      limits_.max_angular_velocity,
      limits_.max_heading_error_for_translation);

    RCLCPP_INFO(
      get_logger(),
      "Waypoint acceptance radius: %.3f m",
      waypoint_acceptance_radius_);

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu waypoints: first=(%.3f, %.3f), final=(%.3f, %.3f)",
      path_.size(),
      path_.front().x,
      path_.front().y,
      path_.back().x,
      path_.back().y);
  }

  ~PathTrackingControllerNode() override
  {
    publish_stop_command();
  }

private:
  void declare_and_validate_parameters()
  {
    gains_.linear =
      declare_parameter<double>("linear_gain", 0.8);
    gains_.angular =
      declare_parameter<double>("angular_gain", 2.0);

    limits_.max_linear_velocity =
      declare_parameter<double>("max_linear_velocity", 0.4);
    limits_.max_angular_velocity =
      declare_parameter<double>("max_angular_velocity", 1.5);
    limits_.max_heading_error_for_translation =
      declare_parameter<double>(
      "max_heading_error_for_translation",
      1.0);

    waypoint_acceptance_radius_ =
      declare_parameter<double>("waypoint_acceptance_radius", 0.15);

    const auto waypoint_values =
      declare_parameter<std::vector<double>>(
      "waypoints",
      std::vector<double>{
        0.0, 0.0,
        1.0, 0.0,
        1.0, 1.0,
      });

    if (waypoint_values.empty() || waypoint_values.size() % 2U != 0U) {
      throw std::invalid_argument(
              "waypoints must contain x, y coordinate pairs");
    }

    for (std::size_t index = 0U;
      index < waypoint_values.size();
      index += 2U)
    {
      const double x = waypoint_values[index];
      const double y = waypoint_values[index + 1U];

      if (!std::isfinite(x) || !std::isfinite(y)) {
        throw std::invalid_argument(
                "waypoint coordinates must be finite");
      }

      path_.push_back(Waypoint{x, y});
    }

    constexpr double pi = 3.14159265358979323846;

    if (!std::isfinite(gains_.linear) || gains_.linear < 0.0) {
      throw std::invalid_argument(
              "linear_gain must be finite and non-negative");
    }

    if (!std::isfinite(gains_.angular) || gains_.angular < 0.0) {
      throw std::invalid_argument(
              "angular_gain must be finite and non-negative");
    }

    if (!std::isfinite(limits_.max_linear_velocity) ||
      limits_.max_linear_velocity < 0.0)
    {
      throw std::invalid_argument(
              "max_linear_velocity must be finite and non-negative");
    }

    if (!std::isfinite(limits_.max_angular_velocity) ||
      limits_.max_angular_velocity < 0.0)
    {
      throw std::invalid_argument(
              "max_angular_velocity must be finite and non-negative");
    }

    if (!std::isfinite(limits_.max_heading_error_for_translation) ||
      limits_.max_heading_error_for_translation < 0.0 ||
      limits_.max_heading_error_for_translation > pi)
    {
      throw std::invalid_argument(
              "max_heading_error_for_translation must be in [0, pi]");
    }

    if (!std::isfinite(waypoint_acceptance_radius_) ||
      waypoint_acceptance_radius_ <= 0.0)
    {
      throw std::invalid_argument(
              "waypoint_acceptance_radius must be finite and positive");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    current_pose_.x = message->pose.pose.position.x;
    current_pose_.y = message->pose.pose.position.y;
    current_pose_.yaw = tf2::getYaw(message->pose.pose.orientation);

    if (!has_odometry_) {
      has_odometry_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Received first odometry: x=%.3f m, y=%.3f m, yaw=%.3f rad",
        current_pose_.x,
        current_pose_.y,
        current_pose_.yaw);
    }
  }

  void control_timer_callback()
  {
    if (!has_odometry_ || path_complete_) {
      publish_stop_command();
      return;
    }

    const auto selection = select_target_waypoint(
      path_,
      current_pose_,
      target_index_,
      waypoint_acceptance_radius_);

    if (selection.target_index != target_index_) {
      target_index_ = selection.target_index;

      RCLCPP_INFO(
        get_logger(),
        "Advanced to waypoint %zu at (%.3f, %.3f)",
        target_index_,
        path_[target_index_].x,
        path_[target_index_].y);
    }

    if (selection.path_complete) {
      path_complete_ = true;
      publish_stop_command();

      RCLCPP_INFO(
        get_logger(),
        "Final waypoint reached; publishing stop command");
      return;
    }

    const auto raw_command = compute_control_command(
      current_pose_,
      path_[target_index_],
      gains_);

    const auto limited_command =
      apply_command_limits(raw_command, limits_);

    publish_control_command(limited_command);
  }

  void publish_control_command(const ControlCommand & command)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = command.linear_velocity;
    message.angular.z = command.angular_velocity;
    cmd_vel_publisher_->publish(message);
  }

  void publish_stop_command()
  {
    geometry_msgs::msg::Twist stop_command;
    cmd_vel_publisher_->publish(stop_command);
  }

  ControllerGains gains_;
  CommandLimits limits_;
  double waypoint_acceptance_radius_{0.15};
  Path path_;

  Pose2D current_pose_;
  bool has_odometry_{false};
  std::size_t target_index_{0U};
  bool path_complete_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace mobile_robot_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<mobile_robot_control::PathTrackingControllerNode>();

  rclcpp::spin(node);
  node.reset();

  rclcpp::shutdown();
  return 0;
}
