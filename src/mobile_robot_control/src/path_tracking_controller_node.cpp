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
#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "mobile_robot_control/path_types.hpp"
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

    stop_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(
        &PathTrackingControllerNode::publish_stop_command,
        this));

    RCLCPP_INFO(
      get_logger(),
      "Path tracking controller started in safe-stop mode");
  }

  ~PathTrackingControllerNode() override
  {
    publish_stop_command();
  }

private:
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

  void publish_stop_command()
  {
    geometry_msgs::msg::Twist stop_command;
    cmd_vel_publisher_->publish(stop_command);
  }

  Pose2D current_pose_;
  bool has_odometry_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;

  rclcpp::TimerBase::SharedPtr stop_timer_;
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
