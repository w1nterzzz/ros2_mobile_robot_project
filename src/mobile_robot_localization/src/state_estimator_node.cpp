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

#include <functional>
#include <memory>

#include "mobile_robot_localization/planar_estimator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

namespace mobile_robot_localization
{

class StateEstimatorNode : public rclcpp::Node
{
public:
  StateEstimatorNode()
  : Node("state_estimator")
  {
    imu_subscription_ =
      create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data",
      rclcpp::SensorDataQoS(),
      std::bind(
        &StateEstimatorNode::imu_callback,
        this,
        std::placeholders::_1));

    wheel_odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      "/wheel/odom",
      rclcpp::SensorDataQoS(),
      std::bind(
        &StateEstimatorNode::wheel_odom_callback,
        this,
        std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "State estimator node started");
  }

private:
  void wheel_odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (estimator_.is_initialized()) {
      return;
    }

    const double x = message->pose.pose.position.x;
    const double y = message->pose.pose.position.y;
    const double yaw =
      tf2::getYaw(message->pose.pose.orientation);

    estimator_.initialize(x, y, yaw);

    if (!estimator_.is_initialized()) {
      return;
    }

    const auto & state = estimator_.state();

    RCLCPP_INFO(
      get_logger(),
      "Estimator initialized: x=%.3f m, y=%.3f m, "
      "yaw=%.3f rad",
      state.x,
      state.y,
      state.yaw);
  }
  void imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr message)
  {
    latest_imu_yaw_rate_ =
      message->angular_velocity.z;
    latest_imu_yaw_rate_variance_ =
      message->angular_velocity_covariance[8];
    has_imu_measurement_ = true;

    RCLCPP_INFO_ONCE(
      get_logger(),
      "First IMU measurement: yaw_rate=%.6f rad/s, "
      "variance=%.6g",
      latest_imu_yaw_rate_,
      latest_imu_yaw_rate_variance_);
  }
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
    imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    wheel_odom_subscription_;

  PlanarEstimator estimator_;

  double latest_imu_yaw_rate_{0.0};
  double latest_imu_yaw_rate_variance_{0.0};
  bool has_imu_measurement_{false};
};

}  // namespace mobile_robot_localization

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      mobile_robot_localization::StateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
