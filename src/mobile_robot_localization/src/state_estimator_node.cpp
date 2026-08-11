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

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <stdexcept>

#include "mobile_robot_localization/planar_estimator.hpp"
#include "mobile_robot_localization/measurement_freshness.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace mobile_robot_localization
{

class StateEstimatorNode : public rclcpp::Node
{
public:
  StateEstimatorNode()
  : Node("state_estimator")
  {
    imu_topic_ = declare_parameter<std::string>(
      "imu_topic", "imu/data");
    wheel_odom_topic_ = declare_parameter<std::string>(
      "wheel_odom_topic", "wheel/odom");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "odom");
    odom_frame_ = declare_parameter<std::string>(
      "odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>(
      "base_frame", "base_footprint");

    max_time_step_ = declare_parameter<double>(
      "max_time_step", 1.0);
    imu_timeout_ = declare_parameter<double>(
      "imu_timeout", 0.2);
    pose_x_variance_ = declare_parameter<double>(
      "pose_x_variance", 0.05);
    pose_y_variance_ = declare_parameter<double>(
      "pose_y_variance", 0.05);
    pose_yaw_variance_ = declare_parameter<double>(
      "pose_yaw_variance", 0.02);
    linear_velocity_variance_ = declare_parameter<double>(
      "linear_velocity_variance", 0.04);
    yaw_rate_variance_ = declare_parameter<double>(
      "yaw_rate_variance", 0.01);
    const bool output_variances_are_valid =
      std::isfinite(pose_x_variance_) &&
      pose_x_variance_ > 0.0 &&
      std::isfinite(pose_y_variance_) &&
      pose_y_variance_ > 0.0 &&
      std::isfinite(pose_yaw_variance_) &&
      pose_yaw_variance_ > 0.0 &&
      std::isfinite(linear_velocity_variance_) &&
      linear_velocity_variance_ > 0.0 &&
      std::isfinite(yaw_rate_variance_) &&
      yaw_rate_variance_ > 0.0;

    if (!output_variances_are_valid) {
      throw std::invalid_argument(
              "output variances must be finite and positive");
    }

    if (
      !std::isfinite(max_time_step_) ||
      max_time_step_ <= 0.0)
    {
      throw std::invalid_argument(
              "max_time_step must be finite and positive");
    }

    if (
      !std::isfinite(imu_timeout_) ||
      imu_timeout_ <= 0.0)
    {
      throw std::invalid_argument(
              "imu_timeout must be finite and positive");
    }

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    imu_subscription_ =
      create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &StateEstimatorNode::imu_callback,
        this,
        std::placeholders::_1));

    wheel_odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &StateEstimatorNode::wheel_odom_callback,
        this,
        std::placeholders::_1));

    odom_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "State estimator node started");
  }

private:
  void broadcast_estimated_transform(
    const rclcpp::Time & stamp)
  {
    const auto & state = estimator_.state();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;

    transform.transform.translation.x = state.x;
    transform.transform.translation.y = state.y;
    transform.transform.rotation.z =
      std::sin(0.5 * state.yaw);
    transform.transform.rotation.w =
      std::cos(0.5 * state.yaw);

    tf_broadcaster_->sendTransform(transform);
  }
  void publish_estimated_odometry(
    const rclcpp::Time & stamp)
  {
    const auto & state = estimator_.state();

    nav_msgs::msg::Odometry odom_message;
    odom_message.header.stamp = stamp;
    odom_message.header.frame_id = odom_frame_;
    odom_message.child_frame_id = base_frame_;

    odom_message.pose.pose.position.x = state.x;
    odom_message.pose.pose.position.y = state.y;
    odom_message.pose.pose.orientation.z =
      std::sin(0.5 * state.yaw);
    odom_message.pose.pose.orientation.w =
      std::cos(0.5 * state.yaw);

    odom_message.twist.twist.linear.x = state.v;
    odom_message.twist.twist.angular.z = state.yaw_rate;

    odom_message.pose.covariance[0] =
      pose_x_variance_;
    odom_message.pose.covariance[7] =
      pose_y_variance_;
    odom_message.pose.covariance[35] =
      pose_yaw_variance_;
    odom_message.twist.covariance[0] =
      linear_velocity_variance_;
    odom_message.twist.covariance[35] =
      yaw_rate_variance_;

    odom_publisher_->publish(odom_message);
  }

  void wheel_odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const rclcpp::Time current_stamp(message->header.stamp);

    if (!estimator_.is_initialized()) {
      const double x = message->pose.pose.position.x;
      const double y = message->pose.pose.position.y;
      const double yaw =
        tf2::getYaw(message->pose.pose.orientation);

      estimator_.initialize(x, y, yaw);

      if (!estimator_.is_initialized()) {
        return;
      }

      last_wheel_odom_stamp_ = current_stamp;

      const auto & state = estimator_.state();

      RCLCPP_INFO(
        get_logger(),
        "Estimator initialized: x=%.3f m, y=%.3f m, "
        "yaw=%.3f rad",
        state.x,
        state.y,
        state.yaw);

      return;
    }

    const double dt =
      (current_stamp - last_wheel_odom_stamp_).seconds();

    if (dt <= 0.0 || dt > max_time_step_) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected wheel odometry timestamp: dt=%.6f s",
        dt);
      return;
    }

    last_wheel_odom_stamp_ = current_stamp;

    if (!has_imu_measurement_) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "Waiting for the first IMU measurement");
      return;
    }

    const double current_time_seconds =
      current_stamp.seconds();
    const double imu_age =
      current_time_seconds - latest_imu_time_seconds_;

    if (
      !is_measurement_fresh(
        latest_imu_time_seconds_,
        current_time_seconds,
        imu_timeout_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Skipping fused update: IMU measurement is not fresh "
        "(age=%.6f s, timeout=%.6f s)",
        imu_age,
        imu_timeout_);
      return;
    }

    const double wheel_linear_velocity =
      message->twist.twist.linear.x;
    const double wheel_yaw_rate =
      message->twist.twist.angular.z;
    const double wheel_yaw_rate_variance =
      message->twist.covariance[35];

    estimator_.update(
      wheel_linear_velocity,
      wheel_yaw_rate,
      wheel_yaw_rate_variance,
      latest_imu_yaw_rate_,
      latest_imu_yaw_rate_variance_,
      dt);

    publish_estimated_odometry(current_stamp);
    broadcast_estimated_transform(current_stamp);
    const auto & state = estimator_.state();

    RCLCPP_INFO_ONCE(
      get_logger(),
      "First fused update: dt=%.6f s, x=%.3f m, "
      "y=%.3f m, yaw=%.3f rad, v=%.3f m/s, "
      "yaw_rate=%.3f rad/s",
      dt,
      state.x,
      state.y,
      state.yaw,
      state.v,
      state.yaw_rate);
  }
  void imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const rclcpp::Time imu_stamp(
      message->header.stamp);
    latest_imu_time_seconds_ =
      imu_stamp.seconds();
    latest_imu_yaw_rate_ =
      message->angular_velocity.z;
    latest_imu_yaw_rate_variance_ =
      message->angular_velocity_covariance[8];
    has_imu_measurement_ = true;

    RCLCPP_INFO_ONCE(
      get_logger(),
      "First IMU measurement: stamp=%.6f s, "
      "yaw_rate=%.6f rad/s, variance=%.6g",
      latest_imu_time_seconds_,
      latest_imu_yaw_rate_,
      latest_imu_yaw_rate_variance_);
  }
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
    imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    wheel_odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
    odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>
  tf_broadcaster_;
  std::string imu_topic_;
  std::string wheel_odom_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  PlanarEstimator estimator_;
  rclcpp::Time last_wheel_odom_stamp_{
    0,
    0,
    RCL_ROS_TIME};

  double latest_imu_yaw_rate_{0.0};
  double latest_imu_yaw_rate_variance_{0.0};
  double latest_imu_time_seconds_{0.0};
  bool has_imu_measurement_{false};
  double max_time_step_{1.0};
  double imu_timeout_{0.2};
  double pose_x_variance_{0.05};
  double pose_y_variance_{0.05};
  double pose_yaw_variance_{0.02};
  double linear_velocity_variance_{0.04};
  double yaw_rate_variance_{0.01};
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
