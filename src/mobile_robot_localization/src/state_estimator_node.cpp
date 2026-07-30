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

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace mobile_robot_localization
{

class StateEstimatorNode : public rclcpp::Node
{
public:
  StateEstimatorNode()
  : Node("state_estimator")
  {
    RCLCPP_INFO(
      get_logger(),
      "State estimator node started");
  }
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
