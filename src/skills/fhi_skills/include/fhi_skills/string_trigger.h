// Copyright 2022-2024 Fraunhofer Italia Research

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
//
// Copyright 2022 Fraunhofer Italia Research. All Rights Reserved.

#ifndef STRING_TRIGGER_SKILL_LIBRARY
#define STRING_TRIGGER_SKILL_LIBRARY
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "fhi_msgs/srv/string_trigger.hpp"

namespace fhi_skills
{
using ServiceType = fhi_msgs::srv::StringTrigger;

class StringTrigger : public BT::RosServiceNode<ServiceType>
{
  using RequestPtr = ServiceType::Request::SharedPtr;
  using ResponsePtr = ServiceType::Response::SharedPtr;

public:
  StringTrigger(
    std::string const & name, BT::NodeConfig const & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<std::string>("data")});
  }

protected:
  bool setRequest(RequestPtr & srv) override;
  BT::NodeStatus onResponseReceived(const ResponsePtr & response) override;

  std::shared_ptr<rclcpp::Node> nh_;
  std::string service_name_;
};

}  // namespace fhi_skills

#endif  // STRING_TRIGGER_SKILL_LIBRARY
