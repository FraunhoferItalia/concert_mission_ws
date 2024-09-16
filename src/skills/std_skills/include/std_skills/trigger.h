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

#ifndef TRIGGER_SKILL_LIBRARY
#define TRIGGER_SKILL_LIBRARY
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace std_skills
{
using TriggerServiceType = std_srvs::srv::Trigger;

class Trigger : public BT::RosServiceNode<TriggerServiceType>
{
  using RequestPtr = TriggerServiceType::Request::SharedPtr;
  using ResponsePtr = TriggerServiceType::Response::SharedPtr;

public:
  Trigger(
    std::string const & name, BT::NodeConfig const & config, const BT::RosNodeParams & params);

protected:
  bool setRequest(RequestPtr & srv) override;
  BT::NodeStatus onResponseReceived(const ResponsePtr & response) override;

  std::shared_ptr<rclcpp::Node> nh_;
  std::string service_name_;
};

}  // namespace std_skills

#endif  // TRIGGER_SKILL_LIBRARY
