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

#include "fhi_skills/string_trigger.h"

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace fhi_skills
{

StringTrigger::StringTrigger(
  std::string const & name, BT::NodeConfig const & config, const BT::RosNodeParams & params)
: BT::RosServiceNode<ServiceType>{name, config, params}, service_name_{name}
{
  return;
}

bool StringTrigger::setRequest(RequestPtr & req)
{
  std::string data;
  if (!getInput<std::string>("data", data)) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not parse input data.");
    return false;
  }
  req->data = data;
  return true;
}

BT::NodeStatus StringTrigger::onResponseReceived(const ResponsePtr & response)
{
  if (!response->success) {
    return BT::NodeStatus::FAILURE;
    RCLCPP_ERROR_STREAM(
      node_->get_logger(), "Service call was not successfull! Message: " << response->message);
  }
  RCLCPP_INFO_STREAM(
    node_->get_logger(), "Service call was successfull! Message: " << response->message);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace fhi_skills
