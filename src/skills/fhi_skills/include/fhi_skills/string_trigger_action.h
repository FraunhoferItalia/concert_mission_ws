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

#ifndef STRING_TRIGGER_ACTION_SKILL
#define STRING_TRIGGER_ACTION_SKILL
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "fhi_msgs/action/string_trigger.hpp"
#include "rclcpp/clock.hpp"

namespace fhi_skills
{
class StringTriggerAction : public BT::RosActionNode<fhi_msgs::action::StringTrigger>
{
public:
  StringTriggerAction(
    const std::string & instance_name, const BT::NodeConfiguration & conf,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<fhi_msgs::action::StringTrigger>(instance_name, conf, params)
  {
    return;
  }

  bool setGoal(Goal & goal)
  {
    std::string data;
    if (!getInput("data", data)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input data!");
      return false;
    }
    goal.data = data;
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr)
  {
    if (wr.result->success) {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(), wr.result->message);
      return BT::NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s: onFailure %d", name().c_str(), error);
    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("data", "trajectory data in json format"),
    });
  }
};
}  // namespace fhi_skills
#endif  // EXECUTE_CROK_TRAJECTORY_SKILL
