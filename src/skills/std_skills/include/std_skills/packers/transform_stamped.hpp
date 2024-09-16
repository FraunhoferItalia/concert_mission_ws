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

#ifndef PACKERS_TRANSFORM_STAMPED_SKILL_LIBRARY
#define PACKERS_TRANSFORM_STAMPED_SKILL_LIBRARY
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include "geometry_msgs/msg/transform_stamped.hpp"

//namespace BT
//{
// template <>
// geometry_msgs::msg::Transform convertFromString(StringView key)
// {
//   // three real numbers separated by semicolons
//   auto parts = BT::splitString(key, ';');
//   if (parts.size() != 7) {
//     throw RuntimeError("invalid input)");
//   } else {
//     geometry_msgs::msg::Transform output;
//     output.translation.x = convertFromString<double>(parts[0]);
//     output.translation.y = convertFromString<double>(parts[1]);
//     output.translation.z = convertFromString<double>(parts[2]);
//     output.rotation.x = convertFromString<double>(parts[3]);
//     output.rotation.y = convertFromString<double>(parts[4]);
//     output.rotation.z = convertFromString<double>(parts[5]);
//     output.rotation.w = convertFromString<double>(parts[6]);

//     return output;
//   }
// }
// }  // namespace BT

namespace std_skills
{

class PackTransformStamped : public BT::SyncActionNode
{
public:
  PackTransformStamped(std::string const & name, BT::NodeConfig const & config)
  : SyncActionNode(name, config)
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("parent_frame_id"), BT::InputPort<std::string>("child_frame_id"),
      BT::InputPort<geometry_msgs::msg::Transform>(
        "transform", geometry_msgs::msg::Transform(), "default transform"),
      BT::BidirectionalPort<geometry_msgs::msg::TransformStamped>("output")};
  }

private:
  virtual BT::NodeStatus tick() override
  {
    std::string parent_frame_id;
    if (!getInput("parent_frame_id", parent_frame_id)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("set_transform_stamped"), "Could not parse input parent_frame_id.");
      return BT::NodeStatus::FAILURE;
    }

    std::string child_frame_id;
    if (!getInput("child_frame_id", child_frame_id)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("set_transform_stamped"), "Could not parse input child_frame_id.");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = parent_frame_id;
    transform_stamped.child_frame_id = child_frame_id;

    geometry_msgs::msg::Transform transform;
    if (!getInput<geometry_msgs::msg::Transform>("transform", transform)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("set_transform_stamped"),
        "Could not parse input pose. Using default one...");
    }
    transform_stamped.transform = transform;

    setOutput("output", transform_stamped);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace std_skills

#endif  //ACTION_SET_TRANSFORM_MSG_NODE_H