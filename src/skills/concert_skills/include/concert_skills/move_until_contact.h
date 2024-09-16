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

#ifndef MOVE_UNTIL_CONTACT_SKILL
#define MOVE_UNTIL_CONTACT_SKILL
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "concert_msgs/action/move_until_contact.hpp"
#include "rclcpp/clock.hpp"

namespace BT
{
template <>
inline geometry_msgs::msg::Vector3 convertFromString(StringView str)
{
  //printf("Converting string: \"%s\"\n", str.data());

  // real numbers separated by semicolons
  auto parts = splitString(str, ';');
  if (parts.size() != 3) {
    throw RuntimeError("invalid input)");
  } else {
    geometry_msgs::msg::Vector3 pose;
    pose.x = convertFromString<double>(parts[0]);
    pose.y = convertFromString<double>(parts[1]);
    pose.z = convertFromString<double>(parts[2]);
    return pose;
  }
}
}  // namespace BT

namespace concert_skills
{
class MoveUntilContact : public BT::RosActionNode<concert_msgs::action::MoveUntilContact>
{
public:
  MoveUntilContact(
    const std::string & instance_name, const BT::NodeConfiguration & conf,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<concert_msgs::action::MoveUntilContact>(instance_name, conf, params)
  {
    return;
  }

  bool setGoal(Goal & goal)
  {
    geometry_msgs::msg::Vector3 motion_direction;
    if (!getInput("motion_direction", motion_direction)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input motion_direction!");
      return false;
    }

    float force;
    if (!getInput("force", force)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input force!");
      return false;
    }

    float speed;
    if (!getInput("speed", speed)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input speed!");
      return false;
    }

    float max_distance;
    if (!getInput("max_distance", max_distance)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input max_distance!");
      return false;
    }

    float timeout_sec;
    if (!getInput("timeout_sec", timeout_sec)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input timeout_sec!");
      return false;
    }

    RCLCPP_INFO_STREAM(
      node_->get_logger(), "motion-direction x: " << motion_direction.x
                                                  << " y: " << motion_direction.y
                                                  << " z: " << motion_direction.z);
    RCLCPP_INFO_STREAM(node_->get_logger(), "force" << force);
    RCLCPP_INFO_STREAM(node_->get_logger(), "speed" << speed);
    RCLCPP_INFO_STREAM(node_->get_logger(), "max_distance" << max_distance);
    RCLCPP_INFO_STREAM(node_->get_logger(), "timeout_sec" << timeout_sec);
    goal.motion_direction = motion_direction;
    goal.force = force;
    goal.speed = speed;
    goal.max_distance = max_distance;
    goal.timeout_sec = timeout_sec;
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr)
  {
    return BT::NodeStatus::SUCCESS;

    if (wr.result->result_code == 0) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "TIMED_OUT!");
      return BT::NodeStatus::FAILURE;
    } else if (wr.result->result_code == 1) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "MAX_DISTANCE_REACHED!");
      return BT::NodeStatus::SUCCESS;
    } else if (wr.result->result_code == 2) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "FORCE_REACHED!");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s: onFailure %d", name().c_str(), error);
    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::Vector3>(
        "motion_direction", geometry_msgs::msg::Vector3(), "Motion direction until contact"),
      BT::InputPort<float>("force", "Force threshhold"),
      BT::InputPort<float>("speed", "Desired speed"),
      BT::InputPort<float>("max_distance", "Maximum distance to move"),
      BT::InputPort<float>("timeout_sec", "Timeout until failure"),
    });
  }
};
}  // namespace concert_skills
#endif  // MOVE_UNTIL_CONTACT_SKILL
