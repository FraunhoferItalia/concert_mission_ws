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

#ifndef UNPACKERS_TRANSFORM_STAMPED_SKILL_LIBRARY
#define UNPACKERS_TRANSFORM_STAMPED_SKILL_LIBRARY
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace std_skills
{

class UnpackTransformStamped : public BT::SyncActionNode
{
public:
  UnpackTransformStamped(std::string const & name, BT::NodeConfig const & config)
  : SyncActionNode(name, config)
  {
    return;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::TransformStamped>("transform_stamped"),
      BT::OutputPort<std::string>("frame_id"),
      BT::OutputPort<std::string>("child_frame_id"),
      BT::OutputPort<geometry_msgs::msg::Transform>("transform"),
      BT::OutputPort<double>("x"),
      BT::OutputPort<double>("y"),
      BT::OutputPort<double>("z"),
      BT::OutputPort<double>("roll"),
      BT::OutputPort<double>("pitch"),
      BT::OutputPort<double>("yaw")};
  }

private:
  virtual BT::NodeStatus tick() override
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    if (!getInput("transform_stamped", transform_stamped)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("unpack_transform_stamped"),
        "Could not parse input `transform_stamped`.");
      return BT::NodeStatus::FAILURE;
    }

    setOutput("frame_id", transform_stamped.header.frame_id);
    setOutput("child_frame_id", transform_stamped.child_frame_id);
    setOutput("transform", transform_stamped.transform);
    setOutput("x", transform_stamped.transform.translation.x);
    setOutput("y", transform_stamped.transform.translation.y);
    setOutput("z", transform_stamped.transform.translation.z);
    tf2::Quaternion q(
      transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    setOutput("roll", roll);
    setOutput("pitch", pitch);
    setOutput("yaw", yaw);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace std_skills

#endif  //UNPACKERS_TRANSFORM_STAMPED_SKILL_LIBRARY