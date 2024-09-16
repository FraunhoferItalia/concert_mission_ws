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

#ifndef PACKERS_POSE_STAMPED_SKILL_LIBRARY
#define PACKERS_POSE_STAMPED_SKILL_LIBRARY
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace std_skills
{

class PackPoseStamped : public BT::SyncActionNode
{
public:
  PackPoseStamped(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts()
  {
    geometry_msgs::msg::Pose pose;
    return {
      BT::InputPort<std::string>("frame_id"),
      BT::InputPort<geometry_msgs::msg::Pose>("pose", pose, "pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_stamped")};
  }
  BT::NodeStatus tick() override;
};

}  // namespace std_skills

#endif  // PACKERS_POSE_STAMPED_SKILL_LIBRARY
