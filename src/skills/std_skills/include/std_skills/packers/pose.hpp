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

#ifndef PACKERS_POSE_SKILL_LIBRARY
#define PACKERS_POSE_SKILL_LIBRARY
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include "geometry_msgs/msg/pose.hpp"

namespace std_skills
{

class PackPose : public BT::SyncActionNode
{
public:
  PackPose(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x", 0.0, "x"),   BT::InputPort<double>("y", 0.0, "y"),
      BT::InputPort<double>("z", 0.0, "z"),   BT::InputPort<double>("Rx", 0.0, "Rx"),
      BT::InputPort<double>("Ry", 0.0, "Ry"), BT::InputPort<double>("Rz", 0.0, "Rz"),
      BT::InputPort<double>("Rw", 1.0, "Rw"), BT::OutputPort<geometry_msgs::msg::Pose>("pose")};
  }
  BT::NodeStatus tick() override;
};

}  // namespace std_skills

#endif  // PACKERS_POSE_SKILL_LIBRARY
