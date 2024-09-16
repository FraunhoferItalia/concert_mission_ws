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

#include "std_skills/packers/pose_stamped.hpp"

#include "std_skills/packers/pose.hpp"

namespace std_skills
{

PackPoseStamped::PackPoseStamped(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus PackPoseStamped::tick()
{
  std::string frame_id, child_frame_id;
  geometry_msgs::msg::Pose pose;
  getInput<std::string>("frame_id", frame_id);
  getInput<std::string>("child_frame_id", child_frame_id);
  getInput<geometry_msgs::msg::Pose>("pose", pose);
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.pose = pose;
  setOutput("pose", pose);
  return BT::NodeStatus::SUCCESS;
}
};  // namespace std_skills