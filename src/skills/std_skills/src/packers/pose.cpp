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
#include "std_skills/packers/pose.hpp"

namespace std_skills
{

PackPose::PackPose(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus PackPose::tick()
{
  double x, y, z, Rx, Ry, Rz, Rw;
  getInput<double>("x", x);
  getInput<double>("y", y);
  getInput<double>("z", z);
  getInput<double>("Rx", Rx);
  getInput<double>("Ry", Ry);
  getInput<double>("Rz", Rz);
  getInput<double>("Rw", Rw);
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = Rx;
  pose.orientation.y = Ry;
  pose.orientation.z = Rz;
  pose.orientation.w = Rw;
  setOutput("pose", pose);
  return BT::NodeStatus::SUCCESS;
}
};  // namespace std_skills