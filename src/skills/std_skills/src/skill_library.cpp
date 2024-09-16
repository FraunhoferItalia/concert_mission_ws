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

#include "std_skills/skill_library.h"

#include <behavior_tree_msgs/parsing_utils.h>
#include <behaviortree_cpp/bt_factory.h>

#include "behaviortree_ros2/ros_node_params.hpp"
#include "bt_executor/skill_library_base.h"
#include "std_skills/packers/pose.hpp"
#include "std_skills/packers/pose_stamped.hpp"
#include "std_skills/packers/transform_stamped.hpp"
#include "std_skills/set_bool.h"
#include "std_skills/trigger.h"
#include "std_skills/unpackers/transform_stamped.hpp"
#include "std_skills/wait_for_acknowledge.h"

namespace std_skills
{

void SkillLibrary::load(BT::BehaviorTreeFactory & factory, BT::RosNodeParams & params) const
{
  BT::RosNodeParams inf_timeout_params = params;
  inf_timeout_params.server_timeout = std::chrono::milliseconds(100000);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("skill_loader"), "Loading std_skills");
  bt_skill::SkillLibraryBase::registerNodeType<SetBool>(factory, params, "std_skills::SetBool");
  bt_skill::SkillLibraryBase::registerNodeType<Trigger>(
    factory, inf_timeout_params, "std_skills::Trigger");
  bt_skill::SkillLibraryBase::registerNodeType<WaitForAcknowledge>(
    factory, params, "std_skills::WaitForAcknowledge");
  factory.registerNodeType<PackPose>("std_skills::packers::Pose");
  factory.registerNodeType<PackPoseStamped>("std_skills::packers::PoseStamped");
  factory.registerNodeType<PackTransformStamped>("std_skills::packers::TransformStamped");
  factory.registerNodeType<UnpackTransformStamped>("std_skills::unpackers::TransformStamped");
  return;
}

}  // namespace std_skills
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(std_skills::SkillLibrary, bt_skill::SkillLibraryBase)
