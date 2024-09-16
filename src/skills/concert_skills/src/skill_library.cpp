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

#include "concert_skills/skill_library.h"

#include <behavior_tree_msgs/parsing_utils.h>
#include <behaviortree_cpp/bt_factory.h>

#include "behaviortree_ros2/ros_node_params.hpp"
#include "bt_executor/skill_library_base.h"
#include "concert_skills/move_until_contact.h"

namespace concert_skills
{

void SkillLibrary::load(BT::BehaviorTreeFactory & factory, BT::RosNodeParams & params) const
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("skill_loader"), "Loading concert_skills");
  bt_skill::SkillLibraryBase::registerNodeType<MoveUntilContact>(
    factory, params, "concert_skills::MoveUntilContact");

  return;
}

}  // namespace concert_skills
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(concert_skills::SkillLibrary, bt_skill::SkillLibraryBase)
