
/*
 * Copyright Ⓒ Fraunhofer Italia Research 2024
 *
 * Author: Michael Terzer (michael.terzer@fraunhofer.it)
 *
 */
#include "nav2_skills/skill_library.h"

#include <behavior_tree_msgs/parsing_utils.h>
#include <behaviortree_cpp/bt_factory.h>

#include "behaviortree_ros2/ros_node_params.hpp"
#include "bt_executor/skill_library_base.h"
#include "nav2_skills/drive_on_heading.h"
#include "nav2_skills/move_laterally.h"
#include "nav2_skills/navigate_to_pose.h"
#include "nav2_skills/serialized/navigate_to_pose.h"
#include "nav2_skills/spin.h"

namespace nav2_skills
{

void SkillLibrary::load(BT::BehaviorTreeFactory & factory, BT::RosNodeParams & params) const
{
  bt_skill::SkillLibraryBase::registerNodeType<nav2_skills::NavigateToPose>(
    factory, params, "nav2_skills::NavigateToPose");
  bt_skill::SkillLibraryBase::registerNodeType<nav2_skills::serialized::NavigateToPose>(
    factory, params, "nav2_skills::serialized::NavigateToPose");
  bt_skill::SkillLibraryBase::registerNodeType<nav2_skills::DriveOnHeading>(
    factory, params, "nav2_skills::DriveOnHeading");
  bt_skill::SkillLibraryBase::registerNodeType<nav2_skills::MoveLaterally>(
    factory, params, "nav2_skills::MoveLaterally");
  bt_skill::SkillLibraryBase::registerNodeType<nav2_skills::Spin>(
    factory, params, "nav2_skills::Spin");
  return;
}

}  // namespace nav2_skills
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_skills::SkillLibrary, bt_skill::SkillLibraryBase)
