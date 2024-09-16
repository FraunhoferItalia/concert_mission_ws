/*
 * Copyright Ⓒ Fraunhofer Italia Research 2024
 *
 * Author: Michael Terzer (michael.terzer@fraunhofer.it)
 *
 */
#ifndef NAV2_SKILLS_SKILL_LIBRARY
#define NAV2_SKILLS_SKILL_LIBRARY
#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>

#include "bt_executor/skill_library_base.h"

namespace nav2_skills
{

class SkillLibrary : public bt_skill::SkillLibraryBase
{
public:
  /**\fn    SkillLibrary
       * \brief Constructor for a skill library
      */
  SkillLibrary() = default;
  SkillLibrary(SkillLibrary const &) = default;
  SkillLibrary & operator=(SkillLibrary const &) = default;
  SkillLibrary(SkillLibrary &&) = default;
  SkillLibrary & operator=(SkillLibrary &&) = default;

  void load(BT::BehaviorTreeFactory & factory, BT::RosNodeParams & params) const override;
};

}  // namespace nav2_skills

#endif  // NAV2_SKILLS_SKILL_LIBRARY