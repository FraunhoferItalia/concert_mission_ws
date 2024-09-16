/*
 * Copyright Ⓒ Fraunhofer Italia Research 2024
 *
 * Author: Michael Terzer (michael.terzer@fraunhofer.it)
 *
 */

#ifndef DRIVE_ON_HEADING_SKILL
#define DRIVE_ON_HEADING_SKILL
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_skills
{
class DriveOnHeading : public BT::RosActionNode<nav2_msgs::action::DriveOnHeading>
{
public:
  DriveOnHeading(
    const std::string & instance_name, const BT::NodeConfiguration & conf,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<nav2_msgs::action::DriveOnHeading>(instance_name, conf, params)
  {
    return;
  }

  bool setGoal(Goal & goal)
  {
    double distance;
    if (!getInput("distance", distance)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input distance!");
      return false;
    }

    double speed;
    if (!getInput("speed", speed)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input speed!");
      return false;
    }

    double time_allowance;
    if (!getInput("time_allowance", time_allowance)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input time_allowance!");
      return false;
    }

    RCLCPP_INFO_STREAM(
      node_->get_logger(), "Moving of " << distance << " on x direction at " << speed);
    goal.target.x = distance;
    goal.speed = (distance > 0) ? std::abs(speed) : -std::abs(speed);
    goal.time_allowance.sec = time_allowance;

    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr)
  {
    //RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived %d", name().c_str(), wr.result->result );
    return BT::NodeStatus::SUCCESS;

    // if this function is called, the action was executed successfully.
    // however it could be up to the application to handle the specific error_code.
    /*if(wr.result->error_code != 0){
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;*/
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s: onFailure %d", name().c_str(), error);
    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("distance", "Distance to travel in x."),
      BT::InputPort<double>("speed", "Speed to comannd."),
      BT::InputPort<double>("time_allowance", "Time that is allowed to move in seconds."),
    });
  }
};
}  // namespace nav2_skills
#endif  // DRIVE_ON_HEADING_SKILL
