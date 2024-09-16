
/*
 * Copyright Ⓒ Fraunhofer Italia Research 2024
 *
 * Author: Marco Magri (marco.magri@fraunhofer.it)
 *
 */

#ifndef NAVIGATE_TO_POSE_SKILL
#define NAVIGATE_TO_POSE_SKILL
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_skills
{
class NavigateToPose : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToPose(
    const std::string & instance_name, const BT::NodeConfiguration & conf,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<nav2_msgs::action::NavigateToPose>(instance_name, conf, params)
  {
    return;
  }

  bool setGoal(Goal & goal)
  {
    std::string frame_id;
    if (!getInput("frame_id", frame_id)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input frame_id!");
      return false;
    }

    geometry_msgs::msg::Pose pose;
    if (!getInput("pose", pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input pose!");
      return false;
    }

    goal.pose.header.frame_id = frame_id;
    goal.behavior_tree = "";
    goal.pose.header.stamp = node_->get_clock()->now();
    goal.pose.pose = pose;
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
      BT::InputPort<std::string>("frame_id", "map", "Name of the frame to plan to."),
      BT::InputPort<geometry_msgs::msg::Pose>(
        "pose", geometry_msgs::msg::Pose(), "Goal offset from the frame expressed in `frame_id`"),
    });
  }
};
}  // namespace nav2_skills
#endif  // NAVIGATE_TO_POSE_SKILL
