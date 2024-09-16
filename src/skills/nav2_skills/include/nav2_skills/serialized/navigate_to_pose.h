/*
 * Copyright Ⓒ Fraunhofer Italia Research 2023
 *
 * Author: Michael Terzer (michael.terzer@fraunhofer.it)
 *
 */

#ifndef SERIALIZED_NAVIGATE_TO_POSE_SKILL
#define SERIALIZED_NAVIGATE_TO_POSE_SKILL
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

using namespace std::chrono_literals;

struct Pose2D
{
  double x, y, theta;
};

// from here: https://github.com/BehaviorTree/BehaviorTree.CPP/blob/b9906d1be0ffbaaaf37a9d59ee674170aeba58f0/examples/t03_generic_ports.cpp#L20
namespace BT
{
template <>
inline Pose2D convertFromString(StringView str)
{
  //printf("Converting string: \"%s\"\n", str.data());

  // real numbers separated by semicolons
  auto parts = splitString(str, ';');
  if (parts.size() != 3) {
    throw RuntimeError("invalid input)");
  } else {
    Pose2D pose;
    pose.x = convertFromString<double>(parts[0]);
    pose.y = convertFromString<double>(parts[1]);
    pose.theta = convertFromString<double>(parts[2]);
    return pose;
  }
}
}  // namespace BT

namespace nav2_skills
{
namespace serialized
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
    Pose2D pose;

    if (!getInput("pose", pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Was not able to parse input pose!");
      return false;
    }

    goal.pose.header.frame_id = "map";
    goal.behavior_tree = "";
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    goal.pose.header.stamp = clock->now();

    goal.pose.pose.position.x = pose.x;
    goal.pose.pose.position.y = pose.y;
    goal.pose.pose.orientation.y = pose.y;

    tf2::Quaternion quaternion{};
    double const roll = 0.0;
    double const pitch = 0.0;
    double const yaw = pose.theta;
    quaternion.setRPY(roll, pitch, yaw);
    quaternion = quaternion.normalize();
    goal.pose.pose.orientation = tf2::toMsg(quaternion);

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
      BT::InputPort<Pose2D>("pose", "Destination to plan to"),
    });
  }
};
}  // namespace serialized
}  // namespace nav2_skills
#endif  // SERIALIZED_NAVIGATE_TO_POSE_SKILL
