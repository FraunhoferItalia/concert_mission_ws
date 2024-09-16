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

#ifndef STD_SKILLS_WAIT_FOR_ACKNOWLEDGE
#define STD_SKILLS_WAIT_FOR_ACKNOWLEDGE
#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace std_skills
{

class WaitForAcknowledge : public BT::ConditionNode
{
public:
  WaitForAcknowledge(
    std::string const & name, BT::NodeConfig const & config, const BT::RosNodeParams & params)
  : BT::ConditionNode(name, config), node_(params.nh)
  {
  }

  void create_service_server(std::string const & name)
  {
    // callback_group_executor_.add_node(node_);
    service_server_ = node_->create_service<std_srvs::srv::Trigger>(
      name, std::bind(
              &WaitForAcknowledge::service_callback_, this, std::placeholders::_1,
              std::placeholders::_2));
  }

  void service_callback_(
    const std_srvs::srv::Trigger::Request::SharedPtr /* request */,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!enabled_.load()) {
      response->success = false;
      response->message = "Not waiting";
    }
    enabled_.store(false);
    response->success = true;
  }

  BT::NodeStatus tick() override
  {
    using namespace std::chrono_literals;
    std::string service_name;
    if (!getInput("service_name", service_name) || service_name.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "service_name port empty ");
      return BT::NodeStatus::FAILURE;
    }
    double timeout;
    if (!getInput("timeout", timeout)) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not parse input timeout.");
      return BT::NodeStatus::FAILURE;
    }
    std::string topic_name;
    if (!getInput("topic_name", topic_name)) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not parse input topic_name.");
      return BT::NodeStatus::FAILURE;
    }
    std::string data;
    if (!getInput("data", data)) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not parse input data.");
      return BT::NodeStatus::FAILURE;
    }

    if (!topic_name.empty() && !data.empty()) {
      publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_name, 10);
    }

    if (!service_server_) {
      create_service_server(service_name);
    }
    enabled_.store(true);
    auto start_time = std::chrono::high_resolution_clock::now();
    while (enabled_.load()) {
      std::this_thread::sleep_for(100ms);
      if (publisher_ && !data.empty()) {
        auto message = std_msgs::msg::String();
        message.data = data;
        publisher_->publish(message);
      }
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "waiting ... ");
      if (
        std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::high_resolution_clock::now() - start_time)
          .count() >= timeout) {
        RCLCPP_INFO(node_->get_logger(), "Timed out. ");
        return BT::NodeStatus::FAILURE;
      }
      // callback_group_executor_.spin_some();
    }
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("service_name"),
      BT::InputPort<std::string>(
        "topic_name", "", "Name of the topic in which the data string has to be published"),
      BT::InputPort<std::string>("data", "", "String to be published on topic name"),
      BT::InputPort<double>(
        "timeout", std::numeric_limits<double>::max(),
        "Number of seconds to wait for before returning failure")};
  }

protected:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string service_name_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_server_ = nullptr;
  std::atomic<bool> enabled_;
};
}  // namespace std_skills

#endif  // STD_SKILLS_WAIT_FOR_ACKNOWLEDGE
