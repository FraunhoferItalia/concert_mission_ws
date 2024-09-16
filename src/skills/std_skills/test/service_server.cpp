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

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using SetBoolServiceT = std_srvs::srv::SetBool;

int number{0};

void set_bool_callback(
  const std::shared_ptr<SetBoolServiceT::Request> request,
  std::shared_ptr<SetBoolServiceT::Response> response)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("my_logger"), "SetBool service called...");
  RCLCPP_INFO_STREAM(rclcpp::get_logger("my_logger"), "data: " << request->data);

  response->success = true;
  response->message = "hello bt service client! " + std::to_string(number);
  number++;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("test_service_server");
  RCLCPP_INFO_STREAM(rclcpp::get_logger("my_logger"), "Start test service server");

  rclcpp::Service<SetBoolServiceT>::SharedPtr set_bool_service =
    nh->create_service<SetBoolServiceT>("set_bool", &set_bool_callback);

  rclcpp::spin(nh);
  rclcpp::shutdown();
}