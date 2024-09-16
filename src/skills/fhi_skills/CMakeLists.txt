cmake_minimum_required(VERSION 3.5)
project(fhi_skills)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bt_executor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(fhi_msgs REQUIRED)

include_directories(
  include
  )
set(LIBRARY_NAME ${PROJECT_NAME})

set(DEPENDENCIES
  rclcpp
  bt_executor
  pluginlib
  behaviortree_cpp
  behaviortree_ros2
  behavior_tree_msgs
  fhi_msgs
)

add_library(${LIBRARY_NAME} SHARED 
  src/skill_library.cpp
  src/string_trigger.cpp
)

ament_target_dependencies(${LIBRARY_NAME} ${DEPENDENCIES})

pluginlib_export_plugin_description_file(bt_executor plugins.xml)

install(
  TARGETS ${LIBRARY_NAME}
  EXPORT  ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  config
  DESTINATION 
  share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

ament_export_libraries(${library_name})
ament_package()

