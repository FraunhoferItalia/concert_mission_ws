cmake_minimum_required(VERSION 3.5)
project(std_skills)

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
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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
  std_srvs
  geometry_msgs
  behavior_tree_msgs
  tf2_geometry_msgs
)

add_library(${LIBRARY_NAME} SHARED 
  src/skill_library.cpp 
  src/trigger.cpp
  src/set_bool.cpp
  src/packers/pose.cpp
  src/packers/pose_stamped.cpp
)

ament_target_dependencies(${LIBRARY_NAME} ${DEPENDENCIES})

pluginlib_export_plugin_description_file(bt_executor plugins.xml)

add_executable(service_server test/service_server.cpp)
ament_target_dependencies(service_server ${DEPENDENCIES})

install(
  TARGETS ${LIBRARY_NAME} service_server
  EXPORT  ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS service_server
  RUNTIME DESTINATION lib/${LIBRARY_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION 
  share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

ament_export_libraries(${library_name})
ament_package()
