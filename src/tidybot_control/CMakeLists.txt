cmake_minimum_required(VERSION 3.22)
project(tidybot_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tidybot_msgs REQUIRED)

add_executable(web_server_subscriber tidybot_control/web_server_subscriber.cpp)

target_include_directories(web_server_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(web_server_subscriber
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "tidybot_msgs")

install(TARGETS web_server_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
