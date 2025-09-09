cmake_minimum_required(VERSION 3.22)
project(tidybot_solver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tidybot_utils REQUIRED)

add_executable(demo_plan src/demo_plan.cpp)
add_executable(teleop_to_moveit src/teleop_to_moveit.cpp)

ament_target_dependencies(demo_plan
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools")

ament_target_dependencies(teleop_to_moveit
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "tidybot_utils")

install(TARGETS demo_plan teleop_to_moveit
  DESTINATION lib/${PROJECT_NAME})

ament_package()
