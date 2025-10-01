cmake_minimum_required(VERSION 3.22)
project(tidybot_solver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tidybot_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(demo_plan src/demo_plan.cpp)
add_executable(teleop_to_moveit src/teleop_to_moveit.cpp)
add_executable(twist_to_moveit src/twist_to_moveit.cpp)
add_executable(keyboard_input src/keyboard_input.cpp)

ament_target_dependencies(demo_plan
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools")

ament_target_dependencies(teleop_to_moveit
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "tidybot_utils")

  ament_target_dependencies(twist_to_moveit
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "tidybot_utils"
  "moveit_servo"
  "moveit_msgs"
  "geometry_msgs"
  "trajectory_msgs"
  "moveit_ros_planning"
  "tf2_geometry_msgs"
  "tf2_ros")

ament_target_dependencies(keyboard_input
  "rclcpp"
  "geometry_msgs"
  "control_msgs"
  "moveit_msgs")

install(TARGETS demo_plan teleop_to_moveit twist_to_moveit keyboard_input
DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
