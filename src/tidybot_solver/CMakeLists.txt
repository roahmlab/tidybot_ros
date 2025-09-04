cmake_minimum_required(VERSION 3.22)
project(tidybot_solver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(tidybot_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(demo_plan src/demo_plan.cpp)
add_executable(teleop_to_moveit src/teleop_to_moveit.cpp)
add_executable(joystick_to_moveit src/joystick_to_moveit.cpp)
add_executable(demo_twist src/demo_twist.cpp)
add_executable(moveit_servo_twist_demo src/moveit_servo_twist_demo.cpp)
add_executable(twist_commander src/twist_commander.cpp)

ament_target_dependencies(demo_plan
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools")

ament_target_dependencies(teleop_to_moveit
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "tidybot_utils")

  ament_target_dependencies(joystick_to_moveit
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "tidybot_utils"
  "moveit_servo")

  ament_target_dependencies(demo_twist
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "tidybot_utils"
  "moveit_servo")

  ament_target_dependencies(moveit_servo_twist_demo
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "tidybot_utils"
  "moveit_servo"
  "geometry_msgs"
  "trajectory_msgs")

  ament_target_dependencies(twist_commander
  "rclcpp"
  "geometry_msgs")

install(TARGETS demo_plan teleop_to_moveit joystick_to_moveit demo_twist moveit_servo_twist_demo twist_commander
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
