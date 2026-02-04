cmake_minimum_required(VERSION 3.22)
project(tidybot_solver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_msgs REQUIRED) # Was in original, not in snippet, but likely needed for moveit_msgs


find_package(control_msgs REQUIRED) # Was in original, not in snippet, but likely needed
find_package(tidybot_utils REQUIRED) # Was in original, not in snippet, but likely needed

find_package(moveit_task_constructor_core REQUIRED)
# find_package(common_interfaces REQUIRED)

add_executable(moveit_ee_pose_ik src/moveit_ee_pose_ik.cpp)
add_executable(moveit_ee_vel_ik src/moveit_ee_vel_ik.cpp)
add_executable(keyboard_input src/keyboard_input.cpp)
add_executable(multi_stage_planner src/multi_stage_planner.cpp)

ament_target_dependencies(moveit_ee_pose_ik
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_core"

  "tidybot_utils"
  "geometry_msgs"
  "std_msgs"
  "sensor_msgs"
  "trajectory_msgs"
  "tf2_ros"
  "tf2_geometry_msgs")

ament_target_dependencies(moveit_ee_vel_ik
  "rclcpp"
  "moveit_ros_planning_interface"

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



ament_target_dependencies(multi_stage_planner
  rclcpp
  rclcpp_action
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  trajectory_msgs
  std_msgs
  tf2_eigen
  tidybot_utils
)

install(TARGETS moveit_ee_pose_ik moveit_ee_vel_ik keyboard_input multi_stage_planner
DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
