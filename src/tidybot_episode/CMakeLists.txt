cmake_minimum_required(VERSION 3.8)
project(tidybot_episode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(actions_recorder src/actions_recorder.cpp)
ament_target_dependencies(actions_recorder
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  rosidl_runtime_cpp
  cv_bridge
  image_transport
  OpenCV
  tf2_ros
)

add_executable(obs_recorder src/obs_recorder.cpp)
ament_target_dependencies(obs_recorder
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  rosidl_runtime_cpp
  cv_bridge
  image_transport
  OpenCV
  tf2_ros
)

install(TARGETS actions_recorder
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS obs_recorder
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
