cmake_minimum_required(VERSION 3.22)
project(tidybot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tidybot_msgs REQUIRED)

add_executable(demo_plan src/demo_plan.cpp)
add_executable(web_server_subscriber src/web_server_subscriber.cpp)

target_include_directories(demo_plan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(demo_plan
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools")

ament_target_dependencies(web_server_subscriber
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "tidybot_msgs")

install(TARGETS demo_plan web_server_subscriber
  DESTINATION lib/${PROJECT_NAME})


if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
