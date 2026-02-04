cmake_minimum_required(VERSION 3.22)
project(tidybot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

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

# generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TeleopMsg.msg"
  "msg/MotionStage.msg"
  "srv/ResetEnv.srv"
  "srv/OpenDrawerTask.srv"
  "action/OpenDrawer.action"
  "action/ExecuteStages.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)
ament_package()
