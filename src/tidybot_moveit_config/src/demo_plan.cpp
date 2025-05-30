#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "gen3_lite");

  auto const target_pos = []{
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = -0.1;
    target_pose.position.y = 0.28;
    target_pose.position.z = 1.0;
    target_pose.orientation.w = 0; // No rotation
    return target_pose;
  }();
  move_group_interface.setPoseTarget(target_pos);
  
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const ok = static_cast<bool>(move_group_interface.plan(plan));
    return std::make_pair(ok, plan);
  }();

  if (success) {
    RCLCPP_INFO(logger, "Planning succeeded");
    RCLCPP_INFO(logger, "Planning time: %.2f seconds", plan.planning_time);
    RCLCPP_INFO(logger, "Number of points in trajectory: %zu", plan.trajectory.joint_trajectory.points.size());
    
    // Execute the plan
    auto const execute_result = move_group_interface.execute(plan);
    if (execute_result) {
      RCLCPP_INFO(logger, "Execution succeeded");
    } else {
      RCLCPP_ERROR(logger, "Execution failed");
    }
  } else {
    RCLCPP_ERROR(logger, "Planning failed");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}