#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "std_msgs/msg/float32.hpp"

using moveit::planning_interface::MoveGroupInterface;

class WebServerSubscriber : public rclcpp::Node {
    public:
    WebServerSubscriber() 
        : Node("web_server_subscriber"),
          arm_move_group_interface(this->shared_from_this(), "gen3_lite") {
        base_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/ws_base_command", 1,
            std::bind(&WebServerSubscriber::pose_callback, this, std::placeholders::_1));
        arm_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/ws_arm_command", 1,
            std::bind(&WebServerSubscriber::pose_callback, this, std::placeholders::_1));
        gripper_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ws_gripper_command", 1,
            std::bind(&WebServerSubscriber::pose_callback, this, std::placeholders::_1));
    }

    void base_callback(const geometry_msgs::msg::Pose &msg) {
        return;
    }

    void arm_callback(const geometry_msgs::msg::Pose &msg) {
        arm_move_group_interface.setPoseTarget(target_pos);
        move_group.set_max_velocity_scaling_factor(1.0)  
        move_group.set_max_acceleration_scaling_factor(1.0)

        auto const [success, plan] = [&arm_move_group_interface]() {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto const ok = static_cast<bool>(arm_move_group_interface.plan(plan));
            return std::make_pair(ok, plan);
        }();

        if (success) {
            RCLCPP_INFO(logger, "Planning succeeded");
            RCLCPP_INFO(logger, "Planning time: %.2f seconds", plan.planning_time);
            RCLCPP_INFO(logger, "Number of points in trajectory: %zu", plan.trajectory.joint_trajectory.points.size());

            // Execute the plan
            auto const execute_result = arm_move_group_interface.execute(plan);
            if (execute_result) RCLCPP_INFO(logger, "Execution succeeded"); 
            else RCLCPP_ERROR(logger, "Execution failed"); 
        }
        else RCLCPP_ERROR(logger, "Planning failed");
    }

    void gripper_callback(const std_msgs::msg::Float32 &msg) {
        return;
    }

    private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_subscriber_;
    rclcpp::Logger logger = rclcpp::get_logger("web_server_subscriber");
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebServerSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}