#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "tidybot_msgs/msg/ws_msg.hpp"

using moveit::planning_interface::MoveGroupInterface;

class WebServerSubscriber : public rclcpp::Node {
    public:
    WebServerSubscriber() : Node("web_server_subscriber") {
        subscriber_ = this->create_subscription<tidybot_msgs::msg::WSMsg>(
            "/ws_commands", 10,
            std::bind(&WebServerSubscriber::pose_callback, this, std::placeholders::_1));
    }

    void pose_callback(const tidybot_msgs::msg::WSMsg &WSMsg) {
        auto const target_pos = [WSMsg]{
            geometry_msgs::msg::Pose target_pose;
            if (WSMsg.teleop_mode == "arm") {
                target_pose.position.x = WSMsg.pos_x;
                target_pose.position.y = WSMsg.pos_y;
                target_pose.position.z = WSMsg.pos_z;
                target_pose.orientation.x = WSMsg.or_x;
                target_pose.orientation.y = WSMsg.or_y;
                target_pose.orientation.z = WSMsg.or_z;
                target_pose.orientation.w = WSMsg.or_w;
            }
            return target_pose;
        }();

        auto arm_move_group_interface = moveit::planning_interface::MoveGroupInterface(this->shared_from_this(), "gen3_lite");
        arm_move_group_interface.setPoseTarget(target_pos);

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


    private:
    rclcpp::Subscription<tidybot_msgs::msg::WSMsg>::SharedPtr subscriber_;
    rclcpp::Logger logger = rclcpp::get_logger("web_server_subscriber");
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebServerSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}