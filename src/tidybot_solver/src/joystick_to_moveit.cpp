// Adapted from https://github.com/moveit/moveit2/blob/jazzy/moveit_ros/moveit_servo/demos/cpp_interface/demo_twist.cpp
#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/utils/logger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

using namespace moveit_servo;

class JoystickToMoveit : public rclcpp::Node {
public:
    JoystickToMoveit() : Node("joystick_to_moveit") {
        moveit::setNodeLoggerName("joystick_to_moveit");
        auto switch_input_cli = this->create_client<moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type");
        // Wait for the service to be available
        while (!switch_input_cli->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for switch_command_type service to be available...");
        }
        auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
        request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
        auto future = switch_input_cli->async_send_request(request);

        if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Successfully switched to TWIST command type.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch command type to TWIST.");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickToMoveit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
