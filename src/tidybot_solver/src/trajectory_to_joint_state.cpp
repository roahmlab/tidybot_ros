#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class TrajectoryToJointState : public rclcpp::Node {
public:
    TrajectoryToJointState() : Node("trajectory_to_joint_state") {
        RCLCPP_INFO(this->get_logger(), "Starting trajectory_to_joint_state node");
        
        // Subscribe to joint trajectory messages
        trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/gen3_7dof_controller/joint_trajectory", 10,
            std::bind(&TrajectoryToJointState::trajectory_callback, this, std::placeholders::_1));
        
        // Publisher for joint state commands
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/tidybot/physical_arm/commands", 10);
        
        RCLCPP_INFO(this->get_logger(), "Node initialized - listening to /gen3_7dof_controller/joint_trajectory");
        RCLCPP_INFO(this->get_logger(), "Publishing to /tidybot/physical_arm/commands");
    }

private:
    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        // Check if trajectory has any points
        if (msg->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory, ignoring");
            return;
        }
        
        // Get the last trajectory point
        const auto& last_point = msg->points.back();
        
        // Create JointState message
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.name = msg->joint_names;
        joint_state_msg.position = last_point.positions;
        
        // Include velocities and efforts if available
        if (!last_point.velocities.empty()) {
            joint_state_msg.velocity = last_point.velocities;
        }
        if (!last_point.accelerations.empty()) {
            joint_state_msg.effort = last_point.accelerations;  // Note: using accelerations as effort placeholder
        }
        
        // Publish the joint state
        joint_state_publisher_->publish(joint_state_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Published joint state with %zu joints from trajectory with %zu points", 
            joint_state_msg.name.size(), msg->points.size());
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryToJointState>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
