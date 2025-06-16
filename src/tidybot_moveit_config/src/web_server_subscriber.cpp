#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "std_msgs/msg/float32.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.hpp>

using moveit::planning_interface::MoveGroupInterface;

class WebServerSubscriber : public rclcpp::Node {
    public:
    WebServerSubscriber() : Node("web_server_subscriber") {
        base_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/base_controller/command", 1,
            std::bind(&WebServerSubscriber::base_callback, this, std::placeholders::_1));
        arm_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/arm_controller/command", 1,
            std::bind(&WebServerSubscriber::arm_callback, this, std::placeholders::_1));
        gripper_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/gripper_controller/command", 1,
            std::bind(&WebServerSubscriber::gripper_callback, this, std::placeholders::_1));

        arm_traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gen3_lite_controller/joint_trajectory", 10);
    }
    
    void initialize_moveit() {
        robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
        robot_model = robot_model_loader.getModel();
        robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
        arm_joint_model_group = robot_model->getJointModelGroup("gen3_lite");
    }
    
    void base_callback(const geometry_msgs::msg::Pose &msg) {
        return;
    }

    void arm_callback(const geometry_msgs::msg::Pose &msg)
    {
        // Get current robot state
        robot_state->setToDefaultValues();
        robot_state->update();

        // Perform IK
        bool found_ik = robot_state->setFromIK(arm_joint_model_group, msg);

        if (!found_ik) {
            RCLCPP_WARN(rclcpp::get_logger("arm_callback"), "IK solution not found");
            return;
        }

        // Extract joint values
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(arm_joint_model_group, joint_values);
        const std::vector<std::string>& joint_names = arm_joint_model_group->getVariableNames();

        // Create trajectory message
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names;

        // Start point
        trajectory_msgs::msg::JointTrajectoryPoint start_point;
        std::vector<double> current_positions;
        robot_state->copyJointGroupPositions(arm_joint_model_group, current_positions);
        start_point.positions = current_positions;
        start_point.time_from_start = rclcpp::Duration(0, 0);
        traj.points.push_back(start_point);

        // End point
        trajectory_msgs::msg::JointTrajectoryPoint end_point;
        end_point.positions = joint_values;
        end_point.time_from_start = rclcpp::Duration::from_seconds(0.1);
        traj.points.push_back(end_point);

        // Publish trajectory
        arm_traj_pub->publish(traj);
        RCLCPP_INFO(rclcpp::get_logger("arm_callback"), "Trajectory sent");
    }

    void gripper_callback(const std_msgs::msg::Float32 &msg) {
        return;
    }

    private:
    moveit::core::RobotModelPtr robot_model;
    moveit::core::RobotStatePtr robot_state;
    rclcpp::Logger logger = rclcpp::get_logger("web_server_subscriber");

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_subscriber_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub;
    const moveit::core::JointModelGroup* arm_joint_model_group;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebServerSubscriber>();
    node->initialize_moveit();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
