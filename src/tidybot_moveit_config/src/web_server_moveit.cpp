#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "std_msgs/msg/float64.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.hpp>

using moveit::planning_interface::MoveGroupInterface;
#include "sensor_msgs/msg/joint_state.hpp"

class WebServerMoveit : public rclcpp::Node {
    public:
    WebServerMoveit() : Node("web_server_subscriber") {
        sensitivity = 0.01; // Incoming end effector position must not be within 1cm of previous pose
        options.discretization_method = kinematics::DiscretizationMethod::ALL_DISCRETIZED;
        options.lock_redundant_joints = false;
        options.return_approximate_solution = true;

        arm_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/arm_controller/command", 1,
            std::bind(&WebServerMoveit::publish_arm, this, std::placeholders::_1));
        gripper_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gripper_controller/command", 1,
            std::bind(&WebServerMoveit::publish_gripper, this, std::placeholders::_1));
        arm_traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gen3_lite_controller/joint_trajectory", 10);
        gripper_traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gen3_lite_2f_controller/joint_trajectory", 10);

        // Debugging
        pose_visual_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/visual_pose", 10);
        joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 1,
            std::bind(&WebServerMoveit::joint_state_callback, this, std::placeholders::_1));
    }
    
    void initialize_moveit() {
        robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
        robot_model = robot_model_loader.getModel();
        robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
        arm_joint_model_group = robot_model->getJointModelGroup("gen3_lite");
        gripper_joint_model_group = robot_model->getJointModelGroup("gen3_lite_2f");
    }

    void publish_arm(const geometry_msgs::msg::Pose &msg) {
        this->publish_pose_visual(msg);
        if (pose_distance(last_pose, msg) < sensitivity) {
            return;
        }

        // Get current robot state
        std::map<std::string, double> joint_position_map;
        for (size_t i = 0; i < last_joint_state.name.size(); ++i) {
            joint_position_map[last_joint_state.name[i]] = last_joint_state.position[i];
        }
        robot_state->setVariablePositions(joint_position_map);
        robot_state->update();

        // Perform IK
        bool found_ik = robot_state->setFromIK(
            arm_joint_model_group, msg, 0.1, 
            moveit::core::GroupStateValidityCallbackFn(), options
        );

        if (!found_ik) {
            RCLCPP_WARN(rclcpp::get_logger("arm_callback"), "IK solution not found");
            return;
        }

        last_pose = msg; // Update to most recent valid pose

        // Extract joint values
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(arm_joint_model_group, joint_values);
        const std::vector<std::string>& joint_names = arm_joint_model_group->getVariableNames();
        
        // this->preview_IK(joint_names, joint_values);

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
        end_point.time_from_start = rclcpp::Duration::from_seconds(0.05);
        traj.points.push_back(end_point);

        // Publish trajectory
        arm_traj_pub->publish(traj);
        RCLCPP_INFO(rclcpp::get_logger("arm_callback"), "Trajectory sent");
    }

    void publish_gripper(const std_msgs::msg::Float64 gripper_delta) {
        // Hardcoded mimic joints
        std::vector<std::string> joint_names = {"right_finger_bottom_joint", "right_finger_tip_joint", 
                                                 "left_finger_bottom_joint", "left_finger_tip_joint"};
        std::vector<double> joint_values = {gripper_delta.data * 1.05 - 0.09,
                                            -0.676 * (gripper_delta.data * 0.71 - 0.5) + 0.149, 
                                            -(gripper_delta.data * 1.05 - 0.09),
                                            -0.676 * (gripper_delta.data * 0.71 - 0.5) + 0.149};
        
        // Create trajectory message
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names;

        // Start point
        trajectory_msgs::msg::JointTrajectoryPoint start_point;
        start_point.positions = joint_values;
        start_point.time_from_start = rclcpp::Duration(0, 0);
        traj.points.push_back(start_point);

        // End point
        trajectory_msgs::msg::JointTrajectoryPoint end_point;
        end_point.positions = joint_values;
        end_point.time_from_start = rclcpp::Duration::from_seconds(0.01);
        traj.points.push_back(end_point);

        // Publish trajectory
        gripper_traj_pub->publish(traj);
    }

    void publish_pose_visual(const geometry_msgs::msg::Pose &pose)
    {
        auto msg = geometry_msgs::msg::PoseStamped();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "world";

        msg.pose.position.x = pose.position.x;
        msg.pose.position.y = pose.position.y;
        msg.pose.position.z = pose.position.z;

        msg.pose.orientation.x = pose.orientation.x;
        msg.pose.orientation.y = pose.orientation.y;
        msg.pose.orientation.z = pose.orientation.z;
        msg.pose.orientation.w = pose.orientation.w;

        pose_visual_pub->publish(msg);
    }

    void preview_IK(const std::vector<std::string>& joint_names, std::vector<double> joint_values) {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.name = joint_names;
        joint_state_msg.position = joint_values;

        std::vector<std::string> extra_joint_names = {
            "joint_x", "joint_y", "joint_th", 
            "left_finger_bottom_joint", "left_finger_tip_joint",
            "right_finger_bottom_joint", "right_finger_tip_joint"
        };
        std::vector<double> extra_joint_positions = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };

        joint_state_msg.name.insert(
            joint_state_msg.name.end(),
            extra_joint_names.begin(),
            extra_joint_names.end()
        );

        joint_state_msg.position.insert(
            joint_state_msg.position.end(),
            extra_joint_positions.begin(),
            extra_joint_positions.end()
        );

        joint_state_pub->publish(joint_state_msg);
    }

    float pose_distance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2) {
        return sqrt(pow(pose1.position.x - pose2.position.x, 2) + 
                    pow(pose1.position.y - pose2.position.y, 2) + 
                    pow(pose1.position.z - pose2.position.z, 2));
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_joint_state = *msg;
    }

    private:
    moveit::core::RobotModelPtr robot_model;
    moveit::core::RobotStatePtr robot_state;
    rclcpp::Logger logger = rclcpp::get_logger("web_server_subscriber");

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_traj_pub;
    const moveit::core::JointModelGroup* arm_joint_model_group;
    const moveit::core::JointModelGroup* gripper_joint_model_group;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_visual_pub;

    geometry_msgs::msg::Pose last_pose;
    sensor_msgs::msg::JointState last_joint_state;

    float sensitivity;
    kinematics::KinematicsQueryOptions options;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebServerMoveit>();
    node->initialize_moveit();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
