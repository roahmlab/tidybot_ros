#include <memory>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Geometry>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.hpp>

// TF2 for frame transforms between planning_frame and model_frame
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using moveit::planning_interface::MoveGroupInterface;
#include "sensor_msgs/msg/joint_state.hpp"

class TeleopToMoveit : public rclcpp::Node {
    public:
    TeleopToMoveit() : Node("teleop_to_moveit") {
        // Parameters
        this->declare_parameter<std::string>("planning_frame", "arm_base_link");
        this->get_parameter("planning_frame", planning_frame);

        // TF setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Initialize last time for time-jump detection
        last_time_ = this->get_clock()->now();
        sensitivity = 0.01; // Incoming end effector position must not be within 1cm of previous pose
        options.discretization_method = kinematics::DiscretizationMethod::ALL_DISCRETIZED;
        options.lock_redundant_joints = false;
        options.return_approximate_solution = true;

        // Listen to global tidybot commands (end effector pose)
        arm_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/tidybot/arm/target_pose", 1,
            std::bind(&TeleopToMoveit::publish_arm, this, std::placeholders::_1));
        delta_ee_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tidybot/arm/delta_commands", 1,
            std::bind(&TeleopToMoveit::publish_delta_ee, this, std::placeholders::_1));
        gripper_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/tidybot/gripper/commands", 1,
            std::bind(&TeleopToMoveit::publish_gripper, this, std::placeholders::_1));

        // Publish to ros2_control controllers
        arm_traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gen3_7dof_controller/joint_trajectory", 10);
        gripper_pos_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/robotiq_2f_85_controller/commands", 10);
        // Publish to real hardware
        arm_command_pub = this->create_publisher<sensor_msgs::msg::JointState>(
            "/tidybot/hardware/arm/commands", 10);
        gripper_command_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/tidybot/hardware/gripper/commands", 10);

        // Debugging
        pose_visual_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/visual_pose", 10);
        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 1,
        std::bind(&TeleopToMoveit::joint_state_callback, this, std::placeholders::_1));

        // Need to periodically publish gripper state to force ros2_control to update in the simulation mode
        gripper_timer = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() {
                std_msgs::msg::Float64MultiArray gripper_msg;
                gripper_msg.data = {gripper_state * 0.8}; // Scale to match ros2_control gripper limits
                gripper_pos_pub->publish(gripper_msg);
            });
    }
    
    void initialize_moveit() {
        robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
        robot_model = robot_model_loader.getModel();
        robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
        arm_joint_model_group = robot_model->getJointModelGroup("gen3_7dof");
        // gripper_joint_model_group = robot_model->getJointModelGroup("gen3_lite_2f");

        // Determine the model/world frame used by MoveIt/RobotModel
        model_frame = robot_model->getModelFrame();
        // If user hasn't overridden planning_frame, keep what was declared; otherwise ensure it's set
        this->get_parameter("planning_frame", planning_frame);
    }

    void publish_arm(const geometry_msgs::msg::Pose &msg) {
        // Handle time jump: reset TF buffer/listener if clock moved backwards
        if (handle_time_jump_and_reset_tf()) {
            return; // skip this callback invocation to allow TF to repopulate
        }
        // Incoming pose is interpreted in the configured planning_frame
        geometry_msgs::msg::PoseStamped pose_in;
        pose_in.header.stamp = this->now();
        pose_in.header.frame_id = planning_frame;
        pose_in.pose = msg;

        geometry_msgs::msg::PoseStamped pose_in_model;
        try {
            if (planning_frame == model_frame) {
                pose_in_model = pose_in;
                pose_in_model.header.frame_id = model_frame;
            } else {
                pose_in_model = tf_buffer_->transform(pose_in, model_frame, tf2::durationFromSec(0.2));
            }
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF transform failed from %s to %s: %s", planning_frame.c_str(), model_frame.c_str(), ex.what());
            return;
        }

        this->publish_pose_visual(pose_in_model.pose);
        // if (pose_distance(last_pose, msg) < sensitivity) {
        //     return;
        // }

        // Get current robot state
        std::map<std::string, double> joint_position_map;
        for (size_t i = 0; i < last_joint_state.name.size(); ++i) {
            joint_position_map[last_joint_state.name[i]] = last_joint_state.position[i];
        }
        robot_state->setVariablePositions(joint_position_map);
        robot_state->update();

        // Perform IK
        bool found_ik = robot_state->setFromIK(
            arm_joint_model_group, pose_in_model.pose, 0.1, 
            moveit::core::GroupStateValidityCallbackFn(), options
        );

        if (!found_ik) {
            RCLCPP_WARN(rclcpp::get_logger("arm_callback"), "IK solution not found");
            return;
        }

        robot_state->enforceBounds();
        last_pose = msg; // Store last pose in planning_frame coordinates
        pose_initialized = true;

        // Extract joint values
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(arm_joint_model_group, joint_values);
        const std::vector<std::string>& joint_names = arm_joint_model_group->getVariableNames();
        
        // RCLCPP_INFO(rclcpp::get_logger("arm_callback"),
        //     "IK input pose: pos=(%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f), norm=%.6f",
        //     msg.position.x, msg.position.y, msg.position.z,
        //     msg.orientation.x, msg.orientation.y,
        //     msg.orientation.z, msg.orientation.w,
        //     std::sqrt(msg.orientation.x*msg.orientation.x +
        //               msg.orientation.y*msg.orientation.y +
        //               msg.orientation.z*msg.orientation.z +
        //               msg.orientation.w*msg.orientation.w));

        // Now check bounds for each joint
        for (size_t i = 0; i < joint_names.size(); ++i) {
            const auto& jm = robot_state->getJointModel(joint_names[i]);
            const auto& bounds = jm->getVariableBounds(joint_names[i]);

            // RCLCPP_INFO(rclcpp::get_logger("arm_callback"),
            //             "Joint %s bounds: [%f, %f], current %f",
            //             joint_names[i].c_str(),
            //             bounds.min_position_, bounds.max_position_,
            //             joint_values[i]);
        }

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

        // Publish trajectory for sim
        arm_traj_pub->publish(traj);
        // RCLCPP_INFO(rclcpp::get_logger("arm_callback"), "Trajectory sent");

        // Publish joint states for real hardware
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->now(); 
        joint_state_msg.name = joint_names;
        joint_state_msg.position = joint_values;
        arm_command_pub->publish(joint_state_msg);
    }

    void publish_delta_ee(const std_msgs::msg::Float64MultiArray &msg) {
        // Handle time jump: reset TF buffer/listener if clock moved backwards
        if (handle_time_jump_and_reset_tf()) {
            return; // skip this callback invocation to allow TF to repopulate
        }
        if (!pose_initialized) {
            // Fill RobotState from the latest joint_state message
            std::map<std::string, double> joint_position_map;
            for (size_t i = 0; i < last_joint_state.name.size(); ++i) {
                joint_position_map[last_joint_state.name[i]] = last_joint_state.position[i];
            }
            robot_state->setVariablePositions(joint_position_map);
            robot_state->update();

            // Get current pose of end-effector
            const Eigen::Isometry3d& current_eef_pose =
                robot_state->getGlobalLinkTransform(arm_joint_model_group->getLinkModelNames().back());

            geometry_msgs::msg::Pose current_pose_msg;
            current_pose_msg.position.x = current_eef_pose.translation().x();
            current_pose_msg.position.y = current_eef_pose.translation().y();
            current_pose_msg.position.z = current_eef_pose.translation().z();

            Eigen::Quaterniond q(current_eef_pose.rotation());
            current_pose_msg.orientation.x = q.x();
            current_pose_msg.orientation.y = q.y();
            current_pose_msg.orientation.z = q.z();
            current_pose_msg.orientation.w = q.w();

            // Transform FK pose from model_frame -> planning_frame for internal tracking
            geometry_msgs::msg::PoseStamped fk_in_model;
            fk_in_model.header.stamp = this->now();
            fk_in_model.header.frame_id = model_frame;
            fk_in_model.pose = current_pose_msg;

            try {
                if (planning_frame == model_frame) {
                    last_pose = current_pose_msg;
                } else {
                    auto fk_in_planning = tf_buffer_->transform(fk_in_model, planning_frame, tf2::durationFromSec(0.2));
                    last_pose = fk_in_planning.pose;
                }
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF transform failed (init FK) from %s to %s: %s", model_frame.c_str(), planning_frame.c_str(), ex.what());
                last_pose = current_pose_msg; // fallback
            }
            pose_initialized = true;

            RCLCPP_INFO(rclcpp::get_logger("arm_callback"), "Initialized last_pose from FK");
        }

        geometry_msgs::msg::Pose target_pose = last_pose;

        target_pose.position.x += msg.data[0];
        target_pose.position.y += msg.data[1];
        target_pose.position.z += msg.data[2];

        Eigen::Quaterniond current_q(
            target_pose.orientation.w,
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z
        );

        double roll_delta  = msg.data[3];
        double pitch_delta = msg.data[4];
        double yaw_delta   = msg.data[5];

        Eigen::AngleAxisd d_roll(roll_delta,   Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd d_pitch(pitch_delta, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd d_yaw(yaw_delta,     Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond delta_q = d_yaw * d_pitch * d_roll;  
        Eigen::Quaterniond new_q = current_q * delta_q;
        new_q.normalize();

        target_pose.orientation.w = new_q.w();
        target_pose.orientation.x = new_q.x();
        target_pose.orientation.y = new_q.y();
        target_pose.orientation.z = new_q.z();
        
        // Visualize target in model frame
        geometry_msgs::msg::PoseStamped target_in;
        target_in.header.stamp = this->now();
        target_in.header.frame_id = planning_frame;
        target_in.pose = target_pose;

        geometry_msgs::msg::PoseStamped target_in_model;
        try {
            if (planning_frame == model_frame) {
                target_in_model = target_in;
                target_in_model.header.frame_id = model_frame;
            } else {
                target_in_model = tf_buffer_->transform(target_in, model_frame, tf2::durationFromSec(0.2));
            }
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF transform failed from %s to %s: %s", planning_frame.c_str(), model_frame.c_str(), ex.what());
            return;
        }
        this->publish_pose_visual(target_in_model.pose);

        // Get current robot state
        std::map<std::string, double> joint_position_map;
        for (size_t i = 0; i < last_joint_state.name.size(); ++i) {
            joint_position_map[last_joint_state.name[i]] = last_joint_state.position[i];
        }
        robot_state->setVariablePositions(joint_position_map);
        robot_state->update();

        // Perform IK
        bool found_ik = robot_state->setFromIK(
            arm_joint_model_group, target_in_model.pose, 0.1, 
            moveit::core::GroupStateValidityCallbackFn(), options
        );

        if (!found_ik) {
            RCLCPP_WARN(rclcpp::get_logger("arm_callback"), "IK solution not found");
            return;
        }

        robot_state->enforceBounds();
        last_pose = target_pose; // Keep in planning_frame

        // Extract joint values
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(arm_joint_model_group, joint_values);
        const std::vector<std::string>& joint_names = arm_joint_model_group->getVariableNames();
        
        // Now check bounds for each joint
        for (size_t i = 0; i < joint_names.size(); ++i) {
            const auto& jm = robot_state->getJointModel(joint_names[i]);
            const auto& bounds = jm->getVariableBounds(joint_names[i]);

            RCLCPP_INFO(rclcpp::get_logger("arm_callback"),
                        "Joint %s bounds: [%f, %f], current %f",
                        joint_names[i].c_str(),
                        bounds.min_position_, bounds.max_position_,
                        joint_values[i]);
        }

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

        // Publish trajectory for sim
        arm_traj_pub->publish(traj);
        RCLCPP_INFO(rclcpp::get_logger("arm_callback"), "Trajectory sent");
        gripper_state = msg.data[6];

        // Publish joint states for real hardware
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->now(); 
        joint_state_msg.name = joint_names;
        joint_state_msg.position = joint_values;
        arm_command_pub->publish(joint_state_msg);

        std_msgs::msg::Float64 gripper_msg;
        gripper_msg.data = gripper_state;
        gripper_command_pub->publish(gripper_msg);
    }

    void publish_gripper(const std_msgs::msg::Float64 gripper_delta) {
        // Update gripper state
        gripper_state = std::clamp(gripper_delta.data, 0.0, 1.0);
        std_msgs::msg::Float64 gripper_msg;
        gripper_msg.data = gripper_state;
        gripper_command_pub->publish(gripper_msg);
    }

    void publish_pose_visual(const geometry_msgs::msg::Pose &pose)
    {
        auto msg = geometry_msgs::msg::PoseStamped();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = model_frame;

        msg.pose.position.x = pose.position.x;
        msg.pose.position.y = pose.position.y;
        msg.pose.position.z = pose.position.z;

        msg.pose.orientation.x = pose.orientation.x;
        msg.pose.orientation.y = pose.orientation.y;
        msg.pose.orientation.z = pose.orientation.z;
        msg.pose.orientation.w = pose.orientation.w;

        pose_visual_pub->publish(msg);
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
    // Returns true if a time jump backward was detected and TF was reset
    bool handle_time_jump_and_reset_tf() {
        auto now = this->get_clock()->now();
        if (last_time_.nanoseconds() > now.nanoseconds()) {
            RCLCPP_WARN(this->get_logger(), "Time jump detected, resetting TF buffer and listener...");
            tf_listener_.reset();
            tf_buffer_.reset();
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            last_time_ = now;
            return true;
        }
        last_time_ = now;
        return false;
    }

    moveit::core::RobotModelPtr robot_model;
    moveit::core::RobotStatePtr robot_state;
    rclcpp::Logger logger = rclcpp::get_logger("web_server_moveit");

    // TF & frame configuration
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string planning_frame;
    std::string model_frame;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr delta_ee_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_command_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_command_pub;
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pos_pub;
    const moveit::core::JointModelGroup* arm_joint_model_group;
    // const moveit::core::JointModelGroup* gripper_joint_model_group;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_visual_pub;

    geometry_msgs::msg::Pose last_pose;
    bool pose_initialized = false;
    sensor_msgs::msg::JointState last_joint_state;

    // Track last callback time to detect backward time jumps and refresh TF
    rclcpp::Time last_time_;

    float sensitivity;
    kinematics::KinematicsQueryOptions options;
    double gripper_state = 0.0;
    rclcpp::TimerBase::SharedPtr gripper_timer;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopToMoveit>();
    node->initialize_moveit();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
