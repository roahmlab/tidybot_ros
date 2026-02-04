#include <memory>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Geometry>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>

// TF2 for frame transforms
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

class PoseToJointsNode : public rclcpp::Node {
public:
    PoseToJointsNode() : Node("pose_to_joints_node") {
        // Parameters
        this->declare_parameter<std::string>("planning_frame", "arm_base_link");
        this->get_parameter("planning_frame", planning_frame_);
        this->declare_parameter<std::string>("planning_group", "gen3_7dof");
        this->get_parameter("planning_group", planning_group_name_);

        // TF setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        last_time_ = this->get_clock()->now();
        last_delta_time_ = this->now();

        // Subscribers
        arm_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/tidybot/arm/target_pose", 1,
            std::bind(&PoseToJointsNode::target_pose_callback, this, std::placeholders::_1));

        delta_ee_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tidybot/arm/delta_commands", 1,
            std::bind(&PoseToJointsNode::delta_cmd_callback, this, std::placeholders::_1));
            
        gripper_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/tidybot/gripper/commands", 1,
            std::bind(&PoseToJointsNode::gripper_cmd_callback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 1,
            std::bind(&PoseToJointsNode::joint_state_callback, this, std::placeholders::_1));

        // Publishers
        arm_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gen3_7dof_controller/joint_trajectory", 10);
            
        gripper_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/robotiq_2f_85_controller/commands", 10);
            
        arm_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/tidybot/hardware/arm/commands", 10);
            
        gripper_command_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/tidybot/hardware/gripper/commands", 10);

        pose_visual_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/visual_pose", 10);


    }
    
    void initialize_moveit() {
        // Load the robot model
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name_);
        
        if (!joint_model_group_) {
            RCLCPP_ERROR(this->get_logger(), "Joint model group '%s' not found!", planning_group_name_.c_str());
            return;
        }

        model_frame_ = robot_model_->getModelFrame();
        RCLCPP_INFO(this->get_logger(), "MoveIt Initialized. Model Frame: %s, Planning Frame: %s", 
                    model_frame_.c_str(), planning_frame_.c_str());
    }

private:
    void target_pose_callback(const geometry_msgs::msg::Pose &msg) {
        if (handle_time_jump()) return;

        // Transform incoming pose (in planning_frame) to model_frame
        geometry_msgs::msg::PoseStamped pose_in;
        pose_in.header = ros2_header(planning_frame_);
        pose_in.pose = msg;
        
        geometry_msgs::msg::PoseStamped pose_model;
        
        if (!transform_pose(pose_in, pose_model, model_frame_)) {
            return;
        }

        publish_pose_visual(pose_model.pose);
        solve_and_publish_ik(pose_model.pose);
    }

    void delta_cmd_callback(const std_msgs::msg::Float64MultiArray &msg) {
        if (handle_time_jump()) return;
        
        if (msg.data.size() < 7) {
            RCLCPP_WARN(this->get_logger(), "Received invalid delta command size.");
            return;
        }

        rclcpp::Time now = this->now();
        double time_since_last_cmd = (now - last_delta_time_).seconds();
        last_delta_time_ = now;

        // If no command received in >0.5s, assume a new episode/chunk 
        // is starting and snap virtual tracker to the real robot.
        if (!valid_virtual_pose_ || time_since_last_cmd > 0.5) {
            update_robot_state_from_feedback();
            virtual_ee_pose_ = robot_state_->getGlobalLinkTransform(joint_model_group_->getLinkModelNames().back());
            valid_virtual_pose_ = true;
            RCLCPP_INFO(this->get_logger(), "Resetting Virtual Tracker to Physical Robot Pose");
        }

        // Get current EE pose in model frame
        const Eigen::Isometry3d& current_tf = robot_state_->getGlobalLinkTransform(joint_model_group_->getLinkModelNames().back());
        
        // Convert Eigen to Pose msg manually to avoid toMsg issues without tf2_eigen header
        geometry_msgs::msg::Pose current_pose;
        Eigen::Quaterniond q_eigen(current_tf.rotation());
        current_pose.position.x = current_tf.translation().x();
        current_pose.position.y = current_tf.translation().y();
        current_pose.position.z = current_tf.translation().z();
        current_pose.orientation.w = q_eigen.w();
        current_pose.orientation.x = q_eigen.x();
        current_pose.orientation.y = q_eigen.y();
        current_pose.orientation.z = q_eigen.z();

        geometry_msgs::msg::PoseStamped current_pose_stamped;
        current_pose_stamped.header = ros2_header(model_frame_);
        current_pose_stamped.pose = current_pose;

        geometry_msgs::msg::PoseStamped current_pose_planning;
        
        if (!transform_pose(current_pose_stamped, current_pose_planning, planning_frame_)) {
            return; // Failed to transform to planning frame
        }
        
        // Apply Delta in Planning Frame
        geometry_msgs::msg::Pose target_pose = current_pose_planning.pose;
        target_pose.position.x += msg.data[0];
        target_pose.position.y += msg.data[1];
        target_pose.position.z += msg.data[2];

        // Apply Rotation Delta
        Eigen::Quaterniond q_curr(target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
        Eigen::AngleAxisd d_roll(msg.data[3], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd d_pitch(msg.data[4], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd d_yaw(msg.data[5], Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q_new = q_curr * d_yaw * d_pitch * d_roll;
        
        target_pose.orientation.w = q_new.w();
        target_pose.orientation.x = q_new.x();
        target_pose.orientation.y = q_new.y();
        target_pose.orientation.z = q_new.z();

        // Transform back to model frame for IK
        geometry_msgs::msg::PoseStamped target_pose_planning;
        target_pose_planning.header = ros2_header(planning_frame_);
        target_pose_planning.pose = target_pose;
        
        geometry_msgs::msg::PoseStamped target_pose_model;
        
        if (!transform_pose(target_pose_planning, target_pose_model, model_frame_)) {
            return;
        }        

        publish_pose_visual(target_pose_model.pose);
        
        // Solve IK
        if (solve_and_publish_ik(target_pose_model.pose)) {
            gripper_state_ = std::clamp(msg.data[6], 0.0, 1.0);
            publish_gripper_hardware(gripper_state_);
            publish_gripper_sim(gripper_state_); // Added sim publisher call
        }
    }

    void gripper_cmd_callback(const std_msgs::msg::Float64 &msg) {
        gripper_state_ = std::clamp(msg.data, 0.0, 1.0);
        publish_gripper_hardware(gripper_state_);
        publish_gripper_sim(gripper_state_);
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_joint_state_ = *msg;
    }


    
    // Core Logic Helpers

    void update_robot_state_from_feedback() {
        if (last_joint_state_.name.empty()) return;

        std::map<std::string, double> joint_map;
        for (size_t i = 0; i < last_joint_state_.name.size(); ++i) {
            joint_map[last_joint_state_.name[i]] = last_joint_state_.position[i];
        }
        robot_state_->setVariablePositions(joint_map);
        robot_state_->update(); // Update transforms
    }

    bool solve_and_publish_ik(const geometry_msgs::msg::Pose &target_pose_model_frame) {
        update_robot_state_from_feedback();

        // IK Options
        kinematics::KinematicsQueryOptions options;
        options.return_approximate_solution = true; 

        bool found_ik = robot_state_->setFromIK(
            joint_model_group_, 
            target_pose_model_frame, 
            0.1,    // Timeout
            moveit::core::GroupStateValidityCallbackFn(), 
            options
        );

        if (found_ik) {
            robot_state_->enforceBounds();
            std::vector<double> joint_values;
            robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);
            
            publish_trajectory(joint_values);
            publish_hardware_command(joint_values);
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "IK solution not found");
            return false;
        }
    }

    void publish_trajectory(const std::vector<double>& joint_positions) {
        if (!arm_traj_pub_) return;
        
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_model_group_->getVariableNames();
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_positions;
        point.time_from_start = rclcpp::Duration::from_seconds(0.05); // Fast execution target
        
        traj.points.push_back(point);
        arm_traj_pub_->publish(traj);
    }

    void publish_hardware_command(const std::vector<double>& joint_positions) {
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = joint_model_group_->getVariableNames();
        js.position = joint_positions;
        arm_command_pub_->publish(js);
    }

    void publish_gripper_hardware(double state) {
        std_msgs::msg::Float64 msg;
        msg.data = state;
        gripper_command_pub_->publish(msg);
    }

    void publish_gripper_sim(double state) {
        if (!gripper_pos_pub_) return;
        std_msgs::msg::Float64MultiArray msg;
        // Send command to both finger_joint and right_outer_knuckle_joint
        msg.data = {state * 0.8, state * 0.8}; // Scale constraint for Robotiq
        gripper_pos_pub_->publish(msg);
    }

    // Utilities

    bool transform_pose(const geometry_msgs::msg::PoseStamped& in, geometry_msgs::msg::PoseStamped& out, const std::string& target_frame) {
        if (in.header.frame_id == target_frame) {
            out = in;
            return true;
        }
        try {
            out = tf_buffer_->transform(in, target_frame, tf2::durationFromSec(0.1));
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "TF Error %s -> %s: %s", in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
            return false;
        }
    }

    bool handle_time_jump() {
        auto now = this->get_clock()->now();
        if (last_time_.nanoseconds() > now.nanoseconds()) {
            RCLCPP_WARN(this->get_logger(), "Time jump detected. Resetting TF.");
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            last_time_ = now;
            return true;
        }
        last_time_ = now;
        return false;
    }

    std_msgs::msg::Header ros2_header(const std::string& frame_id) {
        std_msgs::msg::Header h;
        h.stamp = this->now();
        h.frame_id = frame_id;
        return h;
    }

    void publish_pose_visual(const geometry_msgs::msg::Pose &pose) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = model_frame_;
        msg.pose = pose;
        pose_visual_pub_->publish(msg);
    }

    // Members
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;

    std::string planning_frame_;
    std::string planning_group_name_;
    std::string model_frame_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr delta_ee_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pos_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_command_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_visual_pub_;

    sensor_msgs::msg::JointState last_joint_state_;
    // This virtual command pose accumumlates deltas within an action chunk
    // to avoid drift and resets after not receiving a delta command for 0.5s
    Eigen::Isometry3d virtual_ee_pose_;
    rclcpp::Time last_delta_time_;
    bool valid_virtual_pose_ = false;

    double gripper_state_ = 0.0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseToJointsNode>();
    node->initialize_moveit();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
