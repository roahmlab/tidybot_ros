// Adapted from https://github.com/moveit/moveit2/blob/jazzy/moveit_ros/moveit_servo/demos/cpp_interface/demo_twist.cpp
#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/utils/logger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

using namespace moveit_servo;

class JoystickToMoveit : public rclcpp::Node {
public:
    JoystickToMoveit() : Node("joystick_to_moveit") {
        moveit::setNodeLoggerName("joystick_to_moveit");
        
        // Subscribe to twist commands from joystick controller
        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/tidybot/arm/twist", 10,
            std::bind(&JoystickToMoveit::twistCallback, this, std::placeholders::_1));
        
        // Publisher for joint trajectory commands
        joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gen3_7dof_controller/joint_trajectory", 10);
        
        RCLCPP_INFO(this->get_logger(), "Joystick to MoveIt Servo bridge initialized");
    }
    
    void initialize() {
        // Get the Servo parameters
        const std::string param_namespace = "moveit_servo";
        servo_param_listener_ = std::make_shared<const servo::ParamListener>(shared_from_this(), param_namespace);
        servo_params_ = servo_param_listener_->get_params();
        
        // Create the planning scene monitor
        planning_scene_monitor_ = moveit_servo::createPlanningSceneMonitor(shared_from_this(), servo_params_);
        
        // Create a Servo instance
        servo_ = std::make_unique<Servo>(shared_from_this(), servo_param_listener_, planning_scene_monitor_);
        
        // Set command type to TWIST
        servo_->setCommandType(CommandType::TWIST);

        // Reset smoothing plugin with current robot state to avoid size mismatch on first command
        auto initial_state = servo_->getCurrentRobotState(true /* block_for_state */);
        if (!initial_state.joint_names.empty())
        {
            servo_->resetSmoothing(initial_state);
            last_commanded_state_ = initial_state;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Initial robot state is empty. Smoothing reset skipped.");
        }
        
        // Timer for servo loop
        servo_timer_ = this->create_timer(
            std::chrono::milliseconds(static_cast<int>(servo_params_.publish_period * 1000)),
            std::bind(&JoystickToMoveit::servoLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Servo initialized successfully");
    }

private:
    void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        // Store the latest twist command
        latest_twist_command_ = *msg;
        has_twist_command_ = true;
    }
    
    void servoLoop() {
        if (!servo_ || !has_twist_command_) {
            return;
        }
        
        // Create TwistCommand for Servo
        TwistCommand twist_command{latest_twist_command_.header.frame_id, 
                                 {latest_twist_command_.twist.linear.x,
                                  latest_twist_command_.twist.linear.y,
                                  latest_twist_command_.twist.linear.z,
                                  latest_twist_command_.twist.angular.x,
                                  latest_twist_command_.twist.angular.y,
                                  latest_twist_command_.twist.angular.z}};
        
        // Get current robot state
        auto robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
        
        // Get the next joint state from Servo
        KinematicState next_joint_state = servo_->getNextJointState(robot_state, twist_command);
        last_commanded_state_ = next_joint_state;
        
        // Check servo status
        StatusCode status = servo_->getStatus();
        if (status != StatusCode::NO_WARNING) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Servo status: %d", static_cast<int>(status));
        }
        
        // Convert to JointTrajectory message and publish
        if (!next_joint_state.joint_names.empty()) {
            trajectory_msgs::msg::JointTrajectory joint_trajectory;
            joint_trajectory.header.stamp = this->now();
            joint_trajectory.header.frame_id = "arm_base_link";
            joint_trajectory.joint_names = next_joint_state.joint_names;
            
            trajectory_msgs::msg::JointTrajectoryPoint point;
            // Convert Eigen vectors to std::vector
            point.positions.assign(next_joint_state.positions.data(), 
                                 next_joint_state.positions.data() + next_joint_state.positions.size());
            point.velocities.assign(next_joint_state.velocities.data(), 
                                  next_joint_state.velocities.data() + next_joint_state.velocities.size());
            point.time_from_start = rclcpp::Duration::from_nanoseconds(
                static_cast<int64_t>(servo_params_.publish_period * 1e9));
            
            joint_trajectory.points.push_back(point);
            joint_trajectory_pub_->publish(joint_trajectory);
        }
    }
    
    std::shared_ptr<const servo::ParamListener> servo_param_listener_;
    servo::Params servo_params_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::unique_ptr<Servo> servo_;
    
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::TimerBase::SharedPtr servo_timer_;
    
    geometry_msgs::msg::TwistStamped latest_twist_command_;
    bool has_twist_command_ = false;
    KinematicState last_commanded_state_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickToMoveit>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
