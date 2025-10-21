// Adapted from https://github.com/moveit/moveit2/blob/jazzy/moveit_ros/moveit_servo/demos/cpp_interface/demo_twist.cpp
#include <algorithm>
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
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

using namespace moveit_servo;

const std::string GREEN = "\x1b[32m";
const std::string RED = "\x1b[31m";
const std::string RESET = "\x1b[0m";
const std::string BOLD = "\x1b[1m";

class JoystickToMoveit : public rclcpp::Node {
public:
    JoystickToMoveit() : Node("joystick_to_moveit") {
        moveit::setNodeLoggerName("joystick_to_moveit");
        
        command_output_type_param_ = this->declare_parameter<std::string>(
            "command_output_type", "joint_trajectory");
        update_period_sec_ = this->declare_parameter<double>("update_period", 0.0);
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base");
        end_effector_link_ = this->declare_parameter<std::string>("end_effector_link", "bracelet_link");

        output_type_ = parseOutputType(command_output_type_param_);

        // Subscribe to twist commands from joystick controller
        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/tidybot/arm/twist", 10,
            std::bind(&JoystickToMoveit::twistCallback, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visual_ee_pose", marker_qos);
        
        // Publisher for joint trajectory commands
        if (output_type_ == OutputType::JointTrajectory)
        {
            joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/gen3_7dof_controller/joint_trajectory", 10);
            RCLCPP_INFO(this->get_logger(),"%sPublishing joint trajectories to /gen3_7dof_controller/joint_trajectory%s", GREEN.c_str(), RESET.c_str());
        }
        else
        {
            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
                "/tidybot/hardware/arm/commands", 10);
            RCLCPP_INFO(this->get_logger(), "%sPublishing joint states to /tidybot/hardware/arm/commands%s", GREEN.c_str(), RESET.c_str());
        }
    }
    
    void initialize() {
        // Get the Servo parameters
        const std::string param_namespace = "moveit_servo";
        
        servo_param_listener_ = std::make_shared<const servo::ParamListener>(shared_from_this(), param_namespace);
        servo_params_ = servo_param_listener_->get_params();

        // Create the planning scene monitor
        planning_scene_monitor_ = moveit_servo::createPlanningSceneMonitor(shared_from_this(), servo_params_);

        if (planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene())
        {
            planning_frame_ = planning_scene_monitor_->getPlanningScene()->getPlanningFrame();
        }

        if (planning_frame_.empty())
        {
            planning_frame_ = "bracelet_link";
        }
        RCLCPP_INFO(this->get_logger(), "%sPlanning frame configured to: %s%s", GREEN.c_str(), planning_frame_.c_str(), RESET.c_str());
        
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
        timer_period_sec_ = (update_period_sec_ > 0.0) ? update_period_sec_ : servo_params_.publish_period;
        servo_timer_ = this->create_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timer_period_sec_)),
            std::bind(&JoystickToMoveit::servoLoop, this));
        RCLCPP_INFO(this->get_logger(), "%sMoveit servo loop started with period %.3f sec%s", GREEN.c_str(), timer_period_sec_, RESET.c_str());
        RCLCPP_INFO(this->get_logger(), "%sServo initialized successfully. You can start planning!%s", GREEN.c_str(), RESET.c_str());
    }

private:
    enum class OutputType
    {
        JointTrajectory,
        JointState
    };

    OutputType parseOutputType(const std::string &output_type_param)
    {
        if (output_type_param == "joint_state")
        {
            return OutputType::JointState;
        }
        if (output_type_param != "joint_trajectory")
        {
            RCLCPP_WARN(this->get_logger(),
                        "Unknown command_output_type '%s'. Defaulting to 'joint_trajectory'", output_type_param.c_str());
        }
        return OutputType::JointTrajectory;
    }

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

        // RCLCPP_INFO(this->get_logger(), "Received twist command in frame %s: [lin: %.3f, %.3f, %.3f; ang: %.3f, %.3f, %.3f]",
        //              latest_twist_command_.header.frame_id.c_str(),
        //              twist_command.velocities[0], twist_command.velocities[1], twist_command.velocities[2],
        //              twist_command.velocities[3], twist_command.velocities[4], twist_command.velocities[5]);
        
        // Get current robot state
        auto robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
        
        // Get the next joint state from Servo
        KinematicState next_joint_state = servo_->getNextJointState(robot_state, twist_command);
        
        // Check servo status
        StatusCode status = servo_->getStatus();
        if (status != StatusCode::NO_WARNING) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Servo status: %d", static_cast<int>(status));
        }
        
        if (next_joint_state.joint_names.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Next joint state is empty, skipping publish");
            return;
        }

        if (output_type_ == OutputType::JointTrajectory)
        {
            trajectory_msgs::msg::JointTrajectory joint_trajectory;
            joint_trajectory.header.stamp = this->now();
            joint_trajectory.header.frame_id = "arm_base_link";
            joint_trajectory.joint_names = next_joint_state.joint_names;

            trajectory_msgs::msg::JointTrajectoryPoint start_point;
            start_point.positions.assign(last_commanded_state_.positions.data(),
                                        last_commanded_state_.positions.data() + last_commanded_state_.positions.size());
            if (last_commanded_state_.velocities.size() == last_commanded_state_.positions.size())
            {
                start_point.velocities.assign(last_commanded_state_.velocities.data(),
                                            last_commanded_state_.velocities.data() + last_commanded_state_.velocities.size());
            }
            start_point.time_from_start = rclcpp::Duration::from_nanoseconds(0);
            joint_trajectory.points.push_back(start_point);

            trajectory_msgs::msg::JointTrajectoryPoint end_point;
            // Convert Eigen vectors to std::vector
            end_point.positions.assign(next_joint_state.positions.data(),
                                   next_joint_state.positions.data() + next_joint_state.positions.size());
            end_point.velocities.assign(next_joint_state.velocities.data(),
                                    next_joint_state.velocities.data() + next_joint_state.velocities.size());
            end_point.time_from_start = rclcpp::Duration::from_nanoseconds(
                static_cast<int64_t>(servo_params_.publish_period * 1e9));

            joint_trajectory.points.push_back(end_point);
            joint_trajectory_pub_->publish(joint_trajectory);
        }
        else
        {
            sensor_msgs::msg::JointState joint_state_msg;
            joint_state_msg.header.stamp = this->now();
            joint_state_msg.name = next_joint_state.joint_names;
            joint_state_msg.position.assign(next_joint_state.positions.data(),
                                            next_joint_state.positions.data() + next_joint_state.positions.size());

            if (next_joint_state.velocities.size() == next_joint_state.positions.size())
            {
                joint_state_msg.velocity.assign(next_joint_state.velocities.data(),
                                                next_joint_state.velocities.data() + next_joint_state.velocities.size());
            }
            joint_state_pub_->publish(joint_state_msg);
        }
        publishVisualization(next_joint_state);
        last_commanded_state_ = next_joint_state;
        has_twist_command_ = false;  // Reset flag until next command
    }

    void publishVisualization(const KinematicState &state)
    {
        const auto state_monitor = planning_scene_monitor_->getStateMonitor();
        if (!state_monitor)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Planning scene state monitor not available for visualization");
            return;
        }

        auto current_state = state_monitor->getCurrentState();
        if (!current_state)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Unable to retrieve current robot state for visualization");
            return;
        }

        moveit::core::RobotState robot_state_copy(*current_state);
        for (size_t i = 0; i < state.joint_names.size(); ++i)
        {
            robot_state_copy.setVariablePosition(state.joint_names[i], state.positions[i]);
        }
        robot_state_copy.update();

        const Eigen::Isometry3d &ee_transform = robot_state_copy.getGlobalLinkTransform(end_effector_link_);
        geometry_msgs::msg::PoseStamped ee_pose;
        ee_pose.header.stamp = this->now();
        ee_pose.header.frame_id = planning_frame_;
        ee_pose.pose = tf2::toMsg(ee_transform);

        if (!base_frame_.empty() && base_frame_ != planning_frame_)
        {
            const std::string target_frame = base_frame_;
            const std::string source_frame = planning_frame_;
            geometry_msgs::msg::TransformStamped base_transform;
            geometry_msgs::msg::PoseStamped transformed_pose;
            try
            {
                if (tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.05)))
                {
                    base_transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                    tf2::doTransform(ee_pose, transformed_pose, base_transform);
                    ee_pose = transformed_pose;
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Transform %s -> %s not available", source_frame.c_str(), target_frame.c_str());
                }
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "TF transform failed: %s", ex.what());
            }
        }

        visualization_msgs::msg::Marker marker;
        marker.header = ee_pose.header;
        marker.ns = "twist_to_moveit";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = ee_pose.pose;
        marker.scale.x = 0.2;
        marker.scale.y = 0.04;
        marker.scale.z = 0.04;
        marker.color.r = 0.1f;
        marker.color.g = 0.8f;
        marker.color.b = 0.1f;
        marker.color.a = 0.9f;
        const double lifetime = std::max(timer_period_sec_, 0.05);
        const int64_t lifetime_ns = static_cast<int64_t>(lifetime * 1e9);
        marker.lifetime.sec = static_cast<int32_t>(lifetime_ns / static_cast<int64_t>(1e9));
        marker.lifetime.nanosec = static_cast<uint32_t>(lifetime_ns % static_cast<int64_t>(1e9));
        marker.frame_locked = false;

        marker_pub_->publish(marker);
    }
    
    std::shared_ptr<const servo::ParamListener> servo_param_listener_;
    servo::Params servo_params_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::unique_ptr<Servo> servo_;
    
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr servo_timer_;
    
    geometry_msgs::msg::TwistStamped latest_twist_command_;
    bool has_twist_command_ = false;
    KinematicState last_commanded_state_;

    std::string command_output_type_param_;
    OutputType output_type_ = OutputType::JointTrajectory;
    double update_period_sec_ = 0.0;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string planning_frame_;
    std::string end_effector_link_;
    std::string base_frame_;
    double timer_period_sec_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickToMoveit>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
