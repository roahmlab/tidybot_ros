/**
 * @file multi_stage_executor.cpp
 * @brief Task-agnostic multi-stage motion executor
 * 
 * This node receives sequences of MotionStage messages and executes them
 * in order. It supports PTP (point-to-point), LIN (linear), CIRC (circular),
 * and gripper actions. It does not know about specific tasks like "drawer" -
 * it simply executes the motion primitives it receives.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tidybot_utils/action/execute_stages.hpp>
#include <tidybot_utils/msg/motion_stage.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>
#include <cmath>

using namespace moveit::task_constructor;

class MultiStagePlanner : public rclcpp::Node
{
public:
    using ExecuteStages = tidybot_utils::action::ExecuteStages;
    using MotionStage = tidybot_utils::msg::MotionStage;
    using GoalHandleExecuteStages = rclcpp_action::ServerGoalHandle<ExecuteStages>;

    MultiStagePlanner() : Node("multi_stage_planner")
    {
        // Declare parameters
        this->declare_parameter("arm_group", "gen3_7dof");
        this->declare_parameter("tip_link", "tool_frame");
        
        action_server_ = rclcpp_action::create_server<ExecuteStages>(
            this,
            "execute_stages",
            std::bind(&MultiStagePlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MultiStagePlanner::handle_cancel, this, std::placeholders::_1),
            std::bind(&MultiStagePlanner::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Multi-Stage Executor initialized");
    }

    void init()
    {
        // Initialize PlanningSceneMonitor
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(), "robot_description");
        
        if (psm_->getPlanningScene()) {
            psm_->startStateMonitor();
            psm_->startSceneMonitor(); 
            psm_->startWorldGeometryMonitor(); 
            RCLCPP_INFO(this->get_logger(), "Planning Scene Monitor started");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning Scene Monitor failed to initialize");
        }

        // Pilz Linear Planner
        pilz_lin_planner_ = std::make_shared<solvers::PipelinePlanner>(
            shared_from_this(), "pilz_industrial_motion_planner");
        pilz_lin_planner_->setPlannerId("pilz_industrial_motion_planner", "LIN");
        pilz_lin_planner_->setMaxVelocityScalingFactor(0.1);
        pilz_lin_planner_->setMaxAccelerationScalingFactor(0.1);

        // Pilz Circular Planner
        pilz_circ_planner_ = std::make_shared<solvers::PipelinePlanner>(
            shared_from_this(), "pilz_industrial_motion_planner");
        pilz_circ_planner_->setPlannerId("pilz_industrial_motion_planner", "CIRC");
        pilz_circ_planner_->setMaxVelocityScalingFactor(0.1);
        pilz_circ_planner_->setMaxAccelerationScalingFactor(0.1);

        // Pilz Point-to-Point Planner
        pilz_ptp_planner_ = std::make_shared<solvers::PipelinePlanner>(
            shared_from_this(), "pilz_industrial_motion_planner");
        pilz_ptp_planner_->setPlannerId("pilz_industrial_motion_planner", "PTP");
        pilz_ptp_planner_->setMaxVelocityScalingFactor(0.2);
        pilz_ptp_planner_->setMaxAccelerationScalingFactor(0.2);

        // Initialize publishers
        arm_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gen3_7dof_controller/joint_trajectory", 10);
        gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/robotiq_2f_85_controller/commands", 10);
        
        
        RCLCPP_INFO(this->get_logger(), "Trajectory execution publishers initialized");
    }

private:
    rclcpp_action::Server<ExecuteStages>::SharedPtr action_server_;
    std::mutex executor_mutex_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;

    std::shared_ptr<solvers::PipelinePlanner> pilz_lin_planner_;
    std::shared_ptr<solvers::PipelinePlanner> pilz_circ_planner_;
    std::shared_ptr<solvers::PipelinePlanner> pilz_ptp_planner_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;


    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const ExecuteStages::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to execute %zu stages", 
                    goal->stages.size());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteStages> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteStages> goal_handle)
    {
        std::thread{std::bind(&MultiStagePlanner::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleExecuteStages> goal_handle)
    {
        std::lock_guard<std::mutex> lock(executor_mutex_);
        RCLCPP_INFO(this->get_logger(), "Executing multi-stage motion");
        
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ExecuteStages::Result>();
        auto feedback = std::make_shared<ExecuteStages::Feedback>();
        
        // Wait for current robot state
        if (!psm_->waitForCurrentRobotState(this->get_clock()->now(), 5.0)) {
            RCLCPP_WARN(this->get_logger(), "Timeout waiting for current robot state!");
        }

        // Get parameters
        std::string arm_group = this->get_parameter("arm_group").as_string();
        std::string tip_link = this->get_parameter("tip_link").as_string();

        // Get robot model and state
        auto robot_model = psm_->getRobotModel();
        auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
        robot_state->setToDefaultValues();
        
        {
            planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
            robot_state->update();
            *robot_state = scene->getCurrentState();
        }
        
        const auto* joint_model_group = robot_model->getJointModelGroup(arm_group);
        if (!joint_model_group) {
            RCLCPP_ERROR(this->get_logger(), "Joint model group not found: %s", arm_group.c_str());
            result->success = false;
            result->message = "Joint model group not found";
            result->stages_completed = 0;
            goal_handle->abort(result);
            return;
        }

        // Execute each stage
        int stages_completed = 0;
        for (size_t i = 0; i < goal->stages.size(); ++i) {
            const auto& stage = goal->stages[i];
            
            // Check for cancellation
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Cancelled";
                result->stages_completed = stages_completed;
                goal_handle->canceled(result);
                return;
            }

            // Publish feedback
            feedback->current_stage_index = static_cast<int32_t>(i);
            feedback->current_stage_description = stage.description;
            feedback->stage_progress = 0.0f;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "Executing stage %zu: %s (type=%d)", 
                        i, stage.description.c_str(), stage.stage_type);

            bool stage_success = false;
            switch (stage.stage_type) {
                case MotionStage::STAGE_PTP:
                    stage_success = execute_ptp(stage, robot_state, joint_model_group, tip_link);
                    break;
                case MotionStage::STAGE_LIN:
                    stage_success = execute_lin(stage, robot_state, joint_model_group, tip_link);
                    break;
                case MotionStage::STAGE_CIRC:
                    stage_success = execute_circ(stage, robot_state, joint_model_group, tip_link);
                    break;
                case MotionStage::STAGE_GRIPPER:
                    stage_success = execute_gripper(stage);
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown stage type: %d", stage.stage_type);
                    stage_success = false;
            }

            if (!stage_success) {
                result->success = false;
                result->message = "Failed at stage: " + stage.description;
                result->stages_completed = stages_completed;
                goal_handle->abort(result);
                return;
            }

            // Synchronize robot_state with actual robot position after motion
            // This ensures the next stage computes IK from the correct starting position
            if (stage.stage_type != MotionStage::STAGE_GRIPPER) {
                // Wait for robot to settle and state to update
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                
                // Request fresh state from planning scene
                if (!psm_->waitForCurrentRobotState(this->get_clock()->now(), 2.0)) {
                    RCLCPP_WARN(this->get_logger(), "Timeout waiting for updated robot state");
                }
                
                {
                    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
                    *robot_state = scene->getCurrentState();
                }
                RCLCPP_DEBUG(this->get_logger(), "Robot state synchronized after stage %zu", i);
            }

            stages_completed++;
            feedback->stage_progress = 1.0f;
            goal_handle->publish_feedback(feedback);
        }

        // Success
        result->success = true;
        result->message = "All stages completed successfully";
        result->stages_completed = stages_completed;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Multi-stage execution completed successfully");
    }

    bool execute_ptp(const MotionStage& stage, 
                     std::shared_ptr<moveit::core::RobotState> robot_state,
                     const moveit::core::JointModelGroup* joint_model_group,
                     const std::string& tip_link)
    {
        RCLCPP_INFO(this->get_logger(), "PTP motion to target");
        
        // Solve IK with tight tolerance
        kinematics::KinematicsQueryOptions ik_options;
        ik_options.return_approximate_solution = false;  // Require exact solution
        
        bool found_ik = robot_state->setFromIK(
            joint_model_group, 
            stage.target_pose,
            tip_link,
            1.0,  // Timeout (increased for better solutions)
            moveit::core::GroupStateValidityCallbackFn(),
            ik_options
        );
        
        if (!found_ik) {
            RCLCPP_ERROR(this->get_logger(), "IK failed for PTP motion");
            return false;
        }
        
        robot_state->enforceBounds();
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(joint_model_group, joint_values);
        
        // Publish trajectory
        double duration = stage.duration > 0.0 ? stage.duration : 2.0;
        publish_trajectory(joint_model_group, joint_values, duration);
        
        // Wait for execution
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<int>(duration * 1000) + 500));
        
        return true;
    }

    bool execute_lin(const MotionStage& stage, 
                     std::shared_ptr<moveit::core::RobotState> robot_state,
                     const moveit::core::JointModelGroup* joint_model_group,
                     const std::string& tip_link)
    {
        RCLCPP_INFO(this->get_logger(), "LIN motion to target");
        
        // For now, use IK-based approach similar to PTP
        // TODO: Use Pilz LIN planner for true Cartesian linear motion
        return execute_ptp(stage, robot_state, joint_model_group, tip_link);
    }

    bool execute_circ(const MotionStage& stage, 
                      std::shared_ptr<moveit::core::RobotState> robot_state,
                      const moveit::core::JointModelGroup* joint_model_group,
                      const std::string& tip_link)
    {
        RCLCPP_INFO(this->get_logger(), "CIRC motion around axis");
        
        // Get arc parameters
        Eigen::Vector3d arc_center(
            stage.arc_center.x,
            stage.arc_center.y,
            stage.arc_center.z
        );
        Eigen::Vector3d arc_axis(
            stage.arc_axis.x,
            stage.arc_axis.y,
            stage.arc_axis.z
        );
        arc_axis.normalize();
        double arc_angle = stage.arc_angle;
        
        // Get current end-effector pose
        const Eigen::Isometry3d& current_pose = robot_state->getGlobalLinkTransform(tip_link);
        Eigen::Vector3d current_pos = current_pose.translation();
        Eigen::Quaterniond current_quat(current_pose.rotation());
        
        // Calculate relative position from arc center
        Eigen::Vector3d rel_pos = current_pos - arc_center;
        
        // Generate waypoints along the arc
        // Use more waypoints for smoother motion and better IK continuity
        int num_waypoints = 30;  // Increased from 10 for smoother arc
        double angle_step = arc_angle / num_waypoints;
        double duration_per_waypoint = (stage.duration > 0.0 ? stage.duration : 2.0) / num_waypoints;
        
        RCLCPP_INFO(this->get_logger(), "Generating %d waypoints, arc_angle=%.3f rad (%.1f deg)", 
                    num_waypoints, arc_angle, arc_angle * 180.0 / M_PI);
        
        // Store previous joint values for continuity checking
        std::vector<double> prev_joint_values;
        robot_state->copyJointGroupPositions(joint_model_group, prev_joint_values);
        
        for (int i = 1; i <= num_waypoints; ++i) {
            double angle = angle_step * i;
            
            // Create rotation around arc axis
            Eigen::AngleAxisd rotation(angle, arc_axis);
            
            // Rotate position around arc center
            Eigen::Vector3d new_rel_pos = rotation * rel_pos;
            Eigen::Vector3d waypoint_pos = arc_center + new_rel_pos;
            
            // Also rotate the gripper orientation
            Eigen::Quaterniond waypoint_quat = Eigen::Quaterniond(rotation) * current_quat;
            waypoint_quat.normalize();
            
            // Create waypoint pose
            Eigen::Isometry3d waypoint_pose = Eigen::Isometry3d::Identity();
            waypoint_pose.translation() = waypoint_pos;
            waypoint_pose.linear() = waypoint_quat.toRotationMatrix();
            
            // Solve IK for this waypoint
            // The robot_state already contains previous solution, which seeds the IK
            kinematics::KinematicsQueryOptions ik_options;
            ik_options.return_approximate_solution = false;
            
            bool found_ik = robot_state->setFromIK(
                joint_model_group, 
                waypoint_pose, 
                tip_link, 
                0.5,  // Timeout per waypoint
                moveit::core::GroupStateValidityCallbackFn(),
                ik_options
            );
            
            if (!found_ik) {
                RCLCPP_ERROR(this->get_logger(), "IK failed for arc waypoint %d", i);
                return false;
            }
            
            // Get joint values
            std::vector<double> joint_values;
            robot_state->copyJointGroupPositions(joint_model_group, joint_values);
            
            // Check for large joint jumps (discontinuity detection)
            double max_jump = 0.0;
            for (size_t j = 0; j < joint_values.size(); ++j) {
                double jump = std::abs(joint_values[j] - prev_joint_values[j]);
                if (jump > max_jump) max_jump = jump;
            }
            
            // Warn if jump is too large (> 30 degrees = 0.52 rad)
            if (max_jump > 0.52) {
                RCLCPP_WARN(this->get_logger(), 
                    "Large joint jump detected at waypoint %d (%.2f rad)", i, max_jump);
            }
            
            // Update previous values for next iteration
            prev_joint_values = joint_values;
            
            // Publish trajectory for this waypoint
            publish_trajectory(joint_model_group, joint_values, duration_per_waypoint);
            
            // Wait for execution (shorter wait for more waypoints)
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(duration_per_waypoint * 1000) + 100));
        }
        
        RCLCPP_INFO(this->get_logger(), "CIRC motion completed");
        return true;
    }

    bool execute_gripper(const MotionStage& stage)
    {
        RCLCPP_INFO(this->get_logger(), "Gripper action: position=%.2f", stage.gripper_position);
        
        // Scale the gripper position (0.0-1.0 input maps to 0.0-0.8 command)
        // This matches the scaling used in moveit_ee_pose_ik for phone teleoperation
        double scaled_position = stage.gripper_position * 0.8;
        
        // Send command to both joints (finger_joint, right_outer_knuckle_joint)
        // Note: right_outer_knuckle_joint axis is flipped, so we send negative command
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {scaled_position, -scaled_position};
        gripper_pub_->publish(msg);

        // Publish command multiple times to ensure controller processes it
        double duration = stage.duration > 0.0 ? stage.duration : 1.0;
        int num_publishes = 10;
        int interval_ms = static_cast<int>((duration * 1000) / num_publishes);
        
        for (int i = 0; i < num_publishes; ++i) {
            gripper_pub_->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }
        
        // Extra time for gripper to settle
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        return true;
    }

    void publish_trajectory(const moveit::core::JointModelGroup* joint_model_group,
                           const std::vector<double>& joint_values,
                           double duration)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_model_group->getVariableNames();
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_values;
        point.time_from_start = rclcpp::Duration::from_seconds(duration);
        traj.points.push_back(point);
        
        arm_traj_pub_->publish(traj);
        RCLCPP_INFO(this->get_logger(), "Published trajectory with %zu joints, duration=%.2fs",
                    joint_values.size(), duration);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiStagePlanner>();
    
    // Need to spin briefly before init() to allow shared_from_this() to work
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // Initialize after node is added to executor
    std::thread init_thread([&node]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        node->init();
    });
    
    executor.spin();
    init_thread.join();
    rclcpp::shutdown();
    return 0;
}
