/**
 * @file synchronized_recorder.cpp
 * @brief Synchronized Episode Recorder for TidyBot Platform
 * 
 * This node records both control inputs (actions) and robot observations at a fixed rate,
 * ensuring proper temporal alignment for imitation learning datasets.
 * 
 * Directory Structure:
 * Each episode is stored in a timestamped directory containing:
 * - actions/          : ROS2 bag with control commands (base, arm, gripper)
 * - observations/     : ROS2 bag with robot state (poses, joint states)
 * - base_camera.mp4   : Video from base camera
 * - arm_camera.mp4    : Video from arm camera
 * - ext_camera.mp4    : Video from external camera
 * 
 * Services:
 * - /start_recording  : Start recording an episode
 * - /stop_recording   : Stop recording and save data
 * 
 * Parameters:
 * - storage_uri (string): Base directory for episode storage (default: "episode_bag")
 * - use_sim (bool): Use simulation topics (true) or real robot topics (false)
 * - action_synchronized (bool): Observations are written only when actions are received (default: true)
 * - fps (double): Recording frequency in Hz (default: 10.0)
 * - cameras (string list): Subset of camera streams to record (choices: base, arm, ext; default: all)
 */

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <filesystem>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

constexpr const char* GREEN = "\x1b[32m";
constexpr const char* YELLOW = "\x1b[33m";
constexpr const char* RED = "\x1b[31m";
constexpr const char* RESET = "\x1b[0m";
constexpr const char* BOLD = "\x1b[1m";

class SynchronizedRecorder : public rclcpp::Node
{
private:
    // Observation subscriptions
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState::ConstSharedPtr last_joint_state_;
    
    // TF listener for poses
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Camera subscriptions
    image_transport::Subscriber base_image_sub_;
    image_transport::Subscriber arm_image_sub_;
    image_transport::Subscriber ext_image_sub_;
    sensor_msgs::msg::Image::ConstSharedPtr last_base_image_;
    sensor_msgs::msg::Image::ConstSharedPtr last_arm_image_;
    sensor_msgs::msg::Image::ConstSharedPtr last_ext_image_;

    // Tactile sensor subscriptions
    rclcpp::GenericSubscription::SharedPtr sensor_0_sub_;
    rclcpp::GenericSubscription::SharedPtr sensor_1_sub_;
    std::shared_ptr<rclcpp::SerializedMessage> latest_sensor_0_;
    std::shared_ptr<rclcpp::SerializedMessage> latest_sensor_1_;

    // Latest observation buffers (updated outside writer loop)
    geometry_msgs::msg::PoseStamped latest_base_pose_;
    geometry_msgs::msg::PoseStamped latest_arm_pose_;
    std_msgs::msg::Float64 latest_gripper_state_;
    bool have_base_pose_ = false;
    bool have_arm_pose_ = false;
    bool have_gripper_state_ = false;

    // Action command subscriptions (cache latest commands)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr base_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_cmd_sub_;
    
    // Cached latest commands
    std_msgs::msg::Float64MultiArray::ConstSharedPtr last_base_cmd_;
    geometry_msgs::msg::Pose::ConstSharedPtr last_arm_cmd_;
    std_msgs::msg::Float64::ConstSharedPtr last_gripper_cmd_;

    // Pending write triggers (incremented by any action callback)
    size_t pending_writes_ = 0;
    bool initialized_from_obs_ = false;

    // Recording services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_recording_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_recording_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr finalize_recording_service_;

    // Rosbag writers
    rosbag2_cpp::Writer actions_writer_;
    rosbag2_cpp::Writer obs_writer_;
    rosbag2_storage::StorageOptions actions_storage_options_;
    rosbag2_storage::StorageOptions obs_storage_options_;
    rosbag2_cpp::ConverterOptions converter_options_;

    // Video writers
    cv::VideoWriter base_video_writer_;
    cv::VideoWriter arm_video_writer_;
    cv::VideoWriter ext_video_writer_;
    bool base_video_initialized_ = false;
    bool arm_video_initialized_ = false;
    bool ext_video_initialized_ = false;
    std::string base_video_filename_;
    std::string arm_video_filename_;
    std::string ext_video_filename_;
    bool record_base_camera_ = true;
    bool record_arm_camera_ = true;
    bool record_ext_camera_ = true;

    // Synchronized recording timer
    rclcpp::TimerBase::SharedPtr record_timer_;
    
    // Parameters
    std::string storage_uri_;
    bool use_sim_;
    bool recording_enabled_ = false;
    bool action_synchronized_ = true;
    bool tactile_enabled_ = false;
    double fps_;
    rclcpp::Time last_record_time_;
    
    // Sample counter for alignment
    uint64_t sample_count_ = 0;

    // Track current episode directory for finalize step
    std::string current_episode_dir_;

public:
    SynchronizedRecorder()
        : Node("synchronized_recorder"),
          recording_enabled_(false),
          sample_count_(0)
    {
        // Declare parameters
        declare_parameter<std::string>("storage_uri", "episode_bag");
        declare_parameter<double>("fps", 10.0);
        declare_parameter<std::vector<std::string>>("cameras", std::vector<std::string>{"base", "arm", "ext"});
        declare_parameter<bool>("tactile_enabled", false);
        declare_parameter<bool>("action_synchronized", true);

        get_parameter("storage_uri", storage_uri_);
        get_parameter("fps", fps_);
        std::vector<std::string> camera_param;
        get_parameter("cameras", camera_param);
        get_parameter("tactile_enabled", tactile_enabled_);
        get_parameter("action_synchronized", action_synchronized_);
        configure_camera_selection(camera_param);

        // Initialize TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize storage options
        actions_storage_options_.storage_id = "sqlite3";
        obs_storage_options_.storage_id = "sqlite3";
        converter_options_.input_serialization_format = "cdr";
        converter_options_.output_serialization_format = "cdr";

        // Setup observation subscriptions
        setup_observation_subscriptions();
        
        // Setup action command subscriptions
        setup_action_subscriptions();

        // Create recording services
        start_recording_service_ = this->create_service<std_srvs::srv::Empty>(
            "/start_recording",
            std::bind(&SynchronizedRecorder::start_recording_callback, this, std::placeholders::_1, std::placeholders::_2));
        stop_recording_service_ = this->create_service<std_srvs::srv::Empty>(
            "/stop_recording",
            std::bind(&SynchronizedRecorder::stop_recording_callback, this, std::placeholders::_1, std::placeholders::_2));
        finalize_recording_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/finalize_recording",
            std::bind(&SynchronizedRecorder::finalize_recording_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create synchronized recording timer
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps_));
        record_timer_ = create_wall_timer(period, std::bind(&SynchronizedRecorder::synchronized_record_callback, this));

        RCLCPP_INFO(this->get_logger(), "%s%sEpisode Recorder Node Initialized%s", GREEN, BOLD, RESET);
        RCLCPP_INFO(this->get_logger(), "%s%sStorage URI: %s%s", GREEN, BOLD, storage_uri_.c_str(), RESET);
        RCLCPP_INFO(this->get_logger(), "%s%sRecording frequency: %.1f Hz%s", GREEN, BOLD, fps_, RESET);
        RCLCPP_INFO(this->get_logger(), "%s%sUse simulation topics parameter retained for compatibility: %s%s",
                GREEN, BOLD, use_sim_ ? "true" : "false", RESET);
    }

private:
    void setup_observation_subscriptions()
    {
        // Joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                last_joint_state_ = msg;
                // Update gripper state buffer immediately if possible
                auto it = std::find(msg->name.begin(), msg->name.end(), "finger_joint");
                if (it != msg->name.end())
                {
                    size_t index = std::distance(msg->name.begin(), it);
                    if (index < msg->position.size())
                    {
                        latest_gripper_state_.data = msg->position[index];
                        have_gripper_state_ = true;
                    }
                }
            });
        // Camera images (only subscribe to requested streams)
        if (record_base_camera_)
        {
            base_image_sub_ = image_transport::create_subscription(
                this, "/tidybot/camera_base/color/raw",
                [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
                    last_base_image_ = msg;
                }, "raw");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "%sBase camera recording disabled%s", YELLOW, RESET);
        }

        if (record_arm_camera_)
        {
            rclcpp::QoS qos(1);
            qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
            arm_image_sub_ = image_transport::create_subscription(
                this, "/tidybot/camera_wrist/color/raw",
                [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
                    last_arm_image_ = msg;
                }, "raw", qos.get_rmw_qos_profile());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "%sArm camera recording disabled%s", YELLOW, RESET);
        }

        if (record_ext_camera_)
        {
            rclcpp::QoS qos(1);
            qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
            ext_image_sub_ = image_transport::create_subscription(
                this, "/tidybot/camera_ext/color/raw",
                [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
                    last_ext_image_ = msg;
                }, "raw", qos.get_rmw_qos_profile());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "%sExternal camera recording disabled%s", YELLOW, RESET);
        }

        if (tactile_enabled_)
        {
            rclcpp::QoS qos(1);
            qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

            sensor_0_sub_ = this->create_generic_subscription(
                "/hub_0/sensor_0",
                "sensor_interfaces/msg/SensorState",
                qos,
                [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    latest_sensor_0_ = msg;
                }
            );
            sensor_1_sub_ = this->create_generic_subscription(
                "/hub_0/sensor_1",
                "sensor_interfaces/msg/SensorState",
                qos,
                [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    latest_sensor_1_ = msg;
                }
            );

            RCLCPP_INFO(this->get_logger(), "%sTactile sensor recording enabled%s", GREEN, RESET);
        }
    }

    void setup_action_subscriptions()
    {
        // Subscribe to unified action topics requested for dataset capture.
        base_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tidybot/base/target_pose", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                last_base_cmd_ = msg;
                if (recording_enabled_ && action_synchronized_) { pending_writes_++; }
            });
        arm_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/tidybot/arm/target_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                last_arm_cmd_ = msg;
                if (recording_enabled_ && action_synchronized_) { pending_writes_++; }
            });
        gripper_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/tidybot/gripper/commands", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                last_gripper_cmd_ = msg;
                if (recording_enabled_ && action_synchronized_) { pending_writes_++; }
            });
    }

    void configure_camera_selection(const std::vector<std::string> &camera_names)
    {
        record_base_camera_ = record_arm_camera_ = record_ext_camera_ = false;
        std::string active_log;

        for (auto name : camera_names)
        {
            std::transform(name.begin(), name.end(), name.begin(), ::tolower);
            if (name == "base")      record_base_camera_ = true;
            else if (name == "arm")  record_arm_camera_ = true;
            else if (name == "ext")  record_ext_camera_ = true;
            else {
                RCLCPP_WARN(this->get_logger(), "%sUnknown camera: '%s' (expected base, arm, ext)%s", 
                            YELLOW, name.c_str(), RESET);
                continue;
            }
            
            active_log += (active_log.empty() ? "" : ", ") + name;
        }

        if (active_log.empty()) {
            RCLCPP_WARN(this->get_logger(), "%sNo valid cameras; video capture disabled%s", YELLOW, RESET);
        } else {
            RCLCPP_INFO(this->get_logger(), "%sRecording cameras: %s%s", GREEN, active_log.c_str(), RESET);
        }
    }

    void start_recording_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        (void)request;
        (void)response;
        
        if (recording_enabled_)
        {
            RCLCPP_WARN(this->get_logger(), "%sAlready recording episode%s", YELLOW, RESET);
            return;
        }

        recording_enabled_ = true;
        pending_writes_ = 0;
        sample_count_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "%sStarting synchronized episode recording%s", GREEN, RESET);
        
        // Create episode directory
        std::string timestamp = get_timestamped_filename("episode");
        std::string episode_dir = storage_uri_ + "/" + timestamp;
        current_episode_dir_ = episode_dir;
        
        // Create the episode directory
        std::string mkdir_cmd = "mkdir -p " + episode_dir;
        int result = system(mkdir_cmd.c_str());
        if (result != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "%sFailed to create episode directory: %s%s", RED, episode_dir.c_str(), RESET);
            recording_enabled_ = false;
            current_episode_dir_.clear();
            return;
        }
        
        // Setup actions bag inside episode directory
        actions_storage_options_.uri = episode_dir + "/actions";
        actions_writer_.open(actions_storage_options_, converter_options_);
        
        // Setup observations bag inside episode directory
        obs_storage_options_.uri = episode_dir + "/observations";
        obs_writer_.open(obs_storage_options_, converter_options_);
        
        // Setup video files inside episode directory (only for enabled cameras)
        base_video_filename_ = record_base_camera_ ? episode_dir + "/base_camera.mp4" : std::string();
        arm_video_filename_ = record_arm_camera_ ? episode_dir + "/arm_camera.mp4" : std::string();
        ext_video_filename_ = record_ext_camera_ ? episode_dir + "/ext_camera.mp4" : std::string();
        
        // Prime observation buffers immediately
        update_observation_buffers();
        seed_initial_actions_from_observation();
        initialized_from_obs_ = true;

        RCLCPP_INFO(this->get_logger(), "%sRecording episode to: %s%s", GREEN, episode_dir.c_str(), RESET);
    }

    void stop_recording_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        (void)request;
        (void)response;
        
        if (!recording_enabled_)
        {
            RCLCPP_WARN(this->get_logger(), "No recording in progress to stop.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "%sStopping synchronized episode recording%s", GREEN, RESET);
        
        // Close writers
        actions_writer_.close();
        obs_writer_.close();
        
        // Close video writers
        if (base_video_writer_.isOpened())
        {
            base_video_writer_.release();
            RCLCPP_INFO(this->get_logger(), "Base video closed: %s", base_video_filename_.c_str());
        }
        if (arm_video_writer_.isOpened())
        {
            arm_video_writer_.release();
            RCLCPP_INFO(this->get_logger(), "Arm video closed: %s", arm_video_filename_.c_str());
        }
        if (ext_video_writer_.isOpened())
        {
            ext_video_writer_.release();
            RCLCPP_INFO(this->get_logger(), "Ext video closed: %s", ext_video_filename_.c_str());
        }

        recording_enabled_ = false;
        base_video_initialized_ = false;
        arm_video_initialized_ = false;
        ext_video_initialized_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Recorded %lu synchronized samples", sample_count_);
        
        // Inform that episode is awaiting finalize decision
        if (!current_episode_dir_.empty()) {
            RCLCPP_INFO(this->get_logger(), "%sEpisode recorded at: %s (awaiting finalize)%s", GREEN, current_episode_dir_.c_str(), RESET);
            RCLCPP_INFO(this->get_logger(), "%s  - Actions: %s%s", GREEN, actions_storage_options_.uri.c_str(), RESET);
            RCLCPP_INFO(this->get_logger(), "%s  - Observations: %s%s", GREEN, obs_storage_options_.uri.c_str(), RESET);
        }
    }

    void finalize_recording_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (recording_enabled_) {
            response->success = false;
            response->message = "Cannot finalize while recording is active";
            RCLCPP_WARN(this->get_logger(), "%sCannot finalize while recording is active%s", YELLOW, RESET);
            return;
        }
        if (current_episode_dir_.empty()) {
            response->success = false;
            response->message = "No episode to finalize";
            RCLCPP_WARN(this->get_logger(), "%sNo episode to finalize%s", YELLOW, RESET);
            return;
        }

        bool save = request->data;
        if (save) {
            response->success = true;
            response->message = "Episode saved";
            RCLCPP_INFO(this->get_logger(), "%sEpisode saved at: %s%s", GREEN, current_episode_dir_.c_str(), RESET);
            current_episode_dir_.clear();
            return;
        }

        // Discard: remove directory and all contents
        std::error_code ec;
        std::filesystem::remove_all(current_episode_dir_, ec);
        if (ec) {
            response->success = false;
            response->message = std::string("Failed to delete episode: ") + ec.message();
            RCLCPP_ERROR(this->get_logger(), "%s%s%s", RED, response->message.c_str(), RESET);
        } else {
            response->success = true;
            response->message = "Episode discarded";
            RCLCPP_INFO(this->get_logger(), "%sEpisode discarded: %s%s", GREEN, current_episode_dir_.c_str(), RESET);
        }
        current_episode_dir_.clear();
    }

    void synchronized_record_callback()
    {
        // Handle time jumps
        if (last_record_time_.nanoseconds() > this->get_clock()->now().nanoseconds())
        {
            RCLCPP_WARN(this->get_logger(), "Time jump detected, resetting TF buffer and listener...");
            // Fully destroy and recreate TF structures
            tf_listener_.reset();
            tf_buffer_.reset();
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            // Update last record time to current to avoid repeated resets
            last_record_time_ = this->get_clock()->now();
            return;
        }
        
        last_record_time_ = this->get_clock()->now();
        
        if (!recording_enabled_)
        {
            return;
        }

        // Always update observation buffers; do not write unless triggered
        update_observation_buffers();

        // Only write when there is a pending trigger from an action message
        if ((!action_synchronized_ || pending_writes_ > 0) && perform_write_once())
        {
            if (action_synchronized_) pending_writes_--;
            sample_count_++;
            if (sample_count_ % 100 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Recorded %lu synchronized samples", sample_count_);
            }
        }
    }

    // Update observation buffers (no writes)
    void update_observation_buffers()
    {
        auto current_time = this->get_clock()->now();
        geometry_msgs::msg::PoseStamped identity_pose;
        identity_pose.header.stamp = rclcpp::Time(0);
        identity_pose.pose.orientation.w = 1.0;

        // Base pose
        try
        {
            identity_pose.header.frame_id = "base";
            latest_base_pose_ = tf_buffer_->transform(identity_pose, "world");
            latest_base_pose_.header.stamp = current_time;
            have_base_pose_ = true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Could not get base pose: %s", ex.what());
        }
        // Arm pose
        try
        {
            identity_pose.header.frame_id = "bracelet_link";
            latest_arm_pose_ = tf_buffer_->transform(identity_pose, "arm_base_link");
            latest_arm_pose_.header.stamp = current_time;
            have_arm_pose_ = true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Could not get arm pose: %s", ex.what());
        }
        // Gripper state already updated in joint state callback; if not available, try to compute now
        if (!have_gripper_state_ && last_joint_state_)
        {
            auto it = std::find(last_joint_state_->name.begin(), last_joint_state_->name.end(), "finger_joint");
            if (it != last_joint_state_->name.end())
            {
                size_t index = std::distance(last_joint_state_->name.begin(), it);
                if (index < last_joint_state_->position.size())
                {
                    latest_gripper_state_.data = last_joint_state_->position[index];
                    have_gripper_state_ = true;
                }
            }
        }
    }

    void seed_initial_actions_from_observation()
    {
        // Only seed if we have obs; otherwise leave as null until first obs arrives
        if (have_base_pose_)
        {
            auto seed = std::make_shared<std_msgs::msg::Float64MultiArray>();
            seed->data.resize(3);
            // compute yaw
            double roll, pitch, yaw;
            tf2::Quaternion q;
            tf2::fromMsg(latest_base_pose_.pose.orientation, q);
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            seed->data[0] = latest_base_pose_.pose.position.x;
            seed->data[1] = latest_base_pose_.pose.position.y;
            seed->data[2] = yaw;
            last_base_cmd_ = seed;
        }
        if (have_arm_pose_)
        {
            auto seed = std::make_shared<geometry_msgs::msg::Pose>(latest_arm_pose_.pose);
            last_arm_cmd_ = seed;
        }
        if (have_gripper_state_)
        {
            auto seed = std::make_shared<std_msgs::msg::Float64>(latest_gripper_state_);
            last_gripper_cmd_ = seed;
        }
    }

    bool perform_write_once()
    {
        auto check_missing = [&](bool req, const auto& ptr, const char* name) {
            if (req && !ptr) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "Skipping write: waiting for %s", name);
                return true;
            }
            return false;
        };

        if (check_missing(record_base_camera_, last_base_image_, "base camera")) return false;
        if (check_missing(record_arm_camera_, last_arm_image_, "arm camera"))    return false;
        if (check_missing(record_ext_camera_, last_ext_image_, "ext camera"))    return false;

        // Ensure we have observations
        if (!(have_base_pose_ && have_arm_pose_ && have_gripper_state_))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Skipping write: observations not ready");
            return false;
        }

        // Ensure we have initial action buffers; if not, seed from obs
        if (!last_base_cmd_ || !last_arm_cmd_ || !last_gripper_cmd_)
        {
            seed_initial_actions_from_observation();
        }

        // Write observations
        write_observation_message<geometry_msgs::msg::PoseStamped>(
            std::make_shared<geometry_msgs::msg::PoseStamped>(latest_base_pose_), "/base_pose");
        write_observation_message<geometry_msgs::msg::PoseStamped>(
            std::make_shared<geometry_msgs::msg::PoseStamped>(latest_arm_pose_), "/arm_pose");
        write_observation_message<std_msgs::msg::Float64>(
            std::make_shared<std_msgs::msg::Float64>(latest_gripper_state_), "/gripper_state");
        write_observation_message<sensor_msgs::msg::JointState>(
            std::make_shared<sensor_msgs::msg::JointState>(*last_joint_state_), "/joint_states");
        
        if (tactile_enabled_)
        {
            obs_writer_.write(
                latest_sensor_0_, "/hub_0/sensor_0", 
                "sensor_interfaces/msg/SensorState", 
                this->get_clock()->now()
            );
            obs_writer_.write(
                latest_sensor_1_, "/hub_0/sensor_1", 
                "sensor_interfaces/msg/SensorState", 
                this->get_clock()->now()
            );
        }

        // Write actions (use latest cached commands)
        if (last_base_cmd_)    write_action_message<std_msgs::msg::Float64MultiArray>(std::make_shared<std_msgs::msg::Float64MultiArray>(*last_base_cmd_), "/tidybot/base/target_pose");
        if (last_arm_cmd_)     write_action_message<geometry_msgs::msg::Pose>(std::make_shared<geometry_msgs::msg::Pose>(*last_arm_cmd_), "/tidybot/arm/target_pose");
        if (last_gripper_cmd_) write_action_message<std_msgs::msg::Float64>(std::make_shared<std_msgs::msg::Float64>(*last_gripper_cmd_), "/tidybot/gripper/commands");
    
        // Write camera frames only for requested streams
        if (record_base_camera_ || record_arm_camera_ || record_ext_camera_) {
            try {
                process_video_frame(record_base_camera_, last_base_image_, base_video_writer_, base_video_initialized_, base_video_filename_);
                process_video_frame(record_arm_camera_, last_arm_image_, arm_video_writer_, arm_video_initialized_, arm_video_filename_);
                process_video_frame(record_ext_camera_, last_ext_image_, ext_video_writer_, ext_video_initialized_, ext_video_filename_);
            } catch (const cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "%sCvBridge Error: %s%s", RED, e.what(), RESET);
                return false;
            }
        }

        return true;
    }

    void process_video_frame(bool enabled, const sensor_msgs::msg::Image::ConstSharedPtr& img, 
                            cv::VideoWriter& writer, bool& is_init, const std::string& filename)
    {
        if (!enabled || !img) return;

        auto cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        
        if (!is_init && !filename.empty()) {
            writer.open(filename, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps_, cv::Size(cv_ptr->image.cols, cv_ptr->image.rows), true);
            is_init = writer.isOpened();
            if (!is_init) RCLCPP_ERROR(this->get_logger(), "%sFailed to open video: %s%s", RED, filename.c_str(), RESET);
        }
        
        if (is_init) writer << cv_ptr->image;
    }

    template <typename MsgT>
    void write_action_message(const typename MsgT::SharedPtr &msg, const std::string &topic_name)
    {
        write_message_to_bag<MsgT>(actions_writer_, msg, topic_name);
    }

    template <typename MsgT>
    void write_observation_message(const typename MsgT::SharedPtr &msg, const std::string &topic_name)
    {
        write_message_to_bag<MsgT>(obs_writer_, msg, topic_name);
    }

    template <typename MsgT>
    void write_message_to_bag(rosbag2_cpp::Writer &writer, const typename MsgT::SharedPtr &msg, const std::string &topic_name)
    {
        auto serialized = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<MsgT> serializer;
        serializer.serialize_message(msg.get(), serialized.get());

        const std::string type_name = rosidl_generator_traits::name<MsgT>();
        writer.write(serialized, topic_name, type_name, this->get_clock()->now());
    }

    std::string get_timestamped_filename(const std::string &prefix)
    {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm;
        
#if defined(_MSC_VER)
        localtime_s(&local_tm, &t);
#else
        localtime_r(&t, &local_tm);
#endif

        std::ostringstream oss;
        if (!prefix.empty()) {
            oss << prefix << "_";
        }
        oss << std::put_time(&local_tm, "%Y_%m_%d_%H_%M_%S");
        return oss.str();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SynchronizedRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
