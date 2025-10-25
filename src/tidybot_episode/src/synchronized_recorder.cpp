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
 * 
 * Services:
 * - /start_recording  : Start recording an episode
 * - /stop_recording   : Stop recording and save data
 * 
 * Parameters:
 * - storage_uri (string): Base directory for episode storage (default: "episode_bag")
 * - use_sim (bool): Use simulation topics (true) or real robot topics (false)
 * - fps (double): Recording frequency in Hz (default: 10.0)
 */

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <memory>
#include <ctime>
#include <cstdlib>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

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
    sensor_msgs::msg::Image::ConstSharedPtr last_base_image_;
    sensor_msgs::msg::Image::ConstSharedPtr last_arm_image_;

    // Action command subscriptions (cache latest commands)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr base_cmd_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_cmd_sub_;
    
    // Cached latest commands
    std_msgs::msg::Float64MultiArray::ConstSharedPtr last_base_cmd_;
    trajectory_msgs::msg::JointTrajectory::ConstSharedPtr last_arm_cmd_;
    std_msgs::msg::Float64MultiArray::ConstSharedPtr last_gripper_cmd_;

    // Recording services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_recording_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_recording_service_;

    // Rosbag writers
    rosbag2_cpp::Writer actions_writer_;
    rosbag2_cpp::Writer obs_writer_;
    rosbag2_storage::StorageOptions actions_storage_options_;
    rosbag2_storage::StorageOptions obs_storage_options_;
    rosbag2_cpp::ConverterOptions converter_options_;

    // Video writers
    cv::VideoWriter base_video_writer_;
    cv::VideoWriter arm_video_writer_;
    bool base_video_initialized_ = false;
    bool arm_video_initialized_ = false;
    std::string base_video_filename_;
    std::string arm_video_filename_;

    // Synchronized recording timer
    rclcpp::TimerBase::SharedPtr record_timer_;
    
    // Parameters
    std::string storage_uri_;
    bool use_sim_;
    bool recording_enabled_ = false;
    double fps_;
    rclcpp::Time last_record_time_;
    
    // Sample counter for alignment
    uint64_t sample_count_ = 0;

public:
    SynchronizedRecorder()
        : Node("synchronized_recorder"),
          recording_enabled_(false),
          sample_count_(0)
    {
        // Declare parameters
        declare_parameter<std::string>("storage_uri", "episode_bag");
        declare_parameter<bool>("use_sim", true);
        declare_parameter<double>("fps", 10.0);
        get_parameter("storage_uri", storage_uri_);
        get_parameter("use_sim", use_sim_);
        get_parameter("fps", fps_);

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

        // Create synchronized recording timer
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps_));
        record_timer_ = create_wall_timer(period, std::bind(&SynchronizedRecorder::synchronized_record_callback, this));

        RCLCPP_INFO(this->get_logger(), "Episode Recorder Node Initialized");
        RCLCPP_INFO(this->get_logger(), "Storage URI: %s", storage_uri_.c_str());
        RCLCPP_INFO(this->get_logger(), "Recording frequency: %.1f Hz", fps_);
    }

private:
    void setup_observation_subscriptions()
    {
        // Joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                last_joint_state_ = msg;
            });

        // Camera images
        base_image_sub_ = image_transport::create_subscription(
            this, "/tidybot/camera_base/color/raw",
            [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
                last_base_image_ = msg;
            }, "raw");
        arm_image_sub_ = image_transport::create_subscription(
            this, "/tidybot/camera_wrist/color/raw",
            [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
                last_arm_image_ = msg;
            }, "raw");
    }

    void setup_action_subscriptions()
    {
        if (use_sim_)
        {
            // Simulation topics
            base_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/tidybot_base_pos_controller/commands", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    last_base_cmd_ = msg;
                });
            arm_cmd_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "/gen3_7dof_controller/joint_trajectory", 10,
                [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
                    last_arm_cmd_ = msg;
                });
            gripper_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/robotiq_2f_85_controller/commands", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    last_gripper_cmd_ = msg;
                });
        }
        else
        {
            // Real robot topics
            base_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/tidybot/base/commands", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    last_base_cmd_ = msg;
                });
            arm_cmd_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "/tidybot/arm/pose", 10,
                [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
                    last_arm_cmd_ = msg;
                });
            gripper_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/tidybot/gripper/state", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    last_gripper_cmd_ = msg;
                });
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
            RCLCPP_WARN(this->get_logger(), "Already recording episode");
            return;
        }

        recording_enabled_ = true;
        sample_count_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Starting synchronized episode recording");
        
        // Create episode directory
        std::string timestamp = get_timestamped_filename("episode");
        std::string episode_dir = storage_uri_ + "/" + timestamp;
        
        // Create the episode directory
        std::string mkdir_cmd = "mkdir -p " + episode_dir;
        int result = system(mkdir_cmd.c_str());
        if (result != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create episode directory: %s", episode_dir.c_str());
            recording_enabled_ = false;
            return;
        }
        
        // Setup actions bag inside episode directory
        actions_storage_options_.uri = episode_dir + "/actions";
        actions_writer_.open(actions_storage_options_, converter_options_);
        
        // Setup observations bag inside episode directory
        obs_storage_options_.uri = episode_dir + "/observations";
        obs_writer_.open(obs_storage_options_, converter_options_);
        
        // Setup video files inside episode directory
        base_video_filename_ = episode_dir + "/base_camera.mp4";
        arm_video_filename_ = episode_dir + "/arm_camera.mp4";
        
        RCLCPP_INFO(this->get_logger(), "Recording episode to: %s", episode_dir.c_str());
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

        RCLCPP_INFO(this->get_logger(), "Stopping synchronized episode recording");
        
        // Close writers
        actions_writer_.close();
        obs_writer_.close();
        
        // Close video writers
        if (base_video_writer_.isOpened())
        {
            base_video_writer_.release();
            RCLCPP_INFO(this->get_logger(), "Base video saved to %s", base_video_filename_.c_str());
        }
        if (arm_video_writer_.isOpened())
        {
            arm_video_writer_.release();
            RCLCPP_INFO(this->get_logger(), "Arm video saved to %s", arm_video_filename_.c_str());
        }

        recording_enabled_ = false;
        base_video_initialized_ = false;
        arm_video_initialized_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Recorded %lu synchronized samples", sample_count_);
        
        // Extract episode directory from actions path for cleaner logging
        std::string episode_dir = actions_storage_options_.uri;
        size_t last_slash = episode_dir.find_last_of('/');
        if (last_slash != std::string::npos) {
            episode_dir = episode_dir.substr(0, last_slash);
        }
        
        RCLCPP_INFO(this->get_logger(), "Episode data saved to: %s", episode_dir.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Actions: %s", actions_storage_options_.uri.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Observations: %s", obs_storage_options_.uri.c_str());
    }

    void synchronized_record_callback()
    {
        // Handle time jumps
        if (last_record_time_.nanoseconds() > this->get_clock()->now().nanoseconds())
        {
            RCLCPP_WARN(this->get_logger(), "Time jump detected, resetting tf_buffer...");
            tf_buffer_.reset();
            tf_listener_.reset();
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
            return;
        }
        
        last_record_time_ = this->get_clock()->now();
        
        if (!recording_enabled_)
        {
            return;
        }

        // Record current observations (state at time T)
        record_observations();
        
        // Record current action commands (actions for time T -> T+1)
        record_actions();
        
        sample_count_++;
        
        if (sample_count_ % 100 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Recorded %lu synchronized samples", sample_count_);
        }
    }

    void record_observations()
    {
        auto current_time = this->get_clock()->now();
        
        // Record base pose
        try
        {
            auto transform_stamped = tf_buffer_->lookupTransform("world", "base", rclcpp::Time(0));
            geometry_msgs::msg::PoseStamped base_pose;
            base_pose.header = transform_stamped.header;
            base_pose.header.stamp = current_time;
            base_pose.pose.position.x = transform_stamped.transform.translation.x;
            base_pose.pose.position.y = transform_stamped.transform.translation.y;
            base_pose.pose.position.z = transform_stamped.transform.translation.z;
            base_pose.pose.orientation = transform_stamped.transform.rotation;
            
            write_observation_message<geometry_msgs::msg::PoseStamped>(
                std::make_shared<geometry_msgs::msg::PoseStamped>(base_pose), "/base_pose");
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                  "Could not get base pose: %s", ex.what());
        }

        // Record arm pose
        try
        {
            auto transform_stamped = tf_buffer_->lookupTransform("arm_base_link", "bracelet_link", rclcpp::Time(0));
            geometry_msgs::msg::PoseStamped arm_pose;
            arm_pose.header = transform_stamped.header;
            arm_pose.header.stamp = current_time;
            arm_pose.pose.position.x = transform_stamped.transform.translation.x;
            arm_pose.pose.position.y = transform_stamped.transform.translation.y;
            arm_pose.pose.position.z = transform_stamped.transform.translation.z;
            arm_pose.pose.orientation = transform_stamped.transform.rotation;
            
            write_observation_message<geometry_msgs::msg::PoseStamped>(
                std::make_shared<geometry_msgs::msg::PoseStamped>(arm_pose), "/arm_pose");
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Could not get arm pose: %s", ex.what());
        }

        // Record gripper state
        if (last_joint_state_)
        {
            auto it = std::find(last_joint_state_->name.begin(), last_joint_state_->name.end(), "left_outer_knuckle_joint");
            if (it != last_joint_state_->name.end())
            {
                size_t index = std::distance(last_joint_state_->name.begin(), it);
                if (index < last_joint_state_->position.size())
                {
                    std_msgs::msg::Float64 gripper_state;
                    gripper_state.data = last_joint_state_->position[index];
                    write_observation_message<std_msgs::msg::Float64>(
                        std::make_shared<std_msgs::msg::Float64>(gripper_state), "/gripper_state");
                }
            }
        }

        // Record camera images as video frames
        record_camera_images();
    }

    void record_actions()
    {
        auto current_time = this->get_clock()->now();
        
        // Record base command
        if (last_base_cmd_)
        {
            auto cmd_copy = std::make_shared<std_msgs::msg::Float64MultiArray>(*last_base_cmd_);
            write_action_message<std_msgs::msg::Float64MultiArray>(cmd_copy, "/tidybot_base_pos_controller/commands");
        }

        // Record arm command
        if (last_arm_cmd_)
        {
            auto cmd_copy = std::make_shared<trajectory_msgs::msg::JointTrajectory>(*last_arm_cmd_);
            std::string topic = use_sim_ ? "/gen3_7dof_controller/joint_trajectory" : "/tidybot/arm/command";
            write_action_message<trajectory_msgs::msg::JointTrajectory>(cmd_copy, topic);
        }

        // Record gripper command
        if (last_gripper_cmd_)
        {
            auto cmd_copy = std::make_shared<std_msgs::msg::Float64MultiArray>(*last_gripper_cmd_);
            std::string topic = use_sim_ ? "/robotiq_2f_85_controller/commands" : "/tidybot/gripper/command";
            write_action_message<std_msgs::msg::Float64MultiArray>(cmd_copy, topic);
        }
    }

    void record_camera_images()
    {
        // Record base camera
        if (last_base_image_)
        {
            try
            {
                auto cv_ptr = cv_bridge::toCvCopy(last_base_image_, sensor_msgs::image_encodings::BGR8);
                
                if (!base_video_initialized_)
                {
                    cv::Size frame_size(cv_ptr->image.cols, cv_ptr->image.rows);
                    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
                    base_video_writer_.open(base_video_filename_, fourcc, fps_, frame_size, true);
                    if (base_video_writer_.isOpened())
                    {
                        base_video_initialized_ = true;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to open base video writer");
                    }
                }
                
                if (base_video_initialized_)
                {
                    base_video_writer_ << cv_ptr->image;
                }
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Base camera cv_bridge exception: %s", e.what());
            }
        }

        // Record arm camera
        if (last_arm_image_)
        {
            try
            {
                auto cv_ptr = cv_bridge::toCvCopy(last_arm_image_, sensor_msgs::image_encodings::BGR8);
                
                if (!arm_video_initialized_)
                {
                    cv::Size frame_size(cv_ptr->image.cols, cv_ptr->image.rows);
                    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
                    arm_video_writer_.open(arm_video_filename_, fourcc, fps_, frame_size, true);
                    if (arm_video_writer_.isOpened())
                    {
                        arm_video_initialized_ = true;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to open arm video writer");
                    }
                }
                
                if (arm_video_initialized_)
                {
                    arm_video_writer_ << cv_ptr->image;
                }
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Arm camera cv_bridge exception: %s", e.what());
            }
        }
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
