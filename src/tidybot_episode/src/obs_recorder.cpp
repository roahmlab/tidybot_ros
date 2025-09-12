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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class ObsRecorder : public rclcpp::Node
{
private:
    // joint state listener for recording the gripper state
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState::ConstSharedPtr last_joint_state_;
    // tf_listener for recording the arm pose and base pose
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // subscriptions for base and arm camera images
    image_transport::Subscriber base_image_sub_;
    image_transport::Subscriber arm_image_sub_;

    // start and stop recording services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_recording_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_recording_service_;

    // subscriptions for base and arm commands
    rosbag2_cpp::Writer obs_writer_;
    rosbag2_storage::StorageOptions storage_options_;
    rosbag2_cpp::ConverterOptions converter_options_;

    cv::VideoWriter base_video_writer_;
    bool base_video_initialized_ = false;
    cv::VideoWriter arm_video_writer_;
    bool arm_video_initialized_ = false;

    rclcpp::TimerBase::SharedPtr record_timer_;

    std::string base_video_filename_;
    sensor_msgs::msg::Image::ConstSharedPtr last_base_image_;
    std::string arm_video_filename_;
    sensor_msgs::msg::Image::ConstSharedPtr last_arm_image_;

    std::string storage_uri_;
    bool use_sim_;
    bool recording_enabled_ = false;
    double fps_;
    rclcpp::Time last_record_time_;

public:
    ObsRecorder()
        : Node("aligned_episode_recorder"),
          base_video_initialized_(false),
          arm_video_initialized_(false),
          recording_enabled_(false)
    {
        declare_parameter<std::string>("storage_uri", "episode_bag");
        declare_parameter<bool>("use_sim", false);
        declare_parameter<double>("fps", 10.0);
        get_parameter("storage_uri", storage_uri_);
        get_parameter("use_sim", use_sim_);
        get_parameter("fps", fps_);

        // Initialize storage options
        storage_options_.storage_id = "sqlite3";
        converter_options_.input_serialization_format = "cdr";
        converter_options_.output_serialization_format = "cdr";

        // Create subscription to joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                last_joint_state_ = msg;
            });

        // Create subscriptions for base and arm commands
        base_image_sub_ = image_transport::create_subscription(
            this, "/base_camera/image",
            [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
                last_base_image_ = msg;
            }, "raw");
        arm_image_sub_ = image_transport::create_subscription(
            this, "/arm_camera/image",
            [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
                last_arm_image_ = msg;
            }, "raw");

        // Create services for starting and stopping recording
        start_recording_service_ = this->create_service<std_srvs::srv::Empty>(
            "/obs/start_recording",
            std::bind(&ObsRecorder::start_recording_callback, this, std::placeholders::_1, std::placeholders::_2));
        stop_recording_service_ = this->create_service<std_srvs::srv::Empty>(
            "/obs/stop_recording",
            std::bind(&ObsRecorder::stop_recording_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create a timer to periodically record data
        record_timer_ = create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ObsRecorder::record_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Episode Recorder Node Initialized");
        RCLCPP_INFO(this->get_logger(), "Storage URI: %s", storage_uri_.c_str());
    }

    void start_recording_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        // prevent unused parameter warning
        (void)request;
        (void)response;
        if (recording_enabled_)
        {
            RCLCPP_WARN(this->get_logger(), "Already recording episode");
            return;
        }
        recording_enabled_ = true;
        RCLCPP_INFO(this->get_logger(), "Starting episode recording");
        // Create a new bag file for the episode
        storage_options_.uri = storage_uri_ + "/" + get_timestamped_filename("observations");
        obs_writer_.open(storage_options_, converter_options_);
        base_video_filename_ = storage_options_.uri + "/" + get_timestamped_filename("base_video") + ".mp4";
        arm_video_filename_ = storage_options_.uri + "/" + get_timestamped_filename( "arm_video") + ".mp4";
        return;
    }

    void stop_recording_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        // prevent unused parameter warning
        (void)request;
        (void)response;
        if (!recording_enabled_)
        {
            RCLCPP_WARN(this->get_logger(), "No recording in progress to stop.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Stopping episode recording");
        obs_writer_.close();
        RCLCPP_INFO(this->get_logger(), "Actions bag file saved to: %s", storage_options_.uri.c_str());

        recording_enabled_ = false;
        base_video_initialized_ = false;
        arm_video_initialized_ = false;
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
        return;
    }

    void record_timer_callback()
    {
        // check for time jump back
        if (last_record_time_.nanoseconds() > this->get_clock()->now().nanoseconds())
        {
            RCLCPP_WARN(this->get_logger(), "Time jump detected, resetting tf_buffer and tf_listener...");
            tf_buffer_.reset();
            tf_listener_.reset();

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
            return;
        }
        if (!recording_enabled_)
        {
            return;
        }
        // store the image messaged as mp4 video frames
        // base image:
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(last_base_image_, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (!base_video_initialized_)
        {
            cv::Size frame_size(cv_ptr->image.cols, cv_ptr->image.rows);
            // FOURCC code for MP4 ("mp4v" is generally supported)
            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            base_video_writer_.open(base_video_filename_, fourcc, fps_, frame_size, true);
            if (!base_video_writer_.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open Base VideoWriter");
                rclcpp::shutdown();
                return;
            }
            base_video_initialized_ = true;
        }
        base_video_writer_ << cv_ptr->image;
        // arm image:
        try
        {
            cv_ptr = cv_bridge::toCvCopy(last_arm_image_, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Initialize video writer if not already done
        if (!arm_video_initialized_)
        {
            cv::Size frame_size(cv_ptr->image.cols, cv_ptr->image.rows);
            // FOURCC code for MP4 ("mp4v" is generally supported)
            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            arm_video_writer_.open(arm_video_filename_, fourcc, fps_, frame_size, true);
            if (!arm_video_writer_.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open Arm VideoWriter");
                rclcpp::shutdown();
                return;
            }
            arm_video_initialized_ = true;
        }
        arm_video_writer_ << cv_ptr->image;

        // get the transform from world to base and convert it to a geometry_msgs::msg::PoseStamped
        geometry_msgs::msg::PoseStamped base_pose_stamped;
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform("world", "base", rclcpp::Time(0));
            base_pose_stamped.header = transform_stamped.header;
            base_pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
            base_pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
            base_pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
            base_pose_stamped.pose.orientation = transform_stamped.transform.rotation;
            // write to rosbag
            write_message<geometry_msgs::msg::PoseStamped>(std::make_shared<geometry_msgs::msg::PoseStamped>(base_pose_stamped), "/base_pose");
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform from world to base: %s", ex.what());
            return;
        }   

        // get the transform from arm_base_link to end_effector_link and convert it to a geometry_msgs::msg::PoseStamped
        geometry_msgs::msg::PoseStamped arm_pose_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform("arm_base_link", "end_effector_link", rclcpp::Time(0));
            arm_pose_stamped.header = transform_stamped.header;
            arm_pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
            arm_pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
            arm_pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
            arm_pose_stamped.pose.orientation = transform_stamped.transform.rotation;
            // write to rosbag
            write_message<geometry_msgs::msg::PoseStamped>(std::make_shared<geometry_msgs::msg::PoseStamped>(arm_pose_stamped), "/arm_pose");
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform from arm_base_link to end_effector_link: %s", ex.what());
            return;
        }

        // extract the gripper state from the last joint state message
        if (last_joint_state_)
        {
            std_msgs::msg::Float64 gripper_state;
            auto it = std::find(last_joint_state_->name.begin(), last_joint_state_->name.end(), "left_outer_knuckle_joint");
            if (it != last_joint_state_->name.end())
            {                size_t index = std::distance(last_joint_state_->name.begin(), it);
                if (index < last_joint_state_->position.size())
                {
                    gripper_state.data = last_joint_state_->position[index];
                    // write to rosbag
                    write_message<std_msgs::msg::Float64>(std::make_shared<std_msgs::msg::Float64>(gripper_state), "/gripper_state");
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Gripper joint position index out of bounds");
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Gripper joint not found in joint states");
            }
        }
    }

private:
    template <typename MsgT>
    void write_message(const typename MsgT::SharedPtr &msg,
                       const std::string &topic_name)
    {
    if (!recording_enabled_)
        {
            return;
        }

        // 1) Serialise into a heap‑allocated buffer
        auto serialized = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<MsgT> serializer;
        serializer.serialize_message(msg.get(), serialized.get());

        // 2) Resolve the fully qualified ROS type string once
        const std::string type_name = rosidl_generator_traits::name<MsgT>(); // e.g. "sensor_msgs/msg/JointState"

        // 3) Use the new overload
        obs_writer_.write(
            serialized, // shared_ptr<const SerializedMessage>
            topic_name,
            type_name,
            this->get_clock()->now()); // rclcpp::Time
    }

    std::string get_timestamped_filename(const std::string &prefix)
    {
    // Grab current system time_point
    auto now = std::chrono::system_clock::now();  
    // std::chrono::system_clock::now() returns the current wall‑clock time_point :contentReference[oaicite:0]{index=0}

    // Convert to time_t for calendar operations
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    // system_clock::to_time_t converts a time_point to time_t :contentReference[oaicite:1]{index=1}

    // Break out into local calendar time (thread‑safe)
    std::tm local_tm;
#if defined(_MSC_VER)
    localtime_s(&local_tm, &t);
#else
    localtime_r(&t, &local_tm);
#endif

    // Format as YYYY_MM_DD_HH_MM_SS
    std::ostringstream oss;
    oss << prefix << "_"
        << std::put_time(&local_tm, "%Y_%m_%d_%H_%M_%S");
    // std::put_time with local_tm and format specifier to output timestamp :contentReference[oaicite:2]{index=2}

    return oss.str();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObsRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}