#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <memory>
#include <ctime>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class EpisodeRecorder : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr base_cmd_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_cmd_sub_;
    image_transport::Subscriber base_image_sub_;
    image_transport::Subscriber arm_image_sub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_recording_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_recording_service_;

    rosbag2_cpp::Writer actions_writer;
    rosbag2_storage::StorageOptions storage_options_;
    rosbag2_cpp::ConverterOptions converter_options_;

    cv::VideoWriter base_video_writer_;
    bool base_video_initialized_ = false;
    cv::VideoWriter arm_video_writer_;
    bool arm_video_initialized_ = false;

    std::string base_video_filename_;
    std::string arm_video_filename_;

    std::string storage_uri_;
    bool use_sim_;
    bool recording_enabled_ = false;
    double fps_;

public:
    EpisodeRecorder()
        : Node("episode_recorder"),
          base_video_initialized_(false),
          arm_video_initialized_(false),
          recording_enabled_(false)
    {
        declare_parameter<std::string>("storage_uri", "episode_bag");
        declare_parameter<bool>("use_sim", false);
        declare_parameter<double>("fps", 30.0);
        get_parameter("storage_uri", storage_uri_);
        get_parameter("use_sim", use_sim_);
        get_parameter("fps", fps_);

        // Initialize storage options
        storage_options_.storage_id = "sqlite3";
        converter_options_.input_serialization_format = "cdr";
        converter_options_.output_serialization_format = "cdr";
        // Create subscription to joint states
        // joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        //     "/joint_states", 10,
        //     std::bind(&EpisodeRecorder::joint_state_callback, this, std::placeholders::_1));

        // Create subscriptions for base and arm commands
        if (use_sim_)
        {
            // if recording in simulation, subscribe to the ros2_controller topics
            base_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/tidybot_base_pos_controller/commands", 10,
                std::bind(&EpisodeRecorder::base_cmd_callback, this, std::placeholders::_1));
            arm_cmd_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "/gen3_7dof_controller/joint_trajectory", 10,
                std::bind(&EpisodeRecorder::arm_cmd_callback, this, std::placeholders::_1));
            gripper_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/robotiq_2f_85_controller/commands", 10,
                std::bind(&EpisodeRecorder::gripper_cmd_callback, this, std::placeholders::_1));
            base_image_sub_ = image_transport::create_subscription(
                this, "/base_camera/image",
                std::bind(&EpisodeRecorder::base_image_callback, this, std::placeholders::_1), "raw");
            arm_image_sub_ = image_transport::create_subscription(
                this, "/arm_camera/image",
                std::bind(&EpisodeRecorder::arm_image_callback, this, std::placeholders::_1), "raw");
        }
        // If not in simulation, subscribe to the tidybot_control topics
        else
        {
            base_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/tidybot/base/commands", 10,
                std::bind(&EpisodeRecorder::base_cmd_callback, this, std::placeholders::_1));
            arm_cmd_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "/tidybot/arm/pose", 10,
                std::bind(&EpisodeRecorder::arm_cmd_callback, this, std::placeholders::_1));
            gripper_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/tidybot/gripper/state", 10,
                std::bind(&EpisodeRecorder::gripper_cmd_callback, this, std::placeholders::_1));
        }

        // Create services for starting and stopping recording
        start_recording_service_ = this->create_service<std_srvs::srv::Empty>(
            "start_recording",
            std::bind(&EpisodeRecorder::start_recording_callback, this, std::placeholders::_1, std::placeholders::_2));
        stop_recording_service_ = this->create_service<std_srvs::srv::Empty>(
            "stop_recording",
            std::bind(&EpisodeRecorder::stop_recording_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Episode Recorder Node Initialized");
        RCLCPP_INFO(this->get_logger(), "Storage URI: %s", storage_uri_.c_str());
    }

    // void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    // {
    //     // Write the joint state message to the rosbag
    //     write_message<sensor_msgs::msg::JointState>(msg, "/joint_states");
    //     RCLCPP_INFO(this->get_logger(), "Recorded joint state for episode_%d", episode_count_);
    // }

    void base_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Write the base command message to the rosbag
        write_message<std_msgs::msg::Float64MultiArray>(msg, "/tidybot_base_pos_controller/commands");
    }

    void arm_cmd_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        // Write the arm command message to the rosbag
        if (use_sim_)
        {
            write_message<trajectory_msgs::msg::JointTrajectory>(msg, "/gen3_7dof_controller/joint_trajectory");
        }
        else
        {
            write_message<trajectory_msgs::msg::JointTrajectory>(msg, "/tidybot/arm/command");
        }
    }

    void gripper_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Write the gripper command message to the rosbag
        if (use_sim_)
        {
            write_message<std_msgs::msg::Float64MultiArray>(msg, "/robotiq_2f_85_controller/commands");
        }
        else
        {
            write_message<std_msgs::msg::Float64MultiArray>(msg, "/tidybot/gripper/command");
        }
    }

    void base_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        if (!recording_enabled_)
        {
            return;
        }
        // Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Initialize video writer if not already done
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

        // Write the frame to the video file
        base_video_writer_ << cv_ptr->image;
    }

    void arm_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        if (!recording_enabled_)
        {
            return;
        }
        // Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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

        // Write the frame to the video file
        arm_video_writer_ << cv_ptr->image;
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
        storage_options_.uri = storage_uri_ + "/" + get_timestamped_filename(this->get_clock()->now(), "actions");
        actions_writer.open(storage_options_, converter_options_);
        base_video_filename_ = storage_options_.uri + "/" + get_timestamped_filename(this->get_clock()->now(), "base_video") + ".mp4";
        arm_video_filename_ = storage_options_.uri + "/" + get_timestamped_filename(this->get_clock()->now(), "arm_video") + ".mp4";
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
        actions_writer.close();
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
        actions_writer.write(
            serialized, // shared_ptr<const SerializedMessage>
            topic_name,
            type_name,
            this->get_clock()->now()); // rclcpp::Time
    }

    std::string get_timestamped_filename(const rclcpp::Time &time, const std::string &prefix)
    {
        auto epoch_sec = static_cast<std::time_t>(time.seconds());
        std::tm local_tm = *std::localtime(&epoch_sec);
        std::ostringstream oss;
        int year = local_tm.tm_year + 1900;
        int month = local_tm.tm_mon + 1;
        int day = local_tm.tm_mday;
        int hour = local_tm.tm_hour;
        int minute = local_tm.tm_min;
        int second = local_tm.tm_sec;
        oss << prefix << "_" << year << "_" << month << "_"
            << day << "_" << hour << "_" << minute << "_" << second;
        return oss.str();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EpisodeRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}