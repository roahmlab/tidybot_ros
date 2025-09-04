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

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_recording_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_recording_service_;

    rosbag2_cpp::Writer actions_writer;
    rosbag2_storage::StorageOptions storage_options_;
    rosbag2_cpp::ConverterOptions converter_options_;

    std::string storage_uri_;
    bool use_sim_;
    bool recording_enabled_ = false;

public:
    EpisodeRecorder()
        : Node("episode_recorder"),
          recording_enabled_(false)
    {
        declare_parameter<std::string>("storage_uri", "episode_bag");
        declare_parameter<bool>("use_sim", true);
        get_parameter("storage_uri", storage_uri_);
        get_parameter("use_sim", use_sim_);
        // Initialize storage options
        storage_options_.storage_id = "sqlite3";
        converter_options_.input_serialization_format = "cdr";
        converter_options_.output_serialization_format = "cdr";

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
        storage_options_.uri = storage_uri_ + "/" + get_timestamped_filename("actions");
        actions_writer.open(storage_options_, converter_options_);
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

        // Serialise into a heap‑allocated buffer
        auto serialized = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<MsgT> serializer;
        serializer.serialize_message(msg.get(), serialized.get());

        // Resolve the fully qualified ROS type string once
        const std::string type_name = rosidl_generator_traits::name<MsgT>(); // e.g. "sensor_msgs/msg/JointState"

        // Use the new overload
        actions_writer.write(
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
    auto node = std::make_shared<EpisodeRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}