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

class EpisodeRecorder : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr base_cmd_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_cmd_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_cmd_sub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_recording_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_recording_service_;

    rosbag2_cpp::Writer episode_writer_;
    rosbag2_storage::StorageOptions storage_options_;
    rosbag2_cpp::ConverterOptions converter_options_;

    std::string storage_uri_;
    bool recording_enabled_ = false;

public:
    EpisodeRecorder()
        : Node("episode_recorder"),
          recording_enabled_(false)
    {
        declare_parameter<std::string>("storage_uri", "episode_bag");
        get_parameter("storage_uri", storage_uri_);

        // Initialize storage options
        storage_options_.storage_id = "sqlite3";
        converter_options_.input_serialization_format = "cdr";
        converter_options_.output_serialization_format = "cdr";
        // Create subscription to joint states
        // joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        //     "/joint_states", 10,
        //     std::bind(&EpisodeRecorder::joint_state_callback, this, std::placeholders::_1));

        // Create subscriptions for base and arm commands
        base_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tidybot_base_pos_controller/commands", 10,
            std::bind(&EpisodeRecorder::base_cmd_callback, this, std::placeholders::_1));
        arm_cmd_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/gen3_7dof_controller/joint_trajectory", 10,
            std::bind(&EpisodeRecorder::arm_cmd_callback, this, std::placeholders::_1));
        gripper_cmd_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/gen3_lite_2f_controller/joint_trajectory", 10,
            std::bind(&EpisodeRecorder::gripper_cmd_callback, this, std::placeholders::_1));

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
        write_message<trajectory_msgs::msg::JointTrajectory>(msg, "/gen3_7dof_controller/joint_trajectory");
    }

    void gripper_cmd_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        // Write the gripper command message to the rosbag
        write_message<trajectory_msgs::msg::JointTrajectory>(msg, "/gen3_lite_2f_controller/joint_trajectory");
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
        storage_options_.uri = storage_uri_ + "/" + get_timestamped_filename(this->get_clock()->now());
        episode_writer_.open(storage_options_, converter_options_);
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
        episode_writer_.close();
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

        // 1) Serialise into a heap‑allocated buffer
        auto serialized = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<MsgT> serializer;
        serializer.serialize_message(msg.get(), serialized.get());

        // 2) Resolve the fully qualified ROS type string once
        const std::string type_name = rosidl_generator_traits::name<MsgT>(); // e.g. "sensor_msgs/msg/JointState"

        // 3) Use the new overload
        episode_writer_.write(
            serialized, // shared_ptr<const SerializedMessage>
            topic_name,
            type_name,
            this->get_clock()->now()); // rclcpp::Time
    }

    std::string get_timestamped_filename(const rclcpp::Time &time)
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
        oss << "episode" << "_" << year << "_" << month << "_"
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