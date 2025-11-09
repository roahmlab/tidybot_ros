#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/generic_publisher.hpp"

#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "opencv2/opencv.hpp"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <Eigen/Dense>

namespace fs = std::filesystem;

class ObsReader : public rclcpp::Node
{
public:
    ObsReader()
        : Node("obs_reader")
    {
        this->declare_parameter<std::string>("input_dir", "episode_bag");
        this->declare_parameter<std::string>("output_dir", "data.parquet");
        this->get_parameter("input_dir", root_dir_);
        this->get_parameter("output_dir", output_dir_);
        fs::create_directories(output_dir_);
        process_bags();
    }

private:

    std::string root_dir_, output_dir_;
    struct Row {
        double timestamp;
        int frame_index;
        int index;
        int task_index;
        std::vector<float> obs_state;
        std::vector<float> action;
    };

    static std::shared_ptr<arrow::Table> make_table(const std::vector<Row>& rows)
    {
        arrow::DoubleBuilder ts_builder;
        arrow::Int32Builder frame_builder;
        arrow::Int32Builder episode_builder;
        arrow::Int32Builder task_builder;

        // Create new FloatBuilders directly inside the ListBuilders
        auto obs_value_builder = std::make_shared<arrow::FloatBuilder>();
        arrow::ListBuilder obs_builder(arrow::default_memory_pool(), obs_value_builder);

        auto act_value_builder = std::make_shared<arrow::FloatBuilder>();
        arrow::ListBuilder act_builder(arrow::default_memory_pool(), act_value_builder);

        for (const auto& row : rows) {
            CheckArrowStatus(ts_builder.Append(row.timestamp));
            CheckArrowStatus(frame_builder.Append(row.frame_index));
            CheckArrowStatus(episode_builder.Append(row.index));
            CheckArrowStatus(task_builder.Append(row.task_index));

            // Append observation state
            CheckArrowStatus(obs_builder.Append());  // start new list
            CheckArrowStatus(obs_value_builder->AppendValues(row.obs_state));

            // Append action
            CheckArrowStatus(act_builder.Append());  // start new list
            CheckArrowStatus(act_value_builder->AppendValues(row.action));
        }

        std::shared_ptr<arrow::Array> ts_array;
        std::shared_ptr<arrow::Array> frame_array;
        std::shared_ptr<arrow::Array> episode_array;
        std::shared_ptr<arrow::Array> task_array;
        std::shared_ptr<arrow::Array> obs_array;
        std::shared_ptr<arrow::Array> act_array;

        CheckArrowStatus(ts_builder.Finish(&ts_array));
        CheckArrowStatus(frame_builder.Finish(&frame_array));
        CheckArrowStatus(episode_builder.Finish(&episode_array));
        CheckArrowStatus(task_builder.Finish(&task_array));
        CheckArrowStatus(obs_builder.Finish(&obs_array));
        CheckArrowStatus(act_builder.Finish(&act_array));

        auto schema = arrow::schema({
            arrow::field("timestamp", arrow::float64()),
            arrow::field("frame_index", arrow::int32()),
            arrow::field("index", arrow::int32()),
            arrow::field("task_index", arrow::int32()),
            arrow::field("observation.state", arrow::list(arrow::float32())),
            arrow::field("action", arrow::list(arrow::float32()))
        });

        return arrow::Table::Make(schema, {
            ts_array,
            frame_array,
            episode_array,
            task_array,
            obs_array,
            act_array
        });
    }

    void process_bags()
    {
        int idx = 0;
        const int task_idx = 0; 
        for (auto &entry : fs::directory_iterator(root_dir_)) {
            if (!entry.is_directory()) continue;
            fs::path episode_dir = entry.path();

            fs::path obs_bag = episode_dir / "observations" / "observations_0.db3";
            fs::path act_bag = episode_dir / "actions" / "actions_0.db3";
            if (!fs::exists(obs_bag) || !fs::exists(act_bag)) continue;

            RCLCPP_INFO(this->get_logger(), "Processing %s", episode_dir.string().c_str());
            std::vector<Row> rows = convert_rosbag_to_rows(obs_bag.string(), act_bag.string(), idx, task_idx);

            auto table = make_table(rows);
            fs::path out_path = fs::path(output_dir_) / (episode_dir.filename().string() + ".parquet");
            write_parquet(out_path.string(), table);
            idx++;
        }
    }

    std::vector<Row> convert_rosbag_to_rows(
        const std::string &obs_path, 
        const std::string &act_path,
        int episode_index, 
        int task_index = 0 
    )
    {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = obs_path.substr(0, obs_path.find_last_of("/"));
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options{"", ""};
        rosbag2_cpp::readers::SequentialReader obs_reader;
        obs_reader.open(storage_options, converter_options);

        rclcpp::Serialization<geometry_msgs::msg::PoseStamped> pose_stamped_ser;
        rclcpp::Serialization<std_msgs::msg::Float64> float_ser;

        std::vector<std::array<float, 8>> obs_states; // arm pose + gripper
        std::vector<double> timestamps;

        std::vector<geometry_msgs::msg::Pose> arm_poses;
        std::vector<float> gripper_states;

        while (obs_reader.has_next()) {
            auto msg = obs_reader.read_next();
            const auto &topic = msg->topic_name;
            rclcpp::SerializedMessage serialized(*msg->serialized_data);

            if (topic == "/arm_pose") {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_stamped_ser.deserialize_message(&serialized, &pose_msg);
                arm_poses.push_back(pose_msg.pose);
                timestamps.push_back(
                    static_cast<double>(pose_msg.header.stamp.sec) + 
                    static_cast<double>(pose_msg.header.stamp.nanosec) * 1e-9
                );
            } else if (topic == "/gripper_state") {
                std_msgs::msg::Float64 g_msg;
                float_ser.deserialize_message(&serialized, &g_msg);
                gripper_states.push_back(static_cast<float>(g_msg.data));
            }
        }
        
        // Normalize timestamps to start at 0
        double t0 = timestamps.front();
        for (auto &t : timestamps) {
            t -= t0;
        }

        if (arm_poses.size() != gripper_states.size()) {
            RCLCPP_WARN(rclcpp::get_logger("obs_reader"),
                        "Mismatch obs message counts: arm_pose=%zu gripper=%zu",
                        arm_poses.size(), gripper_states.size());
        }

        // Create observation state vector (xyz + quat + gripper)
        for (size_t i = 0; i < std::min(arm_poses.size(), gripper_states.size()); ++i) {
            const auto &p = arm_poses[i];
            obs_states.push_back({
                static_cast<float>(p.position.x), 
                static_cast<float>(p.position.y), 
                static_cast<float>(p.position.z),
                static_cast<float>(p.orientation.w), 
                static_cast<float>(p.orientation.x), 
                static_cast<float>(p.orientation.y), 
                static_cast<float>(p.orientation.z),
                static_cast<float>(gripper_states[i])
            });
        }

        // Read Actions
        rosbag2_storage::StorageOptions act_storage;
        act_storage.uri = act_path.substr(0, act_path.find_last_of("/"));
        act_storage.storage_id = "sqlite3";

        rosbag2_cpp::readers::SequentialReader act_reader;
        act_reader.open(act_storage, converter_options);

        rclcpp::Serialization<geometry_msgs::msg::Pose> pose_ser;

        std::vector<geometry_msgs::msg::Pose> target_poses;
        std::vector<float> gripper_cmds;

        while (act_reader.has_next()) {
            auto msg = act_reader.read_next();
            const auto &topic = msg->topic_name;
            rclcpp::SerializedMessage serialized(*msg->serialized_data);

            if (topic == "/tidybot/arm/target_pose") {
                geometry_msgs::msg::Pose pose_msg;
                pose_ser.deserialize_message(&serialized, &pose_msg);
                target_poses.push_back(pose_msg);
            } else if (topic == "/tidybot/gripper/commands") {
                std_msgs::msg::Float64 cmd_msg;
                float_ser.deserialize_message(&serialized, &cmd_msg);
                gripper_cmds.push_back(static_cast<float>(cmd_msg.data));
            }
        }

        if (target_poses.size() != gripper_cmds.size()) {
            RCLCPP_WARN(rclcpp::get_logger("obs_reader"),
                        "Mismatch action counts: poses=%zu gripper_cmds=%zu",
                        target_poses.size(), gripper_cmds.size());
        }

        // Compute ΔXYZ, ΔRPY, gripper
        std::vector<Row> rows;
        const size_t n = std::min({obs_states.size(), target_poses.size(), gripper_cmds.size()});

        Eigen::Vector3f prev_xyz(0, 0, 0), prev_rpy(0, 0, 0);
        bool first = true;

        for (size_t i = 0; i < n; ++i) {
            const auto &pose = target_poses[i];
            tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            Eigen::Vector3f xyz(pose.position.x, pose.position.y, pose.position.z);
            Eigen::Vector3f rpy(roll, pitch, yaw);

            Eigen::Vector3f dpos, drpy;
            if (first) {
                dpos = Eigen::Vector3f::Zero();
                drpy = Eigen::Vector3f::Zero();
                first = false;
            } else {
                dpos = xyz - prev_xyz;
                drpy = rpy - prev_rpy;
            }

            prev_xyz = xyz;
            prev_rpy = rpy;

            Row r;
            r.timestamp = timestamps[i];
            r.frame_index = static_cast<int>(i);
            r.index = episode_index;
            r.task_index = task_index;

            // Observation: joint_state + gripper
            r.obs_state.assign(obs_states[i].begin(), obs_states[i].end());

            // Action: dpos(3) + drpy(3) + gripper_cmd(1)
            r.action = {dpos.x(), dpos.y(), dpos.z(),
                        drpy.x(), drpy.y(), drpy.z(),
                        gripper_cmds[i]};

            rows.push_back(std::move(r));
        }

        RCLCPP_INFO(rclcpp::get_logger("obs_reader"), "Converted %zu messages to rows", rows.size());
        return rows;
    }

    void write_parquet(const std::string &path, std::shared_ptr<arrow::Table> table)
    {
        std::shared_ptr<arrow::io::OutputStream> outfile;
        PARQUET_ASSIGN_OR_THROW(outfile, arrow::io::FileOutputStream::Open(path));
        PARQUET_THROW_NOT_OK(
            parquet::arrow::WriteTable(
                *table,
                arrow::default_memory_pool(),
                outfile,
                1024
            )
        );
        RCLCPP_INFO(this->get_logger(), "Wrote Parquet: %s", path.c_str());
    }

    static void CheckArrowStatus(const arrow::Status& st) {
        if (!st.ok()) {
            throw std::runtime_error(st.ToString());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObsReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
