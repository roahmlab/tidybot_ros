#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/generic_publisher.hpp"

#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"

// Message Headers
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp" 

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <Eigen/Dense>

namespace fs = std::filesystem;

enum class StateEncoding {
    POS_QUAT,         // EEF XYZ (3) + Quaternion (4) + Gripper (1)
    JOINT,            // Joint Angles (7) + Gripper (1)
};

enum class ActionEncoding {
    EEF_DELTA,          // EEF Delta XYZ (3) + Delta RPY (3) + Gripper Cmd (1)
    JOINT_DELTA,        // Joint Delta Position (7) + Gripper Cmd (1)
};

const std::string TOPIC_OBS_EE_POSE     = "/arm_pose";
const std::string TOPIC_OBS_JOINT       = "/joint_states"; 
const std::string TOPIC_OBS_GRIPPER     = "/gripper_state"; 

const std::string TOPIC_ACT_EE_TARGET   = "/tidybot/arm/target_pose";
const std::string TOPIC_ACT_EE_GRIPPER  = "/tidybot/gripper/commands";

class ObsReader : public rclcpp::Node
{
public:
    ObsReader() : Node("obs_reader")
    {
        // Parameters
        this->declare_parameter<std::string>("input_dir", "episode_bag");
        this->declare_parameter<std::string>("output_dir", "data.parquet");
        this->declare_parameter<std::string>("state_encoding", "JOINT");     // Options: "POS_QUAT", "JOINT"
        this->declare_parameter<std::string>("action_encoding", "JOINT_DELTA"); // Options: "EEF_DELTA", "JOINT_DELTA"

        this->get_parameter("input_dir", root_dir_);
        this->get_parameter("output_dir", output_dir_);
        
        std::string state_enc_str, act_enc_str;
        this->get_parameter("state_encoding", state_enc_str);
        this->get_parameter("action_encoding", act_enc_str);

        // Resolve Enums
        state_encoding_ = resolve_state_encoding(state_enc_str);
        action_encoding_ = resolve_action_encoding(act_enc_str);

        fs::create_directories(output_dir_);
        process_bags();
    }

private:
    std::string root_dir_, output_dir_;
    StateEncoding state_encoding_;
    ActionEncoding action_encoding_;

    struct Row {
        double timestamp;
        int frame_index;
        int index;
        int task_index;
        std::vector<float> obs_state;
        std::vector<float> action;
    };

    StateEncoding resolve_state_encoding(const std::string& s) {
        if (s == "POS_QUAT") return StateEncoding::POS_QUAT;
        if (s == "JOINT") return StateEncoding::JOINT;
        RCLCPP_WARN(this->get_logger(), "Unknown state_encoding '%s', defaulting to POS_QUAT", s.c_str());
        return StateEncoding::POS_QUAT;
    }

    ActionEncoding resolve_action_encoding(const std::string& s) {
        if (s == "EEF_DELTA") return ActionEncoding::EEF_DELTA;
        if (s == "JOINT_DELTA") return ActionEncoding::JOINT_DELTA;
        RCLCPP_WARN(this->get_logger(), "Unknown action_encoding '%s', defaulting to EEF_DELTA", s.c_str());
        return ActionEncoding::EEF_DELTA;
    }

    // --- Processing Loop ---
    void process_bags()
    {
        int idx = 0;
        const int task_idx = 0; 
        for (auto &entry : fs::directory_iterator(root_dir_)) {
            if (!entry.is_directory()) continue;
            fs::path episode_dir = entry.path();

            fs::path obs_bag = episode_dir / "observations" / "observations_0.db3";
            fs::path act_bag = episode_dir / "actions" / "actions_0.db3";
            
            if (!fs::exists(obs_bag)) {
                RCLCPP_WARN(this->get_logger(), "Observation folder not found");
                continue;
            }
            if (!fs::exists(act_bag)) {
                RCLCPP_WARN(this->get_logger(), "Action folder not found");
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Processing %s", episode_dir.stem().c_str());
            std::vector<Row> rows = convert_rosbag_to_rows(obs_bag.string(), act_bag.string(), idx, task_idx);

            if (!rows.empty()) {
                auto table = make_table(rows);
                fs::path out_path = fs::path(output_dir_) / (episode_dir.stem().string() + ".parquet");
                write_parquet(out_path.string(), table);
            }
            idx++;
        }
    }

    std::vector<Row> convert_rosbag_to_rows(
        const std::string &obs_path, 
        const std::string &act_path,
        int episode_index, 
        int task_index
    )
    {
        // READ OBSERVATIONS
        auto [obs_vectors, timestamps] = read_observations(obs_path);

        // READ ACTIONS
        auto act_vectors = read_actions(act_path, obs_path);

        // MERGE
        std::vector<Row> rows;
        size_t n = std::min(obs_vectors.size(), act_vectors.size());

        for (size_t i = 0; i < n; ++i) {
            Row r;
            r.timestamp = timestamps[i];
            r.frame_index = static_cast<int>(i);
            r.index = episode_index;
            r.task_index = task_index;
            r.obs_state = obs_vectors[i];
            r.action = act_vectors[i];
            rows.push_back(std::move(r));
        }

        RCLCPP_INFO(this->get_logger(), "Generated %zu rows.", rows.size());
        return rows;
    }

    std::pair<std::vector<std::vector<float>>, std::vector<double>> 
    read_observations(const std::string& path)
    {
        std::vector<std::vector<float>> final_states;
        std::vector<double> timestamps;

        rosbag2_storage::StorageOptions store_opt; 
        store_opt.uri = path; 
        store_opt.storage_id = "sqlite3";
        
        rosbag2_cpp::readers::SequentialReader reader; 
        reader.open(store_opt, {"", ""}); // Converter options

        rclcpp::Serialization<geometry_msgs::msg::PoseStamped> ser_pose;
        rclcpp::Serialization<sensor_msgs::msg::JointState> ser_joint;
        rclcpp::Serialization<std_msgs::msg::Float64> ser_float;

        double t0 = -1.0;
        float latest_gripper = 0.0f;
        bool gripper_received = false; // Sync flag

        // Iterate through Bag
        while (reader.has_next()) {
            auto msg = reader.read_next();
            rclcpp::SerializedMessage ser_msg(*msg->serialized_data);

            if (msg->topic_name == TOPIC_OBS_GRIPPER) {
                std_msgs::msg::Float64 g;
                ser_float.deserialize_message(&ser_msg, &g);
                latest_gripper = (float)g.data;
                gripper_received = true;
                continue;
            }

            if (!gripper_received) {
                continue;
            }

            if (state_encoding_ == StateEncoding::POS_QUAT) {
                if (msg->topic_name == TOPIC_OBS_EE_POSE) {
                    geometry_msgs::msg::PoseStamped p;
                    ser_pose.deserialize_message(&ser_msg, &p);

                    double t = p.header.stamp.sec + p.header.stamp.nanosec * 1e-9;
                    if(t0 < 0) t0 = t;

                    std::vector<float> vec = {
                        (float)p.pose.position.x, (float)p.pose.position.y, (float)p.pose.position.z,
                        (float)p.pose.orientation.w, (float)p.pose.orientation.x, 
                        (float)p.pose.orientation.y, (float)p.pose.orientation.z
                    };
                    
                    // Append separate gripper state
                    vec.push_back(latest_gripper);

                    final_states.push_back(vec);
                    timestamps.push_back(t - t0);
                }
            }
            else if (state_encoding_ == StateEncoding::JOINT) {
                if (msg->topic_name == TOPIC_OBS_JOINT) {
                    sensor_msgs::msg::JointState js;
                    ser_joint.deserialize_message(&ser_msg, &js);

                    double t = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9;
                    if(t0 < 0) t0 = t;

                    std::vector<float> vec;
                    // Take first 7 joints (arm)
                    size_t count = std::min((size_t)7, js.position.size());
                    for(size_t i=0; i<count; ++i) vec.push_back((float)js.position[i]);
                    
                    // Padding if < 7
                    while(vec.size() < 7) vec.push_back(0.0f);

                    // Append separate gripper state
                    vec.push_back(latest_gripper);

                    final_states.push_back(vec);
                    timestamps.push_back(t - t0);
                }
            }
        }

        return {final_states, timestamps};
    }

    std::vector<std::vector<float>> read_actions(const std::string& act_path, const std::string& obs_path)
    {
        std::vector<std::vector<float>> final_actions;
        
        // Read Gripper Commands from actions bag
        std::vector<float> gripper_cmds;
        std::vector<geometry_msgs::msg::Pose> poses;

        {
            rosbag2_storage::StorageOptions store_opt; store_opt.uri = act_path; store_opt.storage_id = "sqlite3";
            rosbag2_cpp::readers::SequentialReader reader; reader.open(store_opt, {"",""});
            rclcpp::Serialization<std_msgs::msg::Float64> ser_float;
            rclcpp::Serialization<geometry_msgs::msg::Pose> ser_pose;

            while (reader.has_next()) {
                auto msg = reader.read_next();
                rclcpp::SerializedMessage ser_msg(*msg->serialized_data);

                if (msg->topic_name == TOPIC_ACT_EE_GRIPPER) {
                    std_msgs::msg::Float64 g;
                    ser_float.deserialize_message(&ser_msg, &g);
                    gripper_cmds.push_back((float)g.data);
                }
                else if (action_encoding_ == ActionEncoding::EEF_DELTA && msg->topic_name == TOPIC_ACT_EE_TARGET) {
                    geometry_msgs::msg::Pose p;
                    ser_pose.deserialize_message(&ser_msg, &p);
                    poses.push_back(p);
                }
            }
        }

        // If JOINT_DELTA, read observations for /joint_states
        std::vector<std::vector<double>> joint_positions;
        
        if (action_encoding_ == ActionEncoding::JOINT_DELTA) {
            RCLCPP_INFO(this->get_logger(), "Reading Joint States from Obs Bag for Action Calculation...");
            
            rosbag2_storage::StorageOptions store_opt; store_opt.uri = obs_path; store_opt.storage_id = "sqlite3";
            rosbag2_cpp::readers::SequentialReader reader; reader.open(store_opt, {"",""});
            rclcpp::Serialization<sensor_msgs::msg::JointState> ser_joint;

            while (reader.has_next()) {
                auto msg = reader.read_next();
                rclcpp::SerializedMessage ser_msg(*msg->serialized_data);

                if (msg->topic_name == TOPIC_OBS_JOINT) { // /joint_states
                    sensor_msgs::msg::JointState js;
                    ser_joint.deserialize_message(&ser_msg, &js);
                    joint_positions.push_back(js.position);
                }
            }
        }

        // Process & Merge
        if (action_encoding_ == ActionEncoding::EEF_DELTA) {
             size_t n = std::min(poses.size(), gripper_cmds.size());
            
            Eigen::Vector3f prev_xyz(0,0,0), prev_rpy(0,0,0);
            bool first = true;

            for(size_t i=0; i<n; i++) {
                tf2::Quaternion q(poses[i].orientation.x, poses[i].orientation.y, poses[i].orientation.z, poses[i].orientation.w);
                double r, p, y; tf2::Matrix3x3(q).getRPY(r, p, y);
                
                Eigen::Vector3f xyz(poses[i].position.x, poses[i].position.y, poses[i].position.z);
                Eigen::Vector3f rpy(r, p, y);

                Eigen::Vector3f dpos, drpy;
                if(first) { dpos.setZero(); drpy.setZero(); }
                else      { dpos = xyz - prev_xyz; drpy = rpy - prev_rpy; }

                prev_xyz = xyz; prev_rpy = rpy; first = false;

                final_actions.push_back({
                    dpos.x(), dpos.y(), dpos.z(), 
                    drpy.x(), drpy.y(), drpy.z(), 
                    gripper_cmds[i]
                });
            }
        }
        else if (action_encoding_ == ActionEncoding::JOINT_DELTA) {          
            // assuming 1-to-1 match
            size_t n = std::min(joint_positions.size(), gripper_cmds.size());

            if (n == 0) {
                 RCLCPP_WARN(this->get_logger(), "Zero overlapping joints/gripper cmds found!");
            }

            for(size_t i=0; i<n; i++) {
                std::vector<float> row;
                
                // If it's the very first frame, delta is 0
                if (i == 0) {
                    for(size_t j=0; j<7; j++) row.push_back(0.0f);
                } else {
                    size_t dof = std::min((size_t)7, joint_positions[i].size());
                    
                    for(size_t j=0; j<dof; j++) {
                        double current_q = joint_positions[i][j];
                        double prev_q = joint_positions[i-1][j];
                        double delta = 0.0;

                        // indices 0, 2, 4, 6 are continuous -> Use shortest path
                        if (j == 0 || j == 2 || j == 4 || j == 6) { 
                            delta = get_shortest_delta(current_q, prev_q);
                        } 
                        // indices 1, 3, 5 are limited -> Use standard subtraction
                        else {
                            delta = current_q - prev_q;
                        }
                        
                        row.push_back((float)delta);
                    }
                    
                    // Pad if necessary (if robot has fewer than 7 joints)
                    while(row.size() < 7) row.push_back(0.0f);
                }

                // Append gripper command
                if (i < gripper_cmds.size()) {
                    row.push_back(gripper_cmds[i]);
                } else {
                    row.push_back(0.0f);
                }

                final_actions.push_back(row);
            }
        }

        return final_actions;
    }

    double get_shortest_delta(double current_pos, double prev_pos) {
        double delta = current_pos - prev_pos;
        // If delta is too large positive (e.g. 3.10 - (-3.10) = 6.20), subtract 2*PI
        while (delta > M_PI) delta -= 2.0 * M_PI;
        // If delta is too large negative (e.g. -3.10 - 3.10 = -6.20), add 2*PI
        while (delta < -M_PI) delta += 2.0 * M_PI;
        return delta;
    }

    // Parquet Writer
    static void CheckArrowStatus(const arrow::Status& st) {
        if (!st.ok()) throw std::runtime_error(st.ToString());
    }

    static std::shared_ptr<arrow::Table> make_table(const std::vector<Row>& rows)
    {
        arrow::DoubleBuilder ts_builder;
        arrow::Int32Builder frame_builder, episode_builder, task_builder;
        auto obs_val_builder = std::make_shared<arrow::FloatBuilder>();
        auto act_val_builder = std::make_shared<arrow::FloatBuilder>();
        arrow::ListBuilder obs_builder(arrow::default_memory_pool(), obs_val_builder);
        arrow::ListBuilder act_builder(arrow::default_memory_pool(), act_val_builder);

        for (const auto& row : rows) {
            CheckArrowStatus(ts_builder.Append(row.timestamp));
            CheckArrowStatus(frame_builder.Append(row.frame_index));
            CheckArrowStatus(episode_builder.Append(row.index));
            CheckArrowStatus(task_builder.Append(row.task_index));

            CheckArrowStatus(obs_builder.Append());
            CheckArrowStatus(obs_val_builder->AppendValues(row.obs_state));

            CheckArrowStatus(act_builder.Append());
            CheckArrowStatus(act_val_builder->AppendValues(row.action));
        }

        std::shared_ptr<arrow::Array> ts_arr, frame_arr, ep_arr, task_arr, obs_arr, act_arr;
        ts_builder.Finish(&ts_arr); frame_builder.Finish(&frame_arr);
        episode_builder.Finish(&ep_arr); task_builder.Finish(&task_arr);
        obs_builder.Finish(&obs_arr); act_builder.Finish(&act_arr);

        auto schema = arrow::schema({
            arrow::field("timestamp", arrow::float64()),
            arrow::field("frame_index", arrow::int32()),
            arrow::field("index", arrow::int32()),
            arrow::field("task_index", arrow::int32()),
            arrow::field("observation.state", arrow::list(arrow::float32())),
            arrow::field("action", arrow::list(arrow::float32()))
        });

        return arrow::Table::Make(schema, {ts_arr, frame_arr, ep_arr, task_arr, obs_arr, act_arr});
    }

    void write_parquet(const std::string &path, std::shared_ptr<arrow::Table> table)
    {
        std::shared_ptr<arrow::io::OutputStream> outfile;
        PARQUET_ASSIGN_OR_THROW(outfile, arrow::io::FileOutputStream::Open(path));
        PARQUET_THROW_NOT_OK(
            parquet::arrow::WriteTable(*table, arrow::default_memory_pool(), outfile, 1024)
        );
        RCLCPP_INFO(this->get_logger(), "Wrote Parquet: %s", path.c_str());
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