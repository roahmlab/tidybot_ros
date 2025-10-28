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
#include "opencv2/opencv.hpp"
#include "H5Cpp.h"

namespace fs = std::filesystem;

class ObsReader : public rclcpp::Node
{
public:
    ObsReader()
        : Node("obs_reader")
    {
        this->declare_parameter<std::string>("input_dir", "episode_bag");
        this->declare_parameter<std::string>("output_dir", "data.hdf5");
        this->get_parameter("input_dir", root_dir_);
        this->get_parameter("output_dir", output_dir_);
        inspect_bags();
    }

private:
    // The .hdf5 file structure:
    // /root
    // |-- /data
    //     |-- /demo_<demo_id>
    //         |-- actions (N x 10)
    //         |-- /obs
    //             |-- arm_pos (N x 3)
    //             |-- arm_quat (N x 4)
    //             |-- base_image (N x 84 x 84 x 3)
    //             |-- base_pose (N x 3)
    //             |-- gripper_pos (N x 1)
    //             |-- wrist_image (N x 84 x 84 x 3)
    void inspect_bags()
    {
        // Open output HDF5 file
        H5::H5File *h5file = nullptr;
        // Ensure output directory exists (omitted for brevity)
        std::string out_path = output_dir_;
        // If output_dir_ is a directory, append a file name
        if (fs::is_directory(output_dir_))
        {
            std::string fname = fs::path(root_dir_).filename().string() + ".hdf5";
            out_path = (fs::path(output_dir_) / fname).string();
        }
        h5file = new H5::H5File(out_path, H5F_ACC_TRUNC);

        H5::Group dataGroup = h5file->createGroup("/data");

        for (auto &entry : fs::directory_iterator(root_dir_))
        {
            if (!entry.is_directory())
                continue;
            fs::path bag_dir = entry.path();
            RCLCPP_INFO(this->get_logger(), "Processing bag directory: %s", bag_dir.string().c_str());
            std::string bag_name = bag_dir.filename().string(); // group name

            // Only process if converting to HDF5
            if (h5file)
            {
                RCLCPP_INFO(this->get_logger(), "Converting bag: %s to HDF5", bag_dir.string().c_str());
                fs::path obs_dir = bag_dir / "observations";
                fs::path actions_dir = bag_dir / "actions";

                if (!fs::exists(obs_dir) || !fs::is_directory(obs_dir))
                {
                    RCLCPP_ERROR(this->get_logger(), "Observations directory missing in %s", bag_name.c_str());
                    continue;
                }
                if (!fs::exists(actions_dir) || !fs::is_directory(actions_dir))
                {
                    RCLCPP_ERROR(this->get_logger(), "Actions directory missing in %s", bag_name.c_str());
                    continue;
                }

                std::string obs_db3_path;
                for (auto &file : fs::directory_iterator(obs_dir))
                {
                    if (file.path().extension() == ".db3")
                    {
                        obs_db3_path = file.path().string();
                        break;
                    }
                }
                if (obs_db3_path.empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "No observation .db3 file in %s", bag_name.c_str());
                    continue;
                }

                std::string actions_db3_path;
                for (auto &file : fs::directory_iterator(actions_dir))
                {
                    if (file.path().extension() == ".db3")
                    {
                        actions_db3_path = file.path().string();
                        break;
                    }
                }
                if (actions_db3_path.empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "No actions .db3 file in %s", bag_name.c_str());
                    continue;
                }

                RCLCPP_INFO(this->get_logger(), "Converting bag %s.", bag_name.c_str());
                // Setup rosbag2 reader
                rosbag2_storage::StorageOptions storage_options;
                storage_options.uri = obs_db3_path;
                storage_options.storage_id = "sqlite3";
                rosbag2_cpp::ConverterOptions converter_options;
                converter_options.input_serialization_format = "cdr";
                converter_options.output_serialization_format = "cdr";
                rosbag2_cpp::readers::SequentialReader reader;
                reader.open(storage_options, converter_options);

                // Buffers for collected data
                std::vector<geometry_msgs::msg::PoseStamped> base_msgs;
                std::vector<geometry_msgs::msg::PoseStamped> arm_msgs;
                std::vector<std_msgs::msg::Float64> gripper_msgs;

                // Prepare deserializers
                rclcpp::Serialization<geometry_msgs::msg::PoseStamped> pose_serializer;
                rclcpp::Serialization<std_msgs::msg::Float64> float_serializer;

                // Read all messages in the bag
                while (reader.has_next())
                {
                    auto serialized_msg = reader.read_next();
                    rclcpp::SerializedMessage extracted(*serialized_msg->serialized_data);
                    const std::string &topic = serialized_msg->topic_name;
                    if (topic == "/base_pose")
                    {
                        geometry_msgs::msg::PoseStamped base;
                        pose_serializer.deserialize_message(&extracted, &base);
                        base_msgs.push_back(base);
                    }
                    else if (topic == "/arm_pose")
                    {
                        geometry_msgs::msg::PoseStamped arm;
                        pose_serializer.deserialize_message(&extracted, &arm);
                        arm_msgs.push_back(arm);
                    }
                    else if (topic == "/gripper_state")
                    {
                        std_msgs::msg::Float64 grip;
                        float_serializer.deserialize_message(&extracted, &grip);
                        gripper_msgs.push_back(grip);
                    }
                }

                size_t N = base_msgs.size();
                // Ensure equal counts
                if (arm_msgs.size() != N || gripper_msgs.size() != N)
                {
                    throw std::runtime_error("Message count mismatch in bag " + bag_name);
                }

                // Read actions bag
                rosbag2_storage::StorageOptions action_storage_options;
                action_storage_options.uri = actions_db3_path;
                action_storage_options.storage_id = "sqlite3";
                rosbag2_cpp::readers::SequentialReader action_reader;
                action_reader.open(action_storage_options, converter_options);

                std::vector<double> base_cmd_x;
                std::vector<double> base_cmd_y;
                std::vector<double> base_cmd_yaw;
                std::vector<geometry_msgs::msg::Pose> arm_cmds;
                std::vector<std_msgs::msg::Float64> gripper_cmds;

                rclcpp::Serialization<std_msgs::msg::Float64MultiArray> base_cmd_serializer;
                rclcpp::Serialization<geometry_msgs::msg::Pose> arm_cmd_serializer;

                while (action_reader.has_next())
                {
                    auto serialized_msg = action_reader.read_next();
                    rclcpp::SerializedMessage extracted(*serialized_msg->serialized_data);
                    const std::string &topic = serialized_msg->topic_name;

                    if (topic == "/tidybot/base/target_pose")
                    {
                        std_msgs::msg::Float64MultiArray base_cmd;
                        base_cmd_serializer.deserialize_message(&extracted, &base_cmd);
                        if (base_cmd.data.size() < 3)
                        {
                            RCLCPP_WARN(this->get_logger(),
                                        "Base command in %s has insufficient length (%zu)",
                                        bag_name.c_str(), base_cmd.data.size());
                            continue;
                        }
                        base_cmd_x.push_back(base_cmd.data[0]);
                        base_cmd_y.push_back(base_cmd.data[1]);
                        base_cmd_yaw.push_back(base_cmd.data[2]);
                    }
                    else if (topic == "/tidybot/arm/target_pose")
                    {
                        geometry_msgs::msg::Pose arm_cmd;
                        arm_cmd_serializer.deserialize_message(&extracted, &arm_cmd);
                        arm_cmds.push_back(arm_cmd);
                    }
                    else if (topic == "/tidybot/gripper/commands")
                    {
                        std_msgs::msg::Float64 grip_cmd;
                        float_serializer.deserialize_message(&extracted, &grip_cmd);
                        gripper_cmds.push_back(grip_cmd);
                    }
                }

                size_t base_samples = base_cmd_x.size();
                size_t arm_cmd_samples = arm_cmds.size();
                size_t grip_cmd_samples = gripper_cmds.size();
                size_t action_samples = std::min({base_samples, arm_cmd_samples, grip_cmd_samples});

                if (action_samples == 0)
                {
                    RCLCPP_WARN(this->get_logger(), "No action samples found in %s; skipping", bag_name.c_str());
                    continue;
                }

                if (base_samples != action_samples || arm_cmd_samples != action_samples || grip_cmd_samples != action_samples)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Action topic counts mismatch in %s (base=%zu, arm=%zu, gripper=%zu); truncating to %zu",
                                bag_name.c_str(), base_samples, arm_cmd_samples, grip_cmd_samples, action_samples);
                }

                if (action_samples != N)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Observation and action counts differ in %s (obs=%zu, act=%zu); truncating to %zu",
                                bag_name.c_str(), N, action_samples, std::min(N, action_samples));
                }

                size_t sample_count = std::min(N, action_samples);
                if (sample_count == 0)
                {
                    RCLCPP_WARN(this->get_logger(), "No synchronized samples available in %s; skipping", bag_name.c_str());
                    continue;
                }

                // Open videos using OpenCV
                std::string base_video = (bag_dir / fs::path("base_camera.mp4")).string();
                RCLCPP_INFO(this->get_logger(), "Opening video: %s", base_video.c_str());
                std::string arm_video = (bag_dir / fs::path("arm_camera.mp4")).string();
                RCLCPP_INFO(this->get_logger(), "Opening video: %s", arm_video.c_str());
                cv::VideoCapture base_cap(base_video), arm_cap(arm_video);
                if (!base_cap.isOpened() || !arm_cap.isOpened())
                {
                    throw std::runtime_error("Failed to open video in bag " + bag_name);
                }
                RCLCPP_INFO(this->get_logger(), "Converting videos: %s, %s", base_video.c_str(), arm_video.c_str());

                // Prepare image buffers
                std::vector<uint8_t> base_img_data;
                std::vector<uint8_t> wrist_img_data;
                base_img_data.reserve(N * 84 * 84 * 3);
                wrist_img_data.reserve(N * 84 * 84 * 3);

                cv::Mat frame;
                int frame_count = 0;
                // Read base video frames
                while (base_cap.read(frame))
                {
                    cv::Mat resized;
                    cv::resize(frame, resized, cv::Size(84, 84));
                    // Append pixels (BGR) to buffer
                    base_img_data.insert(base_img_data.end(),
                                         resized.data, resized.data + (84 * 84 * 3));
                    frame_count++;
                }
                size_t base_frames = static_cast<size_t>(frame_count);
                // Read arm(wrist) video frames
                frame_count = 0;
                while (arm_cap.read(frame))
                {
                    cv::Mat resized;
                    cv::resize(frame, resized, cv::Size(84, 84));
                    wrist_img_data.insert(wrist_img_data.end(),
                                          resized.data, resized.data + (84 * 84 * 3));
                    frame_count++;
                }
                size_t wrist_frames = static_cast<size_t>(frame_count);

                if (base_frames != N)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Base video frame count (%zu) differs from observation count (%zu) in %s",
                                base_frames, N, bag_name.c_str());
                }
                if (wrist_frames != N)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Wrist video frame count (%zu) differs from observation count (%zu) in %s",
                                wrist_frames, N, bag_name.c_str());
                }

                sample_count = std::min(sample_count, base_frames);
                sample_count = std::min(sample_count, wrist_frames);
                if (sample_count == 0)
                {
                    RCLCPP_WARN(this->get_logger(), "No synchronized samples after video alignment in %s; skipping", bag_name.c_str());
                    continue;
                }

                // Compute data arrays for HDF5
                std::vector<double> actions_data(sample_count * 10, 0.0);
                std::vector<double> arm_pos_data, arm_quat_data, base_pose_data, gripper_data;
                arm_pos_data.reserve(N * 3);
                arm_quat_data.reserve(N * 4);
                base_pose_data.reserve(N * 3);
                gripper_data.reserve(N);

                // Populate observation datasets
                for (size_t i = 0; i < N; ++i)
                {
                    // Get messages
                    const auto &base = base_msgs[i];
                    const auto &arm = arm_msgs[i];
                    const auto &grip = gripper_msgs[i];

                    // Base pose: x, y, yaw
                    double bx = base.pose.position.x;
                    double by = base.pose.position.y;
                    // Compute yaw from quaternion (ROS uses heading=Yaw as rotation about Z)
                    tf2::Quaternion q_base;
                    tf2::fromMsg(base.pose.orientation, q_base);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(q_base).getRPY(roll, pitch, yaw);
                    base_pose_data.insert(base_pose_data.end(), {bx, by, yaw});

                    // Arm position
                    double ax = arm.pose.position.x;
                    double ay = arm.pose.position.y;
                    double az = arm.pose.position.z;
                    arm_pos_data.insert(arm_pos_data.end(), {ax, ay, az});
                    // Arm orientation quaternion
                    double qx = arm.pose.orientation.x;
                    double qy = arm.pose.orientation.y;
                    double qz = arm.pose.orientation.z;
                    double qw = arm.pose.orientation.w;
                    arm_quat_data.insert(arm_quat_data.end(), {qx, qy, qz, qw});

                    // Gripper position
                    double grip_val = grip.data;
                    gripper_data.push_back(grip_val);

                }

                // Prepare action dataset using recorded command topics
                for (size_t i = 0; i < sample_count; ++i)
                {
                    double bx = base_cmd_x[i];
                    double by = base_cmd_y[i];
                    double byaw = base_cmd_yaw[i];

                    const auto &arm_cmd = arm_cmds[i];
                    double ax = arm_cmd.position.x;
                    double ay = arm_cmd.position.y;
                    double az = arm_cmd.position.z;

                    double qx = arm_cmd.orientation.x;
                    double qy = arm_cmd.orientation.y;
                    double qz = arm_cmd.orientation.z;
                    double qw = arm_cmd.orientation.w;

                    double qw_clamped = std::clamp(qw, -1.0, 1.0);
                    double theta = 2.0 * std::acos(qw_clamped);
                    double sin_half = std::sqrt(std::max(0.0, 1.0 - qw_clamped * qw_clamped));
                    double rx = 0.0, ry = 0.0, rz = 0.0;
                    if (sin_half > 1e-6)
                    {
                        rx = theta * (qx / sin_half);
                        ry = theta * (qy / sin_half);
                        rz = theta * (qz / sin_half);
                    }

                    double grip_val = gripper_cmds[i].data;

                    double *slot = &actions_data[i * 10];
                    slot[0] = bx;
                    slot[1] = by;
                    slot[2] = byaw;
                    slot[3] = ax;
                    slot[4] = ay;
                    slot[5] = az;
                    slot[6] = rx;
                    slot[7] = ry;
                    slot[8] = rz;
                    slot[9] = grip_val;
                }

                // Truncate observation buffers if needed to match sample_count
                if (sample_count < N)
                {
                    arm_pos_data.resize(sample_count * 3);
                    arm_quat_data.resize(sample_count * 4);
                    base_pose_data.resize(sample_count * 3);
                    gripper_data.resize(sample_count);
                }

                if (sample_count < base_frames)
                {
                    base_img_data.resize(sample_count * 84 * 84 * 3);
                }
                if (sample_count < wrist_frames)
                {
                    wrist_img_data.resize(sample_count * 84 * 84 * 3);
                }

                // Write to HDF5: create group and datasets
                std::string bag_prefix = "demo_";
                RCLCPP_INFO(this->get_logger(), "Writing HDF5 file for bag: %s", bag_name.c_str());
                H5::Group bagGroup = dataGroup.createGroup(bag_prefix.append(std::to_string(num_bags_++)));
                H5::Group obsGroup = bagGroup.createGroup("obs");

                auto writeDataset2D = [&](H5::Group &grp, const std::string &name,
                                          hsize_t dim1, hsize_t dim2, H5::PredType type, const void *data)
                {
                    hsize_t dims[2] = {dim1, dim2};
                    H5::DataSpace dspace(2, dims);
                    H5::DataSet dset = grp.createDataSet(name, type, dspace);
                    dset.write(data, type);
                };
                auto writeDataset4D = [&](H5::Group &grp, const std::string &name,
                                          hsize_t dim1, hsize_t dim2, hsize_t dim3, hsize_t dim4,
                                          H5::PredType type, const void *data)
                {
                    hsize_t dims[4] = {dim1, dim2, dim3, dim4};
                    H5::DataSpace dspace(4, dims);
                    H5::DataSet dset = grp.createDataSet(name, type, dspace);
                    dset.write(data, type);
                };

                // Write datasets
                // actions: N x 10
                writeDataset2D(bagGroup, "actions", sample_count, 10, H5::PredType::NATIVE_DOUBLE, actions_data.data());
                writeDataset2D(obsGroup, "arm_pos", sample_count, 3, H5::PredType::NATIVE_DOUBLE, arm_pos_data.data());
                writeDataset2D(obsGroup, "arm_quat", sample_count, 4, H5::PredType::NATIVE_DOUBLE, arm_quat_data.data());
                writeDataset2D(obsGroup, "base_pose", sample_count, 3, H5::PredType::NATIVE_DOUBLE, base_pose_data.data());
                writeDataset2D(obsGroup, "gripper_pos", sample_count, 1, H5::PredType::NATIVE_DOUBLE, gripper_data.data());
                writeDataset4D(obsGroup, "base_image", sample_count, 84, 84, 3, H5::PredType::NATIVE_UCHAR, base_img_data.data());
                writeDataset4D(obsGroup, "wrist_image", sample_count, 84, 84, 3, H5::PredType::NATIVE_UCHAR, wrist_img_data.data());
            }
        }

        if (h5file)
        {
            RCLCPP_INFO(this->get_logger(), "HDF5 file created successfully at %s", h5file->getFileName().c_str());
            delete h5file; // close HDF5 file
        }
    }

    std::string root_dir_;
    std::string output_dir_;
    std::unordered_map<
        std::string,
        rclcpp::GenericPublisher::SharedPtr>
        pubs_;
    int num_bags_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObsReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
