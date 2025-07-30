#include <chrono>
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
        // Optional: if an output directory is specified, convert the bag to hdf5
        // else just replay the messages
        this->declare_parameter<std::string>("output_dir", "data");
        this->get_parameter("input_dir", root_dir_);
        this->get_parameter("output_dir", output_dir_);
        inspect_bags();
    }

private:
    void inspect_bags()
    {
        // Open output HDF5 file if output_dir is set
        H5::H5File *h5file = nullptr;
        if (output_dir_ != "none")
        {
            // Ensure output directory exists (omitted for brevity)
            std::string out_path = output_dir_;
            // If output_dir_ is a directory, append a file name
            if (fs::is_directory(output_dir_))
            {
                std::string fname = fs::path(root_dir_).filename().string() + ".h5";
                out_path = (fs::path(output_dir_) / fname).string();
            }
            h5file = new H5::H5File(out_path, H5F_ACC_TRUNC);
        }

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
                // Open rosbag2 file (the .db3 inside the directory)
                std::string db3_path;
                for (auto &file : fs::directory_iterator(bag_dir))
                {
                    if (file.path().extension() == ".db3")
                    {
                        db3_path = file.path().string();
                        break;
                    }
                }
                if (db3_path.empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "No .db3 file in %s", bag_name.c_str());
                    continue;
                }
                RCLCPP_INFO(this->get_logger(), "Converting bag %s.", bag_name.c_str());
                // Setup rosbag2 reader
                rosbag2_storage::StorageOptions storage_options;
                storage_options.uri = db3_path;
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

                // Open videos using OpenCV
                std::string prefix = "observations_";
                std::string base_video = (bag_dir / fs::path("base_video_" + bag_name.substr(prefix.length()) + ".mp4")).string();
                RCLCPP_INFO(this->get_logger(), "Opening video: %s", base_video.c_str());
                std::string arm_video = (bag_dir / fs::path("arm_video_" + bag_name.substr(prefix.length()) + ".mp4")).string();
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
                if (static_cast<size_t>(frame_count) != N)
                {
                    throw std::runtime_error("Frame count mismatch in base_video of " + bag_name);
                }
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
                if (static_cast<size_t>(frame_count) != N)
                {
                    throw std::runtime_error("Frame count mismatch in arm_video of " + bag_name);
                }

                // Compute data arrays for HDF5
                std::vector<double> actions_data;
                actions_data.resize(N * 10);
                std::vector<double> arm_pos_data, arm_quat_data, base_pose_data, gripper_data;
                arm_pos_data.reserve(N * 3);
                arm_quat_data.reserve(N * 4);
                base_pose_data.reserve(N * 3);
                gripper_data.reserve(N);

                // Populate obs datasets and also build actions dataset entries
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

                    // Prepare state vector S[i] for actions (10 elements)
                    double arm_axis_x = 0.0, arm_axis_y = 0.0, arm_axis_z = 0.0;
                    // Convert arm quaternion to axis-angle (rotation vector)
                    double theta = 2 * acos(qw);
                    if (theta > 1e-6)
                    {
                        double norm = sqrt(1 - qw * qw);
                        arm_axis_x = theta * (qx / norm);
                        arm_axis_y = theta * (qy / norm);
                        arm_axis_z = theta * (qz / norm);
                    }
                    else
                    {
                        // Zero rotation (set axis-angle to 0 vector)
                        arm_axis_x = arm_axis_y = arm_axis_z = 0.0;
                    }
                    // Base yaw already computed as 'yaw'
                    // Fill S[i] = [base_x, base_y, base_yaw, arm_pos(x,y,z), arm_axis(x,y,z), gripper]
                    double Sx[10] = {bx, by, yaw, ax, ay, az, arm_axis_x, arm_axis_y, arm_axis_z, grip_val};

                    // Place into actions_data (to be shifted later)
                    for (int j = 0; j < 10; ++j)
                    {
                        // We will actually assign to actions_data after computing shift
                        actions_data[i * 10 + j] = Sx[j];
                    }
                }

                // Apply shift: actions[0] = interpolated (average) of S0 and S1, actions[i] = S[i-1] for i>=1
                if (N > 0)
                {
                    // If N>=3, interpolate first entry
                    if (N > 2)
                    {
                        // Shift actions_data by the size of one entry (10 elements)
                        shift_right_drop(actions_data, 10);
                        for (int j = 0; j < 10; ++j)
                        {
                            // For orientation components in axis-angle (indices 2 and 6-8), a direct average is an approximation.
                            actions_data[j] = 0.5 * (actions_data[10 + j] + actions_data[20 + j]);
                        }
                    }
                    else if (N == 2)
                    {
                        // For N==2, copy the second to the first
                        shift_right_drop(actions_data, 10);
                        for (int j = 0; j < 10; ++j)
                        {
                            actions_data[j] = actions_data[10 + j]; // Copy second entry to first
                        }
                    }
                    else // N == 1
                    {
                        // If N==1, just copy the first entry as is
                        for (int j = 0; j < 10; ++j)
                        {
                            actions_data[j] = actions_data[j];
                        }
                    }
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
                writeDataset2D(bagGroup, "actions", N, 10, H5::PredType::NATIVE_DOUBLE, actions_data.data());
                writeDataset2D(obsGroup, "arm_pos", N, 3, H5::PredType::NATIVE_DOUBLE, arm_pos_data.data());
                writeDataset2D(obsGroup, "arm_quat", N, 4, H5::PredType::NATIVE_DOUBLE, arm_quat_data.data());
                writeDataset2D(obsGroup, "base_pose", N, 3, H5::PredType::NATIVE_DOUBLE, base_pose_data.data());
                writeDataset2D(obsGroup, "gripper_pos", N, 1, H5::PredType::NATIVE_DOUBLE, gripper_data.data());
                writeDataset4D(obsGroup, "base_image", N, 84, 84, 3, H5::PredType::NATIVE_UCHAR, base_img_data.data());
                writeDataset4D(obsGroup, "wrist_image", N, 84, 84, 3, H5::PredType::NATIVE_UCHAR, wrist_img_data.data());
            }
        }

        if (h5file)
        {
            RCLCPP_INFO(this->get_logger(), "HDF5 file created successfully at %s", h5file->getFileName().c_str());
            delete h5file; // close HDF5 file
        }
    }

    void shift_right_drop(
        std::vector<double> &v,
        size_t k,
        double fill_value = 0.0)
    {
        size_t n = v.size();
        if (n == 0)
        {
            return;
        }
        // clamp k to [0,n]
        k = std::min(k, n);

        std::copy_backward(
            v.begin(),
            v.begin() + (n - k),
            v.end());

        std::fill(v.begin(), v.begin() + k, fill_value);
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
