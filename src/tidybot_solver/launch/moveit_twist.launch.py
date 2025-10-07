import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='Use simulation (Gazebo) if true, hardware if false'
    )

    moveit_config = (MoveItConfigsBuilder("tidybot", package_name="tidybot_moveit_config")
                     .robot_description(file_path="config/tidybot.urdf.xacro")
                     .joint_limits(file_path="config/joint_limits.yaml")
                     .robot_description_semantic()
                     .to_moveit_configs()
    )
    
    # Servo parameters for the C++ API
    servo_params = {
        "moveit_servo": ParameterBuilder("tidybot_solver").yaml("config/tidybot_servo_config.yaml").to_dict()
    }

    twist_to_moveit_node_sim = launch_ros.actions.Node(
        package="tidybot_solver",
        executable="twist_to_moveit",
        name="twist_to_moveit",
        parameters=[
            servo_params,
            {"update_period": 0.001},  # Update period in seconds
            {"planning_group_name": "gen3_7dof"},  
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
            {"command_output_type": "joint_trajectory"},
            {"base_frame": "world"},
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim"))
    )

    twist_to_moveit_node_hardware = launch_ros.actions.Node(
        package="tidybot_solver",
        executable="twist_to_moveit",
        name="twist_to_moveit",
        parameters=[
            servo_params,
            {"update_period": 0.002},  # Update period in seconds
            {"planning_group_name": "gen3_7dof"},  
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": False},
            {"command_output_type": "joint_state"},
            {"base_frame": "base"},
        ],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_sim"))
    )

    return LaunchDescription([
        use_sim_arg,
        twist_to_moveit_node_sim,
        twist_to_moveit_node_hardware,
    ])
