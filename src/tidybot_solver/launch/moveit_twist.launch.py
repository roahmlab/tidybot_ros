import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
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

    # Our custom joystick_to_moveit node that uses MoveIt Servo C++ API
    joystick_to_moveit_node = launch_ros.actions.Node(
        package="tidybot_solver",
        executable="joystick_to_moveit",
        name="joystick_to_moveit",
        parameters=[
            servo_params,
            {"update_period": 0.01},  # Update period in seconds
            {"planning_group_name": "gen3_7dof"},  
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    return launch.LaunchDescription([
        joystick_to_moveit_node,
    ])
