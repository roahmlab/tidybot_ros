import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("tidybot", package_name="tidybot_moveit_config").to_moveit_configs()
    servo_params = {
        "moveit_servo": ParameterBuilder("tidybot_solver").yaml("config/tidybot_servo_parameters.yaml").to_dict()
    }

    servo_node = launch_ros.actions.Node(
        package="tidybot_solver",
        executable="demo_twist",
        parameters=[
            servo_params,
            {"update_period": 0.01},  # Update period in seconds
            {"planning_group_name": "gen3_7dof"},  # Adjust to your robot's planning group
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},  # Use simulation time if applicable
        ],
        output="screen",
    )
    move_group_launch = generate_move_group_launch(moveit_config)

    return launch.LaunchDescription([
        servo_node,
        move_group_launch
    ])