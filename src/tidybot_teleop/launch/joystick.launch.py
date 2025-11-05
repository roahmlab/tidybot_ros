from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils.launches import generate_move_group_launch

import os
from ament_index_python.packages import get_package_share_directory


def launch_joystick_teleop(context, *args, **kwargs):
    controller_type = LaunchConfiguration("controller_type").perform(context)
    config_path = os.path.join(
        get_package_share_directory("tidybot_teleop"),
        "config",
        f"{controller_type}_config.yaml",
    )
    if not os.path.isfile(config_path):
        raise RuntimeError(f"Controller config not found: {config_path}")
    node = Node(
        package="tidybot_teleop",
        executable="joystick_teleop",
        name="joystick_teleop",
        parameters=[
            config_path,
            {"use_sim": LaunchConfiguration("use_sim")},
            {"use_sim_time": LaunchConfiguration("use_sim")},
        ],
    )
    return [node]


def generate_launch_description():

    package_dir = get_package_share_directory("tidybot_teleop")
    solver_pkg_dir = get_package_share_directory("tidybot_solver")
    joy_node_config = os.path.join(package_dir, "config", "joy_node_config.yaml")
    controller_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="Xbox_SeriesX_Wire",
        description="Select joystick controller mapping (Xbox_SeriesX_Wire or Xbox_SeriesX_Wireless)",
        choices=["Xbox_SeriesX_Wire", "Xbox_SeriesX_Wireless"],
    )

    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Use simulation (Gazebo) if true, hardware if false",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[joy_node_config],
    )

    joystick_teleop = OpaqueFunction(function=launch_joystick_teleop)

    # Use our custom joystick_to_moveit node with C++ Servo API
    moveit_servo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                solver_pkg_dir,
                "launch",
                "launch_moveit_vel_ik.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    moveit_servo_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                solver_pkg_dir,
                "launch",
                "launch_moveit_vel_ik.launch.py",
            )
        ),
        condition=UnlessCondition(LaunchConfiguration("use_sim")),
    )

    return LaunchDescription(
        [
            use_sim_arg,
            controller_arg,
            joy_node,
            joystick_teleop,
            moveit_servo_sim,
            moveit_servo_hardware,
        ]
    )
