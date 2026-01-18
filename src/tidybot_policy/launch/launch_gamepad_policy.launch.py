from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory


def launch_gamepad_policy(context, *args, **kwargs):
    controller_type = LaunchConfiguration("controller_type").perform(context)
    config_path = os.path.join(
        get_package_share_directory("tidybot_policy"),
        "config",
        f"{controller_type}_config.yaml",
    )
    if not os.path.isfile(config_path):
        raise RuntimeError(f"Controller config not found: {config_path}")
    
    sim_mode = LaunchConfiguration("sim_mode")
    # Compute use_sim_time based on sim_mode (true if gazebo or isaac)
    use_sim_time_expr = PythonExpression([
        "'", sim_mode, "' != 'hardware'"
    ])
    
    node = Node(
        package="tidybot_policy",
        executable="gamepad_policy",
        name="gamepad_policy",
        parameters=[
            config_path,
            {"sim_mode": sim_mode},
            {"use_sim_time": use_sim_time_expr},
        ],
    )
    return [node]


def generate_launch_description():

    package_dir = get_package_share_directory("tidybot_policy")
    solver_pkg_dir = get_package_share_directory("tidybot_solver")
    joy_node_config = os.path.join(package_dir, "config", "joy_node_config.yaml")
    
    controller_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="Xbox_SeriesX_Wire",
        description="Select joystick controller mapping (Xbox_SeriesX_Wire or Xbox_SeriesX_Wireless)",
        choices=["Xbox_SeriesX_Wire", "Xbox_SeriesX_Wireless"],
    )

    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="hardware",
        description="Simulation mode: 'hardware' for real robot, 'gazebo' for Gazebo sim, 'isaac' for Isaac Sim",
        choices=["hardware", "gazebo", "isaac"],
    )

    record_arg = DeclareLaunchArgument(
        "record",
        default_value="False",
        description="Enable data recording",
        choices=["True", "False"],
    )

    # Compute is_sim based on sim_mode (true if gazebo or isaac)
    is_sim_expr = PythonExpression([
        "'", LaunchConfiguration("sim_mode"), "' != 'hardware'"
    ])

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[joy_node_config],
    )

    joystick_teleop = OpaqueFunction(function=launch_gamepad_policy)

    # Use our custom joystick_to_moveit node with C++ Servo API
    moveit_servo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                solver_pkg_dir,
                "launch",
                "launch_moveit_vel_ik.launch.py",
            )
        ),
        condition=IfCondition(is_sim_expr),
    )

    moveit_servo_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                solver_pkg_dir,
                "launch",
                "launch_moveit_vel_ik.launch.py",
            )
        ),
        condition=UnlessCondition(is_sim_expr),
    )

    phone_teleop_server = Node(
        package="tidybot_policy",
        executable="phone_teleop_server",
        name="phone_teleop_server",
        output="screen",
        parameters=[{"record": LaunchConfiguration("record")}],
    )

    state_controller = Node(
        package="tidybot_policy",
        executable="state_controller",
        name="state_controller",
        output="screen",
        parameters=[
            {"sim_mode": LaunchConfiguration("sim_mode")},
            {"use_remote": True},
            {"record": LaunchConfiguration("record")},
        ],
    )

    return LaunchDescription(
        [
            sim_mode_arg,
            record_arg,
            controller_arg,
            joy_node,
            joystick_teleop,
            moveit_servo_sim,
            moveit_servo_hardware,
            phone_teleop_server,
            state_controller,
        ]
    )
