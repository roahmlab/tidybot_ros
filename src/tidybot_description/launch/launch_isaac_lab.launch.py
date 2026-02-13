"""
Launch TidyBot with Isaac Lab

This launch file starts all necessary nodes to run TidyBot with Isaac Lab
(NVIDIA's GPU-accelerated robotics simulation). Isaac Lab runs as a separate
process and communicates via ROS 2 topics.

Nodes launched:
    - robot_state_publisher: Publishes /tf from /joint_states
    - tf_relay: Relays TF to separate topics for RViz compatibility
    - isaac_lab_bridge: Monitors connection to Isaac Lab
    - rviz2: Visualization with camera feeds (optional)
    - multi_stage_planner: MoveIt-based motion executor (for drawer tasks)
    - drawer_policy: Task-specific policy server (for drawer tasks)
    - sensor_data_recorder: Records contact forces & drawer state (optional)

Usage:
    # First, start Isaac Lab simulation:
    cd isaaclab && ./isaaclab.sh -p tidybot_isaac/scripts/isaac_lab_sim.py --num_envs 1

    # Then launch this:
    ros2 launch tidybot_description launch_isaac_lab.launch.py

    # Without drawer policy stack:
    ros2 launch tidybot_description launch_isaac_lab.launch.py enable_drawer_policy:=false

Arguments:
    use_rviz: Launch RViz for visualization (default: true)
    rviz_config: Path to RViz config file
    enable_drawer_policy: Launch drawer policy stack (default: true)
    record_sensor_data: Enable sensor data recording (default: true)
    output_dir: Directory for recorded sensor data CSV files
    approach_distance: Distance to approach drawer handle from (meters)
"""

import xacro
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    tidybot_pkg = FindPackageShare("tidybot_description")
    default_rviz_config = PathJoinSubstitution([tidybot_pkg, "config", "tidybot.rviz"])
    
    # Robot description from xacro
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([tidybot_pkg, "urdf", "tidybot.xacro"]),
    ])
    
    # Generate URDF for MoveIt (needed by multi_stage_planner)
    robot_description_path = get_package_share_directory("tidybot_description")
    doc = xacro.process_file(str(robot_description_path + "/urdf/tidybot.xacro"))
    urdf_xml = doc.toxml()
    outpath = Path(robot_description_path) / "urdf/tidybot.urdf"
    outpath.write_text(urdf_xml, encoding="utf-8")
    
    # Load MoveIt Configs
    moveit_config = MoveItConfigsBuilder(
        "tidybot", package_name="tidybot_moveit_config"
    ).robot_description(
        file_path=str(outpath)
    ).to_moveit_configs()
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz for visualization"
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Path to RViz config file"
        ),
        DeclareLaunchArgument(
            "enable_drawer_policy",
            default_value="true",
            description="Launch drawer policy stack (planner + policy + recorder)"
        ),
        DeclareLaunchArgument(
            "record_sensor_data",
            default_value="true",
            description="Enable sensor data recording during drawer tasks"
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="./sensor_data",
            description="Directory for recorded sensor data CSV files"
        ),
        DeclareLaunchArgument(
            "approach_distance",
            default_value="0.15",
            description="Distance to approach drawer handle from (meters)"
        ),
        
        # Robot State Publisher
        # Publishes /tf and /tf_static from /joint_states
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description_content,
                "use_sim_time": True,
                "ignore_timestamp": True,  # Important for simulation timing
            }],
        ),
        
        # TF Relay
        # Relays TF to /tf_relay and /tf_static_relay for RViz compatibility
        Node(
            package="tidybot_description",
            executable="tf_relay",
            name="tf_relay",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        
        # Isaac Lab Bridge
        # Monitors connection to Isaac Lab simulation
        Node(
            package="tidybot_description",
            executable="isaac_lab_bridge",
            name="isaac_lab_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "timeout_sec": 10.0,
            }],
        ),
        
        # RViz (optional)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": True}],
            remappings=[
                ("/tf", "/tf_relay"),
                ("/tf_static", "/tf_static_relay"),
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        
        # === Drawer Policy Stack (optional) ===
        
        # Multi-Stage Planner (MoveIt-based motion executor)
        Node(
            condition=IfCondition(LaunchConfiguration("enable_drawer_policy")),
            package="tidybot_solver",
            executable="multi_stage_planner",
            name="multi_stage_planner",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "use_sim_time": True,
                    "arm_group": "gen3_7dof",
                    "tip_link": "tool_frame",
                }
            ],
        ),
        
        # Drawer Policy (task-specific policy server)
        Node(
            condition=IfCondition(LaunchConfiguration("enable_drawer_policy")),
            package="tidybot_policy",
            executable="drawer_policy",
            name="drawer_policy",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "approach_distance": LaunchConfiguration("approach_distance"),
                "approach_duration": 3.0,
                "grasp_duration": 2.0,
                "pull_duration": 2.5,
                "gripper_duration": 1.0,
                "wait_after_grasp_duration": 2.0,
                "record_sensor_data": LaunchConfiguration("record_sensor_data"),
            }],
        ),
        
        # Sensor Data Recorder (records contact forces & drawer state to CSV)
        Node(
            condition=IfCondition(LaunchConfiguration("record_sensor_data")),
            package="tidybot_episode",
            executable="sensor_data_recorder",
            name="sensor_data_recorder",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "output_dir": LaunchConfiguration("output_dir"),
                "record_rate": 20.0,
            }],
        ),
    ])

