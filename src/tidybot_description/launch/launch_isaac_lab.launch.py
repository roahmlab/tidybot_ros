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

Usage:
    # First, start Isaac Lab simulation:
    cd isaaclab && ./isaaclab.sh -p tidybot_isaac/scripts/isaac_lab_sim.py --num_envs 1

    # Then launch this:
    ros2 launch tidybot_description launch_isaac_lab.launch.py

Arguments:
    use_rviz: Launch RViz for visualization (default: true)
    rviz_config: Path to RViz config file
"""

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


def generate_launch_description():
    tidybot_pkg = FindPackageShare("tidybot_description")
    default_rviz_config = PathJoinSubstitution([tidybot_pkg, "config", "tidybot.rviz"])
    
    # Robot description from xacro
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([tidybot_pkg, "urdf", "tidybot.xacro"]),
    ])
    
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
    ])
