"""
Launch TidyBot with Isaac Sim

This launch file starts all necessary nodes to run TidyBot with Isaac Sim
instead of Gazebo. Isaac Sim must be running separately with the ROS 2
Action Graph configured.

Nodes launched:
    - robot_state_publisher: Publishes /tf from /joint_states
    - tf_relay: Relays TF to separate topics for RViz compatibility
    - isaac_sim_bridge: Bridges ros2_control topics to Isaac Sim's /joint_command

Usage:
    ros2 launch tidybot_description launch_isaac_sim.launch.py

Arguments:
    use_rviz: Launch RViz for visualization (default: true)
    publish_rate: Isaac Sim bridge publishing rate in Hz (default: 50.0)
    use_velocity_control: Use velocity control for base (default: false)
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
            "publish_rate",
            default_value="50.0",
            description="Isaac Sim bridge publishing rate in Hz"
        ),
        DeclareLaunchArgument(
            "use_velocity_control",
            default_value="false",
            description="Use velocity control for base instead of position"
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
                "ignore_timestamp": True,  # Important for Isaac Sim timing
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
        
        # Isaac Sim Bridge
        # Bridges ros2_control topics to Isaac Sim's /joint_command
        Node(
            package="tidybot_description",
            executable="isaac_sim_bridge",
            name="isaac_sim_bridge",
            output="screen",
            parameters=[{
                "publish_rate": LaunchConfiguration("publish_rate"),
                "use_velocity_control": LaunchConfiguration("use_velocity_control"),
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

