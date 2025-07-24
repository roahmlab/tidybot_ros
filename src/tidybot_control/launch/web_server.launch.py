import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
from pathlib import Path

def generate_launch_description():
    tidybot_moveit_pkg = FindPackageShare("tidybot_moveit_config")
    tidybot_description_pkg = FindPackageShare("tidybot_description")

    use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value='true',
        description="Use simulation mode if true"
    )

    robot_description_path = get_package_share_directory("tidybot_description")
    doc = xacro.process_file(str(robot_description_path + "/urdf/tidybot.xacro"))
    urdf_xml = doc.toxml()
    outpath = Path(robot_description_path) / "urdf/tidybot.urdf"
    outpath.write_text(urdf_xml, encoding="utf-8")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tidybot_moveit_pkg, "launch", "demo.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("use_sim"))
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tidybot_description_pkg, "launch", "display.launch.py"])
        ),
        launch_arguments={
            "ignore_timestamp": "false",
            "jsp_gui": "false"
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_sim"))
    )

    kinematics_yaml_path = os.path.join(
        get_package_share_directory('tidybot_moveit_config'),
        'config',
        'kinematics.yaml'
    )

    with open(kinematics_yaml_path, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    web_server_publisher = Node(
        package="tidybot_control",
        executable="web_server_publisher",
        name="web_server_publisher",
        output="screen",
    )

    web_server_relay = Node(
        package="tidybot_control",
        executable="ws_relay",
        name="ws_relay",
        output="screen",
        parameters=[{"use_sim_time": True},
                    {"use_sim": LaunchConfiguration("use_sim")}],
        remappings=[
            ("/tf", "/tf_relay"),
            ("/tf_static", "/tf_static_relay"),
        ]
    )

    state_controller = Node(
        package="tidybot_control",
        executable="state_controller",
        name="state_controller",
        output="screen",
        parameters=[{"use_sim": LaunchConfiguration("use_sim")}]
    )
    
    web_server_moveit = Node(
        package="tidybot_moveit_config",
        executable="web_server_moveit",
        name="web_server_moveit",
        output="screen",
        parameters=[{"robot_description_kinematics": kinematics_config}],
        condition=IfCondition(LaunchConfiguration("use_sim"))
    )

    episode_recorder = Node(
        package="tidybot_episode",
        executable="record_episode",
        name="record_episode",
        output="screen",
        parameters=[{"use_sim": LaunchConfiguration("use_sim")}]
    )

    return LaunchDescription([
        use_sim,
        # moveit_launch,
        rviz_launch,
        web_server_publisher,
        web_server_relay,
        state_controller,
        # web_server_moveit,
        episode_recorder,   
    ])