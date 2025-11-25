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
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():

    use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value='true',
        description="Use simulation mode if true"
    )

    record = DeclareLaunchArgument(
        "record",
        default_value='false',
        description="Enable episode recording services and node if true"
    )

    cameras = DeclareLaunchArgument(
        "cameras",
        default_value='["base","arm"]',
        description="Comma-separated list (YAML) of cameras to record (base, arm, ext)"
    )

    robot_description_path = get_package_share_directory("tidybot_description")
    doc = xacro.process_file(str(robot_description_path + "/urdf/tidybot.xacro"))
    urdf_xml = doc.toxml()
    outpath = Path(robot_description_path) / "urdf/tidybot.urdf"
    outpath.write_text(urdf_xml, encoding="utf-8")
    moveit_config = MoveItConfigsBuilder("tidybot", package_name="tidybot_moveit_config").to_moveit_configs()
    moveit_config.robot_description["use_sim_time"] = True

    kinematics_yaml_path = os.path.join(
        get_package_share_directory('tidybot_moveit_config'),
        'config',
        'kinematics.yaml'
    )

    with open(kinematics_yaml_path, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    phone_teleop_server = Node(
        package="tidybot_policy",
        executable="phone_teleop_server",
        name="phone_teleop_server",
        output="screen",
        parameters=[{"record": LaunchConfiguration("record")}],
    )

    phone_teleop = Node(
        package="tidybot_policy",
        executable="phone_policy",
        name="phone_policy",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim")},
                    {"use_sim": LaunchConfiguration("use_sim")},],
        remappings=[
            ("/tf", "/tf_relay"),
            ("/tf_static", "/tf_static_relay"),
        ],
    )

    state_controller = Node(
        package="tidybot_policy",
        executable="state_controller",
        name="state_controller",
        output="screen",
        parameters=[{"use_sim": LaunchConfiguration("use_sim")},
                    {"record": LaunchConfiguration("record")}],
    )
    
    # Optionally include the synchronized recorder when record=true
    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tidybot_episode'),
                'launch',
                'synchronized_recorder.launch.py',
            )
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'storage_uri': 'episode_bag',
            'fps': '10.0',
            'cameras': LaunchConfiguration('cameras'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('record')),
    )
    
    teleop_to_moveit = Node(
        package="tidybot_solver",
        executable="moveit_ee_pose_ik",
        name="moveit_ee_pose_ik",
        output="screen",
        parameters=[{"robot_description_kinematics": kinematics_config},
                    {"planning_frame": "arm_base_link"},
                    {"use_sim_time": LaunchConfiguration("use_sim")}],
        remappings=[("/tf", "/tf_relay"),
                    ("/tf_static", "/tf_static_relay")],
    )

    move_group_launch = generate_move_group_launch(moveit_config)

    return LaunchDescription([
        use_sim,
        record,
        cameras,
        phone_teleop_server,
        phone_teleop,
        state_controller,
        recorder_launch,
        teleop_to_moveit,
        move_group_launch,
    ])