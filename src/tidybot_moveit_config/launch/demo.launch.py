from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os, yaml

def generate_launch_description():
    launch_package_path = Path(get_package_share_directory("tidybot_moveit_config"))

    moveit_config_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_package_path / "launch/move_group.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_package_path / "launch/rsp.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_package_path / "launch/move_group.launch.py")
                )
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         str(launch_package_path / "launch/moveit_rviz.launch.py")
            #     )
            # )
        ]
    )

    return LaunchDescription([
        moveit_config_launch
    ])
