from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("tidybot", package_name="tidybot_moveit_config")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),  
            {"use_sim_time": True}    
        ],
    )

    return LaunchDescription([
        move_group_node,
        TimerAction(
            period=3.0, 
            actions=[generate_demo_launch(moveit_config)]
        )
    ])
