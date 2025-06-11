from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
<<<<<<< HEAD


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("tidybot", package_name="tidybot_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
=======
from launch import LaunchDescription
from launch_ros.actions import Node

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
        generate_demo_launch(moveit_config)
    ])
>>>>>>> master
