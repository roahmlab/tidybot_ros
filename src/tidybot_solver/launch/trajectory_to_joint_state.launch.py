from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tidybot_solver',
            executable='trajectory_to_joint_state',
            name='trajectory_to_joint_state',
            output='screen',
            parameters=[],
            remappings=[]
        )
    ])
