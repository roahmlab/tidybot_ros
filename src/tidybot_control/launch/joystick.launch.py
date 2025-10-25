from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils.launches import generate_move_group_launch

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_dir = get_package_share_directory('tidybot_control')
    description_pkg_dir = get_package_share_directory('tidybot_description')
    solver_pkg_dir = get_package_share_directory('tidybot_solver')
    joy_params = os.path.join(package_dir, 'config', 'joystick.yaml')

    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='Use simulation (Gazebo) if true, hardware if false'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params],
    )

    joystick_controller_node = Node(
        package='tidybot_control',
        executable='joystick_controller',
        name='joystick_controller',
        parameters=[{'use_sim': LaunchConfiguration('use_sim')},
                    {'use_sim_time': LaunchConfiguration('use_sim')}],
    )

    # Use our custom joystick_to_moveit node with C++ Servo API
    moveit_servo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                solver_pkg_dir,
                'launch',
                'moveit_twist.launch.py',
            )
        ),
        launch_arguments={'command_output_type': 'joint_trajectory'}.items(),
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    moveit_servo_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                solver_pkg_dir,
                'launch',
                'moveit_twist.launch.py',
            )
        ),
        launch_arguments={'command_output_type': 'joint_state'}.items(),
        condition=UnlessCondition(LaunchConfiguration('use_sim'))
    )

    return LaunchDescription([
        use_sim_arg,
        joy_node,
        joystick_controller_node,
        moveit_servo_sim,
        moveit_servo_hardware,
    ])