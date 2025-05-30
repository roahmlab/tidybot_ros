from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ros_gz_sim_pkg_path = FindPackageShare('ros_gz_sim')
    sim_pkg_path = FindPackageShare('tidybot_description')  
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([sim_pkg_path, 'urdf'])
    ))
    ld.add_action(SetEnvironmentVariable(
        'GZ_SIM_PLUGIN_PATH',
        PathJoinSubstitution([sim_pkg_path, 'plugins'])
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'on_exit_shutdown': 'True'
        }.items(),
    ))
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([sim_pkg_path, 'launch', 'display.launch.py']),
    ))
    ld.add_action(Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{
            'name': 'tidybot',
            'topic': 'robot_description',
        }],
        output='screen',
    ))

    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            '/world/empty/model/tidybot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/rrbot/joint_state', 'joint_states'),
        ],
        output='screen'
    ))

    return ld