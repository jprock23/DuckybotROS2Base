from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # loading urdf
    pkg_path = FindPackageShare(package='deliberation').find('deliberation')
    urdf_path = os.path.join(pkg_path, "duckiebot.urdf")
    with open(urdf_path) as file:
        urdf = file.read()
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
            arguments=[urdf]),
        # Node(
        #      package='deliberation',
        #      executable='state_publisher',
        #      name='state_publisher',
        #      output='screen'),
    ])