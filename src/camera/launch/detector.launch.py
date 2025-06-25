from camera import tag_detector_node
from launch.actions import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description()->LaunchDescription:
    """
    Generate a launch description for the tag_detector node.

    Returns
    -------
        LaunchDescription: the launch description

    """
        
    tag_detector_node = Node(
            package='camera',
            executable='tag_detector',
            parameters=[{
                'flip': True,
            }]
    )

    return LaunchDescription([
        tag_detector_node
    ])