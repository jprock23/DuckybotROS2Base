from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description()->LaunchDescription:
    
    localizer_launch_dir = PathJoinSubstitution([FindPackageShare('localizer'), 'launch'])
    camera_launch_dir = PathJoinSubstitution([FindPackageShare('camera'), 'launch'])
    
    map_node = Node(
        package='map_node',
        executable='map_gen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([localizer_launch_dir, "localizer.launch.py"])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([camera_launch_dir, "camera.launch.py"])
        ),
        map_node
    ])