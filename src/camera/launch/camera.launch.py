from launch.actions import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description for the camera node.

    Returns
    -------
        LaunchDescription: the launch description

    """
    # parameters
    camera_param_name = "camera"
    camera_param_default = str(0)
    camera_param = LaunchConfiguration(
        camera_param_name,
        default=camera_param_default,
    )
    camera_launch_arg = DeclareLaunchArgument(
        camera_param_name,
        default_value=camera_param_default,
        description="camera ID or name"
    )

    format_param_name = "format"
    format_param_default = "YUYV"
    format_param = LaunchConfiguration(
        format_param_name,
        default=format_param_default,
    )
    format_launch_arg = DeclareLaunchArgument(
        format_param_name,
        default_value=format_param_default,
        description="pixel format"
    )

    calibration_param_name = "camera_info_url"
    calibration_param_default = "package://camera/config/calibration/ost.yaml"
    calibration_param = LaunchConfiguration(
        calibration_param_name,
        default=calibration_param_default,
    )
    calibration_launch_arg = DeclareLaunchArgument(
        calibration_param_name,
        default_value=calibration_param_default,
        description="calibration file"
    )
    
    frame_id_param_name = "frame_id"
    frame_id_param_default = "ceil_camera"
    frame_id_param = LaunchConfiguration(
        frame_id_param_name,
        default=frame_id_param_default,
    )
    frame_id_launch_arg = DeclareLaunchArgument(
        frame_id_param_name,
        default_value=frame_id_param_default,
        description="Camera's frame id"
    )    

    # camera node
    camera_node = Node(
            package='camera_ros',
            executable="camera_node",
            parameters=[{
                "camera": camera_param,
                "width": 640,
                "height": 480,
                "format": format_param,
                "camera_info_url": calibration_param,
                "frame_id": frame_id_param
            }]
        )

    return LaunchDescription([
        # container,
        camera_launch_arg,
        format_launch_arg,
        calibration_launch_arg,
        frame_id_launch_arg,
        camera_node,
    ])

