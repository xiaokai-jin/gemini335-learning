from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="gemini335"),
            DeclareLaunchArgument("serial_number", default_value=""),
            DeclareLaunchArgument("enable_color", default_value="true"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            Node(
                package="orbbec_camera",
                executable="orbbec_camera_node",
                name="orbbec_camera",
                namespace=LaunchConfiguration("camera_name"),
                output="screen",
                parameters=[
                    {
                        "device_type": "camera",
                        "camera_name": LaunchConfiguration("camera_name"),
                        "serial_number": LaunchConfiguration("serial_number"),
                        "enable_color": LaunchConfiguration("enable_color"),
                        "color_width": 1280,
                        "color_height": 720,
                        "color_fps": 30,
                        "color_format": "ANY",
                        "enable_depth": LaunchConfiguration("enable_depth"),
                        "depth_registration": False,
                        "enable_point_cloud": False,
                        "enable_accel": False,
                        "enable_gyro": False,
                        "enable_frame_sync": True,
                    }
                ],
            ),
        ]
    )
