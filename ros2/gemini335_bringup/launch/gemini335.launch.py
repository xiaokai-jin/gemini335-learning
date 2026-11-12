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
            DeclareLaunchArgument("enable_point_cloud", default_value="true"),
            DeclareLaunchArgument("color_width", default_value="1280"),
            DeclareLaunchArgument("color_height", default_value="720"),
            DeclareLaunchArgument("color_fps", default_value="60"),
            DeclareLaunchArgument("depth_width", default_value="1280"),
            DeclareLaunchArgument("depth_height", default_value="800"),
            DeclareLaunchArgument("depth_fps", default_value="30"),
            DeclareLaunchArgument("enable_accel", default_value="false"),
            DeclareLaunchArgument("enable_gyro", default_value="false"),
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
                        "color_width": LaunchConfiguration("color_width"),
                        "color_height": LaunchConfiguration("color_height"),
                        "color_fps": LaunchConfiguration("color_fps"),
                        "color_format": "ANY",
                        "enable_depth": LaunchConfiguration("enable_depth"),
                        "depth_width": LaunchConfiguration("depth_width"),
                        "depth_height": LaunchConfiguration("depth_height"),
                        "depth_fps": LaunchConfiguration("depth_fps"),
                        "depth_registration": False,
                        "enable_point_cloud": LaunchConfiguration("enable_point_cloud"),
                        "enable_accel": LaunchConfiguration("enable_accel"),
                        "enable_gyro": LaunchConfiguration("enable_gyro"),
                        "enable_frame_sync": True,
                    }
                ],
            ),
        ]
    )
