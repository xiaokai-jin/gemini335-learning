from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Device selection and ROS namespace.
            DeclareLaunchArgument("camera_name", default_value="gemini335"),
            DeclareLaunchArgument("serial_number", default_value=""),

            # Color and depth streams.
            DeclareLaunchArgument("enable_color", default_value="true"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            DeclareLaunchArgument("color_width", default_value="1280"),
            DeclareLaunchArgument("color_height", default_value="720"),
            DeclareLaunchArgument("color_fps", default_value="60"),
            # The Gemini 335 requires depth and both IR streams to share a
            # resolution and frame rate when the IR streams are enabled.
            DeclareLaunchArgument("depth_width", default_value="1280"),
            DeclareLaunchArgument("depth_height", default_value="800"),
            DeclareLaunchArgument("depth_fps", default_value="30"),
            DeclareLaunchArgument("enable_left_ir", default_value="true"),
            DeclareLaunchArgument("enable_right_ir", default_value="true"),
            DeclareLaunchArgument("left_ir_width", default_value="1280"),
            DeclareLaunchArgument("left_ir_height", default_value="800"),
            DeclareLaunchArgument("left_ir_fps", default_value="30"),
            DeclareLaunchArgument("right_ir_width", default_value="1280"),
            DeclareLaunchArgument("right_ir_height", default_value="800"),
            DeclareLaunchArgument("right_ir_fps", default_value="30"),

            # Point cloud output. Colored point clouds use software alignment
            # for the current Gemini 335 color/depth profile.
            DeclareLaunchArgument("enable_point_cloud", default_value="true"),
            DeclareLaunchArgument("enable_colored_point_cloud", default_value="true"),
            DeclareLaunchArgument("point_cloud_decimation_filter_factor", default_value="4"),
            DeclareLaunchArgument("point_cloud_qos", default_value="default"),
            DeclareLaunchArgument("cloud_frame_id", default_value=""),
            DeclareLaunchArgument("ordered_pc", default_value="false"),
            DeclareLaunchArgument("depth_registration", default_value="true"),
            DeclareLaunchArgument("align_mode", default_value="SW"),

            # Optional motion sensors.
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
                        # Device parameters.
                        "device_type": "camera",
                        "camera_name": LaunchConfiguration("camera_name"),
                        "serial_number": LaunchConfiguration("serial_number"),

                        # Color stream parameters.
                        "enable_color": LaunchConfiguration("enable_color"),
                        "color_width": LaunchConfiguration("color_width"),
                        "color_height": LaunchConfiguration("color_height"),
                        "color_fps": LaunchConfiguration("color_fps"),
                        "color_format": "ANY",

                        # Depth stream and depth-to-color alignment.
                        "enable_depth": LaunchConfiguration("enable_depth"),
                        "depth_width": LaunchConfiguration("depth_width"),
                        "depth_height": LaunchConfiguration("depth_height"),
                        "depth_fps": LaunchConfiguration("depth_fps"),
                        "depth_registration": LaunchConfiguration("depth_registration"),
                        "align_mode": LaunchConfiguration("align_mode"),

                        # Left and right infrared streams.
                        "enable_left_ir": LaunchConfiguration("enable_left_ir"),
                        "left_ir_width": LaunchConfiguration("left_ir_width"),
                        "left_ir_height": LaunchConfiguration("left_ir_height"),
                        "left_ir_fps": LaunchConfiguration("left_ir_fps"),
                        "left_ir_format": "ANY",
                        "enable_right_ir": LaunchConfiguration("enable_right_ir"),
                        "right_ir_width": LaunchConfiguration("right_ir_width"),
                        "right_ir_height": LaunchConfiguration("right_ir_height"),
                        "right_ir_fps": LaunchConfiguration("right_ir_fps"),
                        "right_ir_format": "ANY",

                        # Depth and colored point cloud output.
                        "enable_point_cloud": LaunchConfiguration("enable_point_cloud"),
                        "enable_colored_point_cloud": LaunchConfiguration(
                            "enable_colored_point_cloud"
                        ),
                        "point_cloud_decimation_filter_factor": LaunchConfiguration(
                            "point_cloud_decimation_filter_factor"
                        ),
                        "point_cloud_qos": LaunchConfiguration("point_cloud_qos"),
                        "cloud_frame_id": LaunchConfiguration("cloud_frame_id"),
                        "ordered_pc": LaunchConfiguration("ordered_pc"),

                        # Motion sensors and synchronized capture.
                        "enable_accel": LaunchConfiguration("enable_accel"),
                        "enable_gyro": LaunchConfiguration("enable_gyro"),
                        "enable_frame_sync": True,
                    }
                ],
            ),
        ]
    )
