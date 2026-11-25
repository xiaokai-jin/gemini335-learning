from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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
            DeclareLaunchArgument("enable_publish_extrinsic", default_value="true"),

            # Optional motion sensors.
            DeclareLaunchArgument("enable_accel", default_value="false"),
            DeclareLaunchArgument("enable_gyro", default_value="false"),
            DeclareLaunchArgument("enable_green_jar_detection", default_value="false"),
            DeclareLaunchArgument("green_jar_minimum_area", default_value="500"),
            DeclareLaunchArgument("green_jar_lower_h", default_value="35"),
            DeclareLaunchArgument("green_jar_lower_s", default_value="40"),
            DeclareLaunchArgument("green_jar_lower_v", default_value="20"),
            DeclareLaunchArgument("green_jar_upper_h", default_value="85"),
            DeclareLaunchArgument("green_jar_upper_s", default_value="255"),
            DeclareLaunchArgument("green_jar_upper_v", default_value="255"),
            DeclareLaunchArgument("green_jar_gamma", default_value="70"),
            DeclareLaunchArgument("green_jar_clahe_clip", default_value="20"),
            DeclareLaunchArgument(
                "green_jar_results_dir",
                default_value="/home/xiaokai/gemini335-learning/results/images",
            ),
            DeclareLaunchArgument(
                "green_jar_image_topic",
                default_value="/gemini335/color/image_raw",
            ),

            # ArUco 节点开关及参数：
            # - enable_aruco_detection：true 时启动检测节点，false 时不启动，
            #   可避免在只需要相机或绿色颜色分割时额外占用计算资源；
            # - aruco_image_topic：ArUco 节点订阅的彩色图像话题；
            # - aruco_dictionary：必须与生成和打印标记时使用的字典一致；
            # - aruco_results_dir：按 s/f 保存检测案例图的目标目录；
            # - aruco_show_windows=false：关闭本地 OpenCV 窗口，适合远程或无桌面运行，
            #   节点仍继续发布标注图像和 JSON 检测结果。
            DeclareLaunchArgument("enable_aruco_detection", default_value="false"),
            DeclareLaunchArgument(
                "aruco_image_topic",
                default_value="/gemini335/color/image_raw",
            ),
            DeclareLaunchArgument(
                "aruco_camera_info_topic",
                default_value="/gemini335/color/camera_info",
            ),
            DeclareLaunchArgument(
                "aruco_dictionary",
                default_value="DICT_4X4_50",
            ),
            DeclareLaunchArgument("aruco_marker_length_m", default_value="0.0485"),
            DeclareLaunchArgument("aruco_pose_smoothing_alpha", default_value="0.7"),
            DeclareLaunchArgument("aruco_pose_filter_reset_sec", default_value="0.5"),
            DeclareLaunchArgument(
                "aruco_results_dir",
                default_value="/home/xiaokai/gemini335-learning/results/images/marker",
            ),
            DeclareLaunchArgument("aruco_show_windows", default_value="true"),
            DeclareLaunchArgument("enable_aruco_rviz", default_value="false"),
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
                        "enable_publish_extrinsic": LaunchConfiguration(
                            "enable_publish_extrinsic"
                        ),

                        # Motion sensors and synchronized capture.
                        "enable_accel": LaunchConfiguration("enable_accel"),
                        "enable_gyro": LaunchConfiguration("enable_gyro"),
                        "enable_frame_sync": True,
                    }
                ],
            ),
            Node(
                package="gemini335_bringup",
                executable="object_detection_color.py",
                name="green_jar_detector",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("enable_green_jar_detection")
                ),
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("green_jar_image_topic"),
                        "minimum_area": LaunchConfiguration("green_jar_minimum_area"),
                        "lower_h": LaunchConfiguration("green_jar_lower_h"),
                        "lower_s": LaunchConfiguration("green_jar_lower_s"),
                        "lower_v": LaunchConfiguration("green_jar_lower_v"),
                        "upper_h": LaunchConfiguration("green_jar_upper_h"),
                        "upper_s": LaunchConfiguration("green_jar_upper_s"),
                        "upper_v": LaunchConfiguration("green_jar_upper_v"),
                        "low_light_gamma": LaunchConfiguration("green_jar_gamma"),
                        "clahe_clip": LaunchConfiguration("green_jar_clahe_clip"),
                        "results_dir": LaunchConfiguration("green_jar_results_dir"),
                        "show_windows": True,
                    }
                ],
            ),
            Node(
                package="gemini335_bringup",
                executable="object_detection_aruco.py",
                name="aruco_detector",
                output="screen",
                # 只有明确启用时才启动 ArUco 节点，不影响相机和绿色分割节点。
                condition=IfCondition(
                    LaunchConfiguration("enable_aruco_detection")
                ),
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("aruco_image_topic"),
                        "camera_info_topic": LaunchConfiguration(
                            "aruco_camera_info_topic"
                        ),
                        "dictionary_name": LaunchConfiguration("aruco_dictionary"),
                        "marker_length_m": LaunchConfiguration(
                            "aruco_marker_length_m"
                        ),
                        "pose_smoothing_alpha": LaunchConfiguration(
                            "aruco_pose_smoothing_alpha"
                        ),
                        "pose_filter_reset_sec": LaunchConfiguration(
                            "aruco_pose_filter_reset_sec"
                        ),
                        "results_dir": LaunchConfiguration("aruco_results_dir"),
                        "show_windows": LaunchConfiguration("aruco_show_windows"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="aruco_rviz",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("enable_aruco_rviz")
                ),
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("gemini335_bringup"),
                            "rviz",
                            "aruco.rviz",
                        ]
                    ),
                ],
            ),
        ]
    )
