#!/usr/bin/env python3

"""
ArUco 标记检测节点：
订阅 Gemini 335 彩色图像，识别 4x4 ID 0~49 标记，
发布标记编号、四个角点和检测后的可视化图像。
"""

import json
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        # 声明输入、标注输出和检测数据输出的话题参数。
        # 这里使用参数而不是固定话题，方便以后切换相机或重命名话题。
        # image_topic：订阅的彩色图像话题，Gemini 335 默认发布到该话题。
        self.declare_parameter("image_topic", "/gemini335/color/image_raw")
        # camera_info_topic：与上述彩色图像对应的相机内参和畸变参数话题。
        self.declare_parameter(
            "camera_info_topic", "/gemini335/color/camera_info"
        )
        # annotated_topic：检测成功后发布带标记边框、编号和中心点的彩色图像。
        self.declare_parameter("annotated_topic", "/gemini335/detection/aruco/image")
        # detection_topic：发布 JSON 文本，包含帧信息、标记数量、编号和角点。
        self.declare_parameter("detection_topic", "/gemini335/detection/aruco")
        # dictionary_name：标记编码字典；生成标记和检测标记必须使用相同字典。
        # DICT_4X4_50 表示每个标记内部编码使用 4×4 方格，共包含 50 个编号。
        self.declare_parameter("dictionary_name", "DICT_4X4_50")
        # marker_length_m：打印后黑色标记外边缘的实际边长，单位为米。
        # 当前打印测量值为 48.5 mm，因此这里使用 0.0485 m。
        self.declare_parameter("marker_length_m", 0.0485)
        # marker_frame_prefix：各检测标记 TF 子坐标系的名称前缀。
        self.declare_parameter("marker_frame_prefix", "aruco_marker_")
        # pose_smoothing_alpha：位姿低通平滑系数，范围为 0~1。
        # 默认 0.7 让位姿较快跟随目标；值越接近 1 响应越快，但抖动也越明显。
        self.declare_parameter("pose_smoothing_alpha", 0.7)
        # pose_filter_reset_sec：标记丢失超过该时间后清除旧滤波状态。
        self.declare_parameter("pose_filter_reset_sec", 0.5)
        # show_windows：是否创建 OpenCV 本地窗口；远程或无桌面运行时可设为 false。
        self.declare_parameter("show_windows", True)
        # results_dir：按 s/f 保存成功或失败检测案例时使用的目标目录。
        self.declare_parameter(
            "results_dir",
            os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__), "..", "results", "images", "marker"
                )
            ),
        )

        # adaptive_thresh_constant：自适应阈值公式中的常量。
        # 值越大，判定为黑色像素的要求越严格；当前默认值通常可满足室内光照。
        self.declare_parameter("adaptive_thresh_constant", 7)
        # adaptive_thresh_win_size_min：自适应二值化使用的最小窗口边长（像素）。
        # 窗口越小可保留细小边缘，但容易受到局部噪声影响。
        self.declare_parameter("adaptive_thresh_win_size_min", 3)
        # adaptive_thresh_win_size_max：自适应二值化使用的最大窗口边长（像素）。
        # 从最小窗口逐步增加到最大窗口，可同时适配不同距离和大小的标记。
        self.declare_parameter("adaptive_thresh_win_size_max", 23)
        # adaptive_thresh_win_size_step：每次增加窗口尺寸的步长（像素）。
        # 较小步长尝试更多窗口，检测更稳但计算量更大。
        self.declare_parameter("adaptive_thresh_win_size_step", 10)
        # min_marker_perimeter_rate：允许标记周长相对于图像对角线周长的最小比例。
        # 0.03 可过滤画面中过小的候选标记，距离太远时可能漏检。
        self.declare_parameter("min_marker_perimeter_rate", 0.03)
        # max_marker_perimeter_rate：允许标记周长相对于图像对角线周长的最大比例。
        # 4.0 为较宽松上限，可避免标记靠近相机时被过滤。
        self.declare_parameter("max_marker_perimeter_rate", 4.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        annotated_topic = str(self.get_parameter("annotated_topic").value)
        detection_topic = str(self.get_parameter("detection_topic").value)
        self.show_windows = bool(self.get_parameter("show_windows").value)
        self.marker_length_m = float(
            self.get_parameter("marker_length_m").value
        )
        self.marker_frame_prefix = str(
            self.get_parameter("marker_frame_prefix").value
        )
        self.pose_smoothing_alpha = float(
            self.get_parameter("pose_smoothing_alpha").value
        )
        self.pose_filter_reset_sec = float(
            self.get_parameter("pose_filter_reset_sec").value
        )
        if self.marker_length_m <= 0:
            raise ValueError("marker_length_m 必须大于 0")
        if not 0.0 < self.pose_smoothing_alpha <= 1.0:
            raise ValueError("pose_smoothing_alpha 必须在 (0, 1] 范围内")
        if self.pose_filter_reset_sec <= 0.0:
            raise ValueError("pose_filter_reset_sec 必须大于 0")

        # 每个 ID 独立保存上一时刻的滤波后平移、姿态四元数和时间戳。
        self.pose_filter_state = {}

        # CameraInfo 是相机内参的来源；没有有效且分辨率匹配的内参时，
        # 仍输出编号和角点，但不会发布伪造或不可靠的三维位姿。
        self.camera_info = None
        self._camera_info_warning = None

        dictionary_name = str(self.get_parameter("dictionary_name").value)
        available_dictionaries = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        }
        if dictionary_name not in available_dictionaries:
            raise ValueError(
                "不支持的 ArUco 字典：%s" % dictionary_name
            )

        # getPredefinedDictionary 根据字典名称创建内置编码表。
        # 该对象包含每个 ID 对应的二进制编码，必须和生成打印标记时一致。
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            available_dictionaries[dictionary_name]
        )
        # DetectorParameters_create 创建 OpenCV 4.5.4 的旧版检测参数对象。
        # 后续逐项覆盖参数，是为了让节点的检测行为可在 ROS2 启动时配置。
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.adaptiveThreshConstant = float(
            self.get_parameter("adaptive_thresh_constant").value
        )
        self.parameters.adaptiveThreshWinSizeMin = int(
            self.get_parameter("adaptive_thresh_win_size_min").value
        )
        self.parameters.adaptiveThreshWinSizeMax = int(
            self.get_parameter("adaptive_thresh_win_size_max").value
        )
        self.parameters.adaptiveThreshWinSizeStep = int(
            self.get_parameter("adaptive_thresh_win_size_step").value
        )
        self.parameters.minMarkerPerimeterRate = float(
            self.get_parameter("min_marker_perimeter_rate").value
        )
        self.parameters.maxMarkerPerimeterRate = float(
            self.get_parameter("max_marker_perimeter_rate").value
        )

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.detection_pub = self.create_publisher(String, detection_topic, 10)
        # 广播相机光学坐标系到每个 ArUco 标记坐标系的实时变换。
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.show_windows:
            # 创建 OpenCV 可视化窗口；按 q/f/s 时分别退出、保存失败、保存成功案例。
            cv2.namedWindow("ArUco Marker Detection", cv2.WINDOW_NORMAL)

        self.get_logger().info(
            "订阅图像 %s 和相机内参 %s，使用 %s，检测结果发布到 %s"
            % (
                self.image_topic,
                self.camera_info_topic,
                dictionary_name,
                detection_topic,
            )
        )

    def camera_info_callback(self, msg):
        """保存最新一帧相机标定信息，供对应分辨率的彩色图像估计位姿。"""
        self.camera_info = msg

    def get_camera_calibration(self, image_width, image_height, image_frame_id):
        """返回有效且分辨率匹配的 OpenCV 相机矩阵和畸变系数。"""
        info = self.camera_info
        if info is None:
            self.warn_camera_info_once(
                "尚未收到相机内参；当前只输出标记编号和角点"
            )
            return None

        if info.width != image_width or info.height != image_height:
            self.warn_camera_info_once(
                "CameraInfo 分辨率 %dx%d 与图像 %dx%d 不一致；跳过位姿估计"
                % (info.width, info.height, image_width, image_height)
            )
            return None

        if (
            info.header.frame_id
            and image_frame_id
            and info.header.frame_id != image_frame_id
        ):
            self.warn_camera_info_once(
                "CameraInfo 坐标系 %s 与图像坐标系 %s 不一致；跳过位姿估计"
                % (info.header.frame_id, image_frame_id)
            )
            return None

        camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
            self.warn_camera_info_once(
                "CameraInfo 内参无效（fx/fy 必须大于 0）；跳过位姿估计"
            )
            return None

        distortion_model = info.distortion_model
        if distortion_model not in ("plumb_bob", "rational_polynomial", ""):
            self.warn_camera_info_once(
                "不支持畸变模型 %s；当前只支持 plumb_bob 或 "
                "rational_polynomial，跳过位姿估计" % distortion_model
            )
            return None

        distortion = np.asarray(info.d, dtype=np.float64)
        if distortion.size == 0:
            distortion = np.zeros(5, dtype=np.float64)
        elif not distortion_model:
            self.warn_camera_info_once(
                "CameraInfo 未声明畸变模型但提供了畸变参数；跳过位姿估计"
            )
            return None
        return camera_matrix, distortion

    def warn_camera_info_once(self, message):
        """同一种内参问题只警告一次，避免图像回调中持续刷屏。"""
        if self._camera_info_warning != message:
            self.get_logger().warning(message)
            self._camera_info_warning = message

    @staticmethod
    def rotation_matrix_to_quaternion(rotation):
        """将 OpenCV 旋转矩阵转换为 ROS 使用的 x/y/z/w 四元数。"""
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = np.sqrt(trace + 1.0) * 2.0
            w = 0.25 * scale
            x = (rotation[2, 1] - rotation[1, 2]) / scale
            y = (rotation[0, 2] - rotation[2, 0]) / scale
            z = (rotation[1, 0] - rotation[0, 1]) / scale
        elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif rotation[1, 1] > rotation[2, 2]:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale

        return (float(x), float(y), float(z), float(w))

    @staticmethod
    def slerp_quaternion(previous, current, alpha):
        """沿最短旋转路径对两个单位四元数做球面线性插值。"""
        previous = np.asarray(previous, dtype=np.float64)
        current = np.asarray(current, dtype=np.float64)
        dot = float(np.dot(previous, current))

        # q 和 -q 表示相同旋转；翻转符号可确保沿较短路径插值。
        if dot < 0.0:
            current = -current
            dot = -dot
        dot = float(np.clip(dot, -1.0, 1.0))

        if dot > 0.9995:
            result = previous + alpha * (current - previous)
            result /= np.linalg.norm(result)
            return tuple(float(value) for value in result)

        angle = np.arccos(dot)
        sin_angle = np.sin(angle)
        result = (
            np.sin((1.0 - alpha) * angle) / sin_angle * previous
            + np.sin(alpha * angle) / sin_angle * current
        )
        result /= np.linalg.norm(result)
        return tuple(float(value) for value in result)

    @staticmethod
    def quaternion_to_rotation_matrix(quaternion):
        """将 x/y/z/w 顺序的单位四元数转换为 3×3 旋转矩阵。"""
        x, y, z, w = quaternion
        return np.array(
            [
                [
                    1 - 2 * (y * y + z * z),
                    2 * (x * y - z * w),
                    2 * (x * z + y * w),
                ],
                [
                    2 * (x * y + z * w),
                    1 - 2 * (x * x + z * z),
                    2 * (y * z - x * w),
                ],
                [
                    2 * (x * z - y * w),
                    2 * (y * z + x * w),
                    1 - 2 * (x * x + y * y),
                ],
            ],
            dtype=np.float64,
        )

    def smooth_pose(self, marker_id, translation, quaternion, timestamp):
        """按标记 ID 独立平滑位姿；首次出现或中断后以当前测量重新初始化。"""
        state = self.pose_filter_state.get(marker_id)
        alpha = self.pose_smoothing_alpha

        if (
            state is None
            or timestamp <= state["timestamp"]
            or timestamp - state["timestamp"] > self.pose_filter_reset_sec
        ):
            filtered_translation = np.asarray(translation, dtype=np.float64)
            filtered_quaternion = tuple(float(value) for value in quaternion)
        else:
            filtered_translation = (
                (1.0 - alpha) * state["translation"]
                + alpha * np.asarray(translation, dtype=np.float64)
            )
            filtered_quaternion = self.slerp_quaternion(
                state["quaternion"], quaternion, alpha
            )

        self.pose_filter_state[marker_id] = {
            "translation": filtered_translation,
            "quaternion": filtered_quaternion,
            "timestamp": timestamp,
        }
        return filtered_translation, filtered_quaternion

    def image_callback(self, msg):
        try:
            # 将 ROS2 sensor_msgs/Image 转换为 OpenCV 的 BGR 图像。
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as exc:
            self.get_logger().error("无法转换图像：%s" % exc)
            return

        # ArUco 方格检测通常只需要灰度图，可减少颜色噪声影响。
        # 它首先转换为单通道亮度图，再交给自适应阈值和轮廓算法处理。
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            # detectMarkers 返回标记四角、编号，以及未被识别为标记的候选框。
            # corners：每个标记的四角像素坐标；
            # ids：与 corners 一一对应的标记编号；
            # rejected：看起来像标记轮廓但编码校验失败的候选框。
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters,
            )
        except cv2.error as exc:
            # 图像异常时只记录错误并跳过这一帧，不让整个 ROS2 节点退出。
            self.get_logger().error("ArUco 检测失败：%s" % exc)
            return

        calibration = self.get_camera_calibration(
            frame.shape[1], frame.shape[0], msg.header.frame_id
        )
        pose_data = None
        if ids is not None and calibration is not None:
            camera_matrix, distortion = calibration
            try:
                # 使用标记实际边长、相机内参、镜头畸变和检测角点估计位姿。
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.marker_length_m,
                    camera_matrix,
                    distortion,
                )
                pose_data = []
                timestamp = (
                    msg.header.stamp.sec
                    + msg.header.stamp.nanosec / 1e9
                )
                for index, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                    raw_rotation_matrix, _ = cv2.Rodrigues(rvec)
                    raw_translation = tvec.reshape(3)
                    raw_quaternion = self.rotation_matrix_to_quaternion(
                        raw_rotation_matrix
                    )
                    marker_id = int(ids[index, 0])
                    translation, quaternion = self.smooth_pose(
                        marker_id,
                        raw_translation,
                        raw_quaternion,
                        timestamp,
                    )
                    rotation_matrix = self.quaternion_to_rotation_matrix(
                        quaternion
                    )
                    filtered_rvec, _ = cv2.Rodrigues(rotation_matrix)

                    parent_frame = (
                        msg.header.frame_id
                        or self.camera_info.header.frame_id
                    )
                    if parent_frame:
                        # TF parent 是相机光学坐标系，child 是以标记中心为原点的目标坐标系。
                        transform = TransformStamped()
                        transform.header.stamp = msg.header.stamp
                        transform.header.frame_id = parent_frame
                        transform.child_frame_id = (
                            "%s%d"
                            % (
                                self.marker_frame_prefix,
                                marker_id,
                            )
                        )
                        transform.transform.translation.x = float(translation[0])
                        transform.transform.translation.y = float(translation[1])
                        transform.transform.translation.z = float(translation[2])
                        (
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w,
                        ) = quaternion
                        self.tf_broadcaster.sendTransform(transform)
                    else:
                        self.warn_camera_info_once(
                            "图像和 CameraInfo 都没有 frame_id；无法广播 TF"
                        )

                    # ROS optical frame 与 OpenCV 相机坐标一致：
                    # X 向图像右方、Y 向图像下方、Z 向相机前方，单位为米。
                    pose_data.append(
                        {
                            # 这些字段和 TF 都使用滤波后的值。
                            "translation_m": [
                                round(float(value), 5)
                                for value in translation
                            ],
                            "rotation_vector_rad": [
                                round(float(value), 5)
                                for value in filtered_rvec.reshape(3)
                            ],
                            "rotation_matrix": [
                                [round(float(value), 5) for value in row]
                                for row in rotation_matrix
                            ],
                            "quaternion_xyzw": [
                                round(value, 6) for value in quaternion
                            ],
                            "distance_m": round(
                                float(np.linalg.norm(translation)), 5
                            ),
                        }
                    )

                    # 在图像上绘制以标记边长一半为长度的 XYZ 坐标轴。
                    cv2.drawFrameAxes(
                        frame,
                        camera_matrix,
                        distortion,
                        filtered_rvec,
                        translation.reshape(1, 1, 3),
                        self.marker_length_m * 0.5,
                    )
            except cv2.error as exc:
                self.get_logger().error("ArUco 位姿估计失败：%s" % exc)
                pose_data = None

        detections = []
        if ids is not None:
            # 在彩色原图上画出标记外边框和角点。
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for index, (marker_id, marker_corners) in enumerate(
                zip(ids.flatten(), corners)
            ):
                # 每个标记有四个角点，按坐标顺序通常为：
                # 左上、右上、右下、左下。
                marker_corners = marker_corners.reshape(4, 2)
                points = [
                    # 保留两位小数，减少 JSON 内容长度，同时保持足够的像素精度。
                    [round(float(point[0]), 2), round(float(point[1]), 2)]
                    for point in marker_corners
                ]

                # 标记中心采用四角 x、y 坐标的算术平均值。
                # 相比只取外接框中心，它与实际四角形状更一致。
                center_u = round(float(marker_corners[:, 0].mean()), 2)
                center_v = round(float(marker_corners[:, 1].mean()), 2)
                detections.append(
                    {
                        # class_name 用于和颜色分割、YOLO 等统一检测接口兼容。
                        "class_name": "aruco",
                        # id 是标记字典中的编号，例如贴在罐子上的编号 0~5。
                        "id": int(marker_id),
                        # corners 是图像像素坐标，不是相机三维坐标。
                        "corners": points,
                        # center 是标记在彩色图像上的中心像素 [u, v]。
                        "center": [center_u, center_v],
                        # pose 在内参有效且与图像分辨率匹配时才存在。
                        "pose": (
                            pose_data[index]
                            if pose_data is not None
                            and index < len(pose_data)
                            else None
                        ),
                    }
                )

                if pose_data is not None and index < len(pose_data):
                    position = pose_data[index]["translation_m"]
                    pose_label = "X:%.2f Y:%.2f Z:%.2fm" % tuple(position)
                    cv2.putText(
                        frame,
                        pose_label,
                        (
                            int(points[0][0]),
                            min(frame.shape[0] - 8, int(points[0][1] + 24)),
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        2,
                    )

                cv2.putText(
                    frame,
                    "ID %d" % int(marker_id),
                    (int(points[0][0]), max(20, int(points[0][1] - 8))),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                cv2.circle(
                    frame,
                    (int(center_u), int(center_v)),
                    5,
                    (0, 0, 255),
                    -1,
                )

        payload = {
            # 使用 JSON 发布结构化检测结果，便于后续节点和外部程序读取。
            # frame_id：彩色图像对应的相机坐标系或 TF 坐标系名称。
            "frame_id": msg.header.frame_id,
            "pose_frame_id": (
                self.camera_info.header.frame_id
                if self.camera_info is not None
                else None
            ),
            # image_size：彩色图像宽度和高度，单位像素，格式为 [width, height]。
            "image_size": [frame.shape[1], frame.shape[0]],
            # timestamp：图像消息头中的秒级时间戳，可用于同步深度图或跟踪。
            "timestamp": (
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            ),
            # dictionary：本次检测使用的 ArUco 编码字典名称。
            "dictionary": str(self.get_parameter("dictionary_name").value),
            "marker_length_m": self.marker_length_m,
            "camera_info_valid": calibration is not None,
            "pose_valid": pose_data is not None,
            # marker_count：当前帧成功识别的标记数量。
            "marker_count": len(detections),
            # detections：成功标记数组，每个元素包含 ID、角点和中心。
            "detections": detections,
        }
        self.detection_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        if self.show_windows:
            cv2.imshow("ArUco Marker Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                rclpy.shutdown()
            elif key in (ord("s"), ord("f")):
                case = "success" if key == ord("s") else "failure"
                output_dir = str(self.get_parameter("results_dir").value)
                os.makedirs(output_dir, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                output_path = os.path.join(
                    output_dir, "%s_%s.jpg" % (stamp, case)
                )
                cv2.imwrite(output_path, frame)
                self.get_logger().info(
                    "保存 %s 案例到 %s" % (case, output_path)
                )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
