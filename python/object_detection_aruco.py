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
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        # 声明输入、标注输出和检测数据输出的话题参数。
        # 这里使用参数而不是固定话题，方便以后切换相机或重命名话题。
        # image_topic：订阅的彩色图像话题，Gemini 335 默认发布到该话题。
        self.declare_parameter("image_topic", "/gemini335/color/image_raw")
        # annotated_topic：检测成功后发布带标记边框、编号和中心点的彩色图像。
        self.declare_parameter("annotated_topic", "/gemini335/detection/aruco/image")
        # detection_topic：发布 JSON 文本，包含帧信息、标记数量、编号和角点。
        self.declare_parameter("detection_topic", "/gemini335/detection/aruco")
        # dictionary_name：标记编码字典；生成标记和检测标记必须使用相同字典。
        # DICT_4X4_50 表示每个标记内部编码使用 4×4 方格，共包含 50 个编号。
        self.declare_parameter("dictionary_name", "DICT_4X4_50")
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
        annotated_topic = str(self.get_parameter("annotated_topic").value)
        detection_topic = str(self.get_parameter("detection_topic").value)
        self.show_windows = bool(self.get_parameter("show_windows").value)

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
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.detection_pub = self.create_publisher(String, detection_topic, 10)

        if self.show_windows:
            # 创建 OpenCV 可视化窗口；按 q/f/s 时分别退出、保存失败、保存成功案例。
            cv2.namedWindow("ArUco Marker Detection", cv2.WINDOW_NORMAL)

        self.get_logger().info(
            "订阅 %s，使用 %s，检测结果发布到 %s"
            % (self.image_topic, dictionary_name, detection_topic)
        )

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
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters,
            )
        except cv2.error as exc:
            # 图像异常时只记录错误并跳过这一帧，不让整个 ROS2 节点退出。
            self.get_logger().error("ArUco 检测失败：%s" % exc)
            return

        detections = []
        if ids is not None:
            # 在彩色原图上画出标记外边框和角点。
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for marker_id, marker_corners in zip(ids.flatten(), corners):
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
                    }
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
            # image_size：彩色图像宽度和高度，单位像素，格式为 [width, height]。
            "image_size": [frame.shape[1], frame.shape[0]],
            # timestamp：图像消息头中的秒级时间戳，可用于同步深度图或跟踪。
            "timestamp": (
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            ),
            # dictionary：本次检测使用的 ArUco 编码字典名称。
            "dictionary": str(self.get_parameter("dictionary_name").value),
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
