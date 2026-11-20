#!/usr/bin/env python3

"""
绿色罐子颜色分割节点：
订阅 Gemini 335 的彩色图像，经暗光增强、HSV 分割和轮廓筛选后，
发布标注图像以及包含目标框、中心和面积的 JSON 检测结果。
"""

import json
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class GreenObjectDetector(Node):
    def __init__(self):
        super().__init__("green_object_detector")

        # ROS2 话题和检测参数可在启动时通过 --ros-args 统一配置。
        self.declare_parameter("image_topic", "/gemini335/color/image_raw")
        self.declare_parameter("annotated_topic", "/gemini335/detection/image")
        self.declare_parameter("detection_topic", "/gemini335/detection/green_jar")
        self.declare_parameter("lower_h", 35)
        self.declare_parameter("lower_s", 40)
        self.declare_parameter("lower_v", 20)
        self.declare_parameter("upper_h", 85)
        self.declare_parameter("upper_s", 255)
        self.declare_parameter("upper_v", 255)
        self.declare_parameter("minimum_area", 500)
        self.declare_parameter("kernel_size", 5)
        self.declare_parameter("max_detections", 1)
        self.declare_parameter("show_windows", True)
        self.declare_parameter("low_light_gamma", 70)
        self.declare_parameter("clahe_clip", 20)
        self.declare_parameter("clahe_grid", 8)
        self.declare_parameter(
            "results_dir",
            os.path.abspath(
                os.path.join(os.path.dirname(__file__), "..", "results", "images")
            ),
        )

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.minimum_area = int(self.get_parameter("minimum_area").value)
        self.kernel_size = int(self.get_parameter("kernel_size").value)
        self.max_detections = int(self.get_parameter("max_detections").value)
        self.show_windows = bool(self.get_parameter("show_windows").value)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        annotated_topic = str(self.get_parameter("annotated_topic").value)
        detection_topic = str(self.get_parameter("detection_topic").value)
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.detection_pub = self.create_publisher(String, detection_topic, 10)

        if self.show_windows:
            # 创建同一个检测窗口及滑条，实时调节 HSV 范围和暗光增强参数。
            cv2.namedWindow("Green Jar Detection", cv2.WINDOW_NORMAL)
            cv2.createTrackbar(
                "H min", "Green Jar Detection",
                int(self.get_parameter("lower_h").value), 179,
                lambda value: None,
            )
            cv2.createTrackbar(
                "S min", "Green Jar Detection",
                int(self.get_parameter("lower_s").value), 255,
                lambda value: None,
            )
            cv2.createTrackbar(
                "V min", "Green Jar Detection",
                int(self.get_parameter("lower_v").value), 255,
                lambda value: None,
            )
            cv2.createTrackbar(
                "H max", "Green Jar Detection",
                int(self.get_parameter("upper_h").value), 179,
                lambda value: None,
            )
            cv2.createTrackbar(
                "S max", "Green Jar Detection",
                int(self.get_parameter("upper_s").value), 255,
                lambda value: None,
            )
            cv2.createTrackbar(
                "V max", "Green Jar Detection",
                int(self.get_parameter("upper_v").value), 255,
                lambda value: None,
            )
            cv2.createTrackbar(
                "Gamma x100", "Green Jar Detection",
                int(self.get_parameter("low_light_gamma").value), 200,
                lambda value: None,
            )
            cv2.createTrackbar(
                "CLAHE clip x10", "Green Jar Detection",
                int(self.get_parameter("clahe_clip").value), 100,
                lambda value: None,
            )

        self.get_logger().info(
            "Subscribed to %s; publishing detections to %s"
            % (self.image_topic, detection_topic)
        )

    def get_lower_upper(self):
        # 无图形窗口时使用启动参数；有窗口时优先读取用户拖动后的滑条值。
        if not self.show_windows:
            return (
                (
                    int(self.get_parameter("lower_h").value),
                    int(self.get_parameter("lower_s").value),
                    int(self.get_parameter("lower_v").value),
                ),
                (
                    int(self.get_parameter("upper_h").value),
                    int(self.get_parameter("upper_s").value),
                    int(self.get_parameter("upper_v").value),
                ),
            )
        return (
            (
                cv2.getTrackbarPos("H min", "Green Jar Detection"),
                cv2.getTrackbarPos("S min", "Green Jar Detection"),
                cv2.getTrackbarPos("V min", "Green Jar Detection"),
            ),
            (
                cv2.getTrackbarPos("H max", "Green Jar Detection"),
                cv2.getTrackbarPos("S max", "Green Jar Detection"),
                cv2.getTrackbarPos("V max", "Green Jar Detection"),
            ),
        )

    def image_callback(self, msg):
        try:
            # CvBridge 将 ROS2 图像消息转换为 OpenCV 使用的 BGR 图像。
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error("Could not convert image: %s" % exc)
            return

        lower, upper = self.get_lower_upper()

        # Gamma 用于提亮暗部；这里除以 100 是为了让滑条使用整数。
        if self.show_windows:
            gamma_x100 = cv2.getTrackbarPos(
                "Gamma x100", "Green Jar Detection"
            )
            clahe_clip_x10 = cv2.getTrackbarPos(
                "CLAHE clip x10", "Green Jar Detection"
            )
        else:
            gamma_x100 = int(self.get_parameter("low_light_gamma").value)
            clahe_clip_x10 = int(self.get_parameter("clahe_clip").value)

        gamma_x100 = max(10, min(200, gamma_x100))
        gamma = gamma_x100 / 100.0
        gamma_lut = np.array(
            [
                ((value / 255.0) ** gamma) * 255.0
                for value in range(256)
            ],
            dtype=np.uint8,
        )
        # LUT 能一次性完成全图 Gamma 调整，计算量低于逐像素运算。
        enhanced = cv2.LUT(frame, gamma_lut)

        # HSV 中的 H 表示颜色、S 表示饱和度、V 表示亮度。
        hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        clahe_clip_x10 = max(0, min(100, clahe_clip_x10))
        if clahe_clip_x10 > 0:
            # CLAHE 在局部区域增强亮度对比度，适合桌面光照不均匀的情况。
            clahe = cv2.createCLAHE(
                clipLimit=clahe_clip_x10 / 10.0,
                tileGridSize=(int(self.get_parameter("clahe_grid").value),) * 2,
            )
            hsv[:, :, 2] = clahe.apply(hsv[:, :, 2])

        # 保留同时落在 H/S/V 上下限内的像素，形成绿色目标二值掩膜。
        mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8),
                           np.array(upper, dtype=np.uint8))

        # 腐蚀和膨胀去除小噪点；闭运算填补罐子内部或边缘的小空洞。
        kernel_size = max(1, int(self.kernel_size))
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        candidates = []
        for contour in contours:
            # 轮廓面积低于阈值时认为是噪声，不作为目标候选。
            area = float(cv2.contourArea(contour))
            if area < self.minimum_area:
                continue

            x, y, width, height = cv2.boundingRect(contour)
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            # 用轮廓面积矩计算中心，而不是直接使用外接框几何中心。
            center_u = int(moments["m10"] / moments["m00"])
            center_v = int(moments["m01"] / moments["m00"])
            perimeter = cv2.arcLength(contour, True)
            circularity = (
                4.0 * np.pi * area / (perimeter * perimeter)
                if perimeter > 0 else 0.0
            )

            candidates.append(
                {
                    "class_name": "green_jar",
                    # 当前置信度仅作为面积占比的原型值，不是训练模型置信度。
                    "confidence": round(float(min(1.0, area / max(1.0, frame.shape[0] * frame.shape[1])) * 2.0), 3),
                    "box": [int(x), int(y), int(x + width), int(y + height)],
                    "center": [center_u, center_v],
                    "area": round(area, 1),
                    "circularity": round(float(circularity), 3),
                    "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                }
            )

        candidates.sort(key=lambda item: item["area"], reverse=True)
        # 每帧只保留面积最大的若干个候选目标，避免异常轮廓过多。
        detections = candidates[:self.max_detections]

        for detection in detections:
            # 在原图上画绿色外接框和红色中心点，不修改发布结果坐标。
            x1, y1, x2, y2 = detection["box"]
            center_u, center_v = detection["center"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (center_u, center_v), 5, (0, 0, 255), -1)
            label = "green_jar area=%.0f" % detection["area"]
            cv2.putText(
                frame, label, (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )

        payload = json.dumps(
            {
                # 使用统一字典结构输出类别、框、中心、面积和时间戳。
                "frame_id": msg.header.frame_id,
                "image_size": [frame.shape[1], frame.shape[0]],
                "hsv_lower": list(lower),
                "hsv_upper": list(upper),
                "detections": detections,
            },
            ensure_ascii=False,
        )
        self.detection_pub.publish(String(data=payload))

        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        if self.show_windows:
            cv2.imshow("Green Jar Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                rclpy.shutdown()
            elif key in (ord("s"), ord("f")):
                # 保存成功/失败案例，用于记录暗光效果和误检情况。
                case = "success" if key == ord("s") else "failure"
                output_dir = str(self.get_parameter("results_dir").value)
                os.makedirs(output_dir, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(
                    os.path.join(output_dir, "%s_%s.jpg" % (stamp, case)),
                    frame,
                )
                self.get_logger().info("Saved %s case to %s" % (case, output_dir))


def main(args=None):
    rclpy.init(args=args)
    node = GreenObjectDetector()
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
