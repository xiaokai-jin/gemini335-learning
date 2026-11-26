#!/usr/bin/env python3

"""在 ROS2 彩色图像上运行 CUDA 加速的 YOLOE 开放词汇目标检测。"""

import json
import os
import threading
import time
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLOE


class YoloObjectDetector(Node):
    def __init__(self):
        super().__init__("yolo_object_detector")

        # 话题和模型参数。YOLOE 可在运行时通过文本提示切换关注类别。
        self.declare_parameter("image_topic", "/gemini335/color/image_raw")
        self.declare_parameter("annotated_topic", "/gemini335/detection/yolo/image")
        self.declare_parameter("detection_topic", "/gemini335/detection/yolo")
        self.declare_parameter("model", "models/yoloe-26s-seg.pt")
        self.declare_parameter(
            "classes",
            "a green jar,a green tissue package,a bottle,a cup,a person",
        )
        self.declare_parameter("image_size", 640)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("max_detections", 50)
        self.declare_parameter("max_inference_rate", 20.0)
        self.declare_parameter("device", "0")
        self.declare_parameter("half_precision", True)
        self.declare_parameter("show_window", True)
        self.declare_parameter(
            "results_dir",
            os.path.abspath(
                os.path.join(os.path.dirname(__file__), "..", "results", "images", "yolo")
            ),
        )

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.model_name = str(self.get_parameter("model").value)
        self.classes = [
            item.strip()
            for item in str(self.get_parameter("classes").value).split(",")
            if item.strip()
        ]
        if not self.classes:
            raise ValueError("classes 至少需要提供一个类别提示")

        self.image_size = int(self.get_parameter("image_size").value)
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.max_detections = int(self.get_parameter("max_detections").value)
        self.max_inference_rate = float(
            self.get_parameter("max_inference_rate").value
        )
        self.device = str(self.get_parameter("device").value)
        self.half_precision = bool(
            self.get_parameter("half_precision").value
        )
        self.show_window = bool(self.get_parameter("show_window").value)
        self.results_dir = str(self.get_parameter("results_dir").value)

        if self.image_size <= 0 or self.max_detections <= 0:
            raise ValueError("image_size 和 max_detections 必须大于 0")
        if self.max_inference_rate <= 0.0:
            raise ValueError("max_inference_rate 必须大于 0")
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold 必须在 [0, 1] 范围内")
        if not 0.0 <= self.iou_threshold <= 1.0:
            raise ValueError("iou_threshold 必须在 [0, 1] 范围内")

        self.bridge = CvBridge()
        self.annotated_pub = self.create_publisher(
            Image, str(self.get_parameter("annotated_topic").value), 10
        )
        self.detection_pub = self.create_publisher(
            String, str(self.get_parameter("detection_topic").value), 10
        )

        # 首次启动会从 Ultralytics 官方资源下载预训练权重。
        self.get_logger().info(
            "加载模型 %s，并设置文本类别：%s"
            % (self.model_name, ", ".join(self.classes))
        )
        self.model = YOLOE(self.model_name)
        self.model.set_classes(self.classes)

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        if self.show_window:
            cv2.namedWindow("YOLO Object Detection", cv2.WINDOW_NORMAL)

        self._frame_lock = threading.Lock()
        self._result_lock = threading.Lock()
        self._frame_available = threading.Event()
        self._stop_worker = threading.Event()
        self._pending_frame = None
        self._pending_result = None
        self._display_frame = None
        self._inference_thread = threading.Thread(
            target=self._inference_worker,
            name="yolo-inference",
            daemon=True,
        )
        self._inference_thread.start()
        self._ui_timer = self.create_timer(1.0 / 30.0, self._publish_result)

        self.get_logger().info(
            "订阅 %s；device=%s, half=%s, max_inference_rate=%.1f Hz；"
            "发布检测到 %s"
            % (
                self.image_topic,
                self.device,
                self.half_precision,
                self.max_inference_rate,
                str(self.get_parameter("detection_topic").value),
            )
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as exc:
            self.get_logger().error("无法转换输入图像：%s" % exc)
            return

        with self._frame_lock:
            self._pending_frame = (frame.copy(), msg.header)
            self._frame_available.set()

    def _inference_worker(self):
        min_interval = 1.0 / self.max_inference_rate
        next_inference_time = 0.0

        while not self._stop_worker.is_set():
            self._frame_available.wait()
            if self._stop_worker.is_set():
                break

            delay = next_inference_time - time.monotonic()
            if delay > 0.0 and self._stop_worker.wait(delay):
                break

            with self._frame_lock:
                frame_data = self._pending_frame
                self._pending_frame = None
                self._frame_available.clear()

            if frame_data is None:
                continue

            frame, header = frame_data
            start_time = time.monotonic()
            next_inference_time = start_time + min_interval
            try:
                result = self.model.predict(
                    source=frame,
                    imgsz=self.image_size,
                    conf=self.confidence_threshold,
                    iou=self.iou_threshold,
                    max_det=self.max_detections,
                    device=self.device,
                    half=self.half_precision,
                    verbose=False,
                )[0]
            except Exception as exc:
                self.get_logger().error("YOLO 推理失败：%s" % exc)
                continue

            annotated = result.plot()
            detections = []
            if result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                confidences = result.boxes.conf.cpu().numpy()
                class_ids = result.boxes.cls.int().cpu().tolist()

                for box, confidence, class_id in zip(
                    boxes, confidences, class_ids
                ):
                    detections.append(
                        {
                            "class_name": str(result.names[class_id]),
                            "class_id": int(class_id),
                            "confidence": round(float(confidence), 4),
                            "box": [round(float(value), 2) for value in box],
                        }
                    )

            payload = {
                "frame_id": header.frame_id,
                "timestamp": header.stamp.sec + header.stamp.nanosec / 1e9,
                "image_size": [int(frame.shape[1]), int(frame.shape[0])],
                "model": self.model_name,
                "classes": self.classes,
                "device": self.device,
                "detection_count": len(detections),
                "detections": detections,
            }

            with self._result_lock:
                self._pending_result = (payload, annotated, header)

    def _publish_result(self):
        with self._result_lock:
            result_data = self._pending_result
            self._pending_result = None

        if result_data is not None:
            payload, annotated, header = result_data
            self.detection_pub.publish(
                String(data=json.dumps(payload, ensure_ascii=False))
            )
            annotated_msg = self.bridge.cv2_to_imgmsg(
                annotated, encoding="bgr8"
            )
            annotated_msg.header = header
            self.annotated_pub.publish(annotated_msg)
            self._display_frame = annotated

        if not self.show_window:
            return

        if self._display_frame is not None:
            cv2.imshow("YOLO Object Detection", self._display_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            rclpy.shutdown()
        elif key in (ord("s"), ord("f")) and self._display_frame is not None:
            case = "success" if key == ord("s") else "failure"
            os.makedirs(self.results_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = os.path.join(
                self.results_dir, "%s_%s.jpg" % (stamp, case)
            )
            if not cv2.imwrite(output_path, self._display_frame):
                self.get_logger().error("无法保存案例图：%s" % output_path)
            else:
                self.get_logger().info("已保存案例图：%s" % output_path)

    def destroy_node(self):
        self._stop_worker.set()
        self._frame_available.set()
        if self._inference_thread.is_alive():
            self._inference_thread.join(timeout=5.0)
            if self._inference_thread.is_alive():
                self.get_logger().warning("YOLO 推理线程尚未在 5 秒内结束")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
