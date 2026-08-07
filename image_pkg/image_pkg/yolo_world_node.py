#!/usr/bin/env python3
"""Trained-YOLO ROS node retaining the legacy /yolo/detections JSON contract."""
import json
import threading

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from image_pkg.yolo_world_detector import (
    Detection,
    YoloDetector,
    deduplicate_detections,
)


class YoloWorldNode(Node):
    def __init__(self):
        super().__init__("yolo_world_detection")
        defaults = {
            "image_topic": "/bench_camera/image_raw",
            "detection_topic": "/yolo/detections",
            "annotated_image_topic": "/yolo/annotated_image",
            "model_path": "models/best.pt",
            "target_classes": [
                # Only the arrowed models in the five work zones are targets.
                # Zone names and home are navigation labels, not object classes.
                "material_spare_igbt",
                "aruco_sample",
                "material_grease_can",
                "aging_rack",
                "board_test_fixture",
                "tooling_fixture_box",
                "tooling_hand_tools",
                "high_voltage_probe_kit",
            ],
            # Keep the eight new best.pt labels unchanged end-to-end.
            "class_label_aliases": [""],
            # The custom Gazebo model produces valid object confidences in the
            # 0.08--0.28 range.  Keep this in sync with pose_estimation.yaml;
            # otherwise a direct ``ros2 run`` silently rejects every target.
            "confidence_threshold": 0.05,
            "nms_iou_threshold": 0.45,
            "device": "auto",
            "inference_imgsz": 640,
            "publish_annotated_image": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.detector = YoloDetector(
            self.get_parameter("model_path").value,
            self.get_parameter("device").value,
            get_package_share_directory("image_pkg"),
        )
        self.target_classes = {
            str(value).strip().lower()
            for value in self.get_parameter("target_classes").value
        }
        self._aruco_detect = self._make_aruco_detector()
        self.bridge = CvBridge()
        confidence = self.get_parameter("confidence_threshold").value
        self.confidence = float(confidence)
        self.nms_iou = float(self.get_parameter("nms_iou_threshold").value)
        self.inference_imgsz = int(self.get_parameter("inference_imgsz").value)
        publish_annotated = self.get_parameter("publish_annotated_image").value
        self.publish_annotated = bool(publish_annotated)
        self.get_logger().info(
            "YOLO detector ready: model=%s, confidence=%.3f, image=%s"
            % (
                self.get_parameter("model_path").value,
                self.confidence,
                self.get_parameter("image_topic").value,
            )
        )
        self.detection_pub = self.create_publisher(
            String, self.get_parameter("detection_topic").value, 10
        )
        self.annotated_pub = None
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image, self.get_parameter("annotated_image_topic").value, 1
            )
        self._condition = threading.Condition()
        self._latest_message = None
        self._stopping = False
        self._worker = threading.Thread(
            target=self._inference_loop, daemon=True
        )
        self._worker.start()
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.callback,
            qos_profile_sensor_data,
        )

    def callback(self, msg):
        # Do not run inference in the ROS subscription callback.  Keeping only
        # the most recent message prevents an ever-growing latency backlog when
        # inference is slower than the camera.
        with self._condition:
            self._latest_message = msg
            self._condition.notify()

    def _inference_loop(self):
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._stopping or self._latest_message is not None
                )
                if self._stopping:
                    return
                msg = self._latest_message
                self._latest_message = None
            self._process_message(msg)

    def _process_message(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
            detections = self.detector.infer(
                image, self.confidence, self.nms_iou, self.inference_imgsz
            )
            detections = [
                Detection(
                    self._canonical_label(detection.label),
                    detection.confidence,
                    detection.box,
                )
                for detection in detections
            ]
            detections = [
                detection for detection in detections
                if detection.label.strip().lower() in self.target_classes
            ]
            detections = self._add_aruco_fallback(image, detections)
            detections = deduplicate_detections(detections)
        except Exception as exc:
            # This runs in a background thread.  Catch every frame-level error
            # so a single OpenCV/Ultralytics exception cannot permanently kill
            # detections without leaving a ROS-visible error.
            self.get_logger().warning(f"YOLO frame skipped: {exc}")
            return
        try:
            output = String()
            output.data = json.dumps({
                "timestamp": float(
                    msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                ),
                "frame_id": str(msg.header.frame_id),
                "image_width": int(image.shape[1]),
                "image_height": int(image.shape[0]),
                "detections": [
                    {
                        "label": str(detection.label),
                        "confidence": float(detection.confidence),
                        # ArUco's min/max output uses numpy scalar integers;
                        # convert all box values before JSON serialization.
                        "box": [int(value) for value in detection.box],
                    }
                    for detection in detections
                ],
            })
        except Exception as exc:
            self.get_logger().warning(f"YOLO result serialization skipped: {exc}")
            return
        try:
            self.detection_pub.publish(output)
        except Exception as exc:  # ROS may already be shutting down.
            self.get_logger().debug(f"YOLO result discarded: {exc}")
            return
        if detections:
            self.get_logger().info(
                "Published %d detection(s): %s"
                % (len(detections), ", ".join(d.label for d in detections))
            )
        if self.publish_annotated:
            annotated = image.copy()
            for d in detections:
                x1, y1, x2, y2 = d.box
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated,
                    f"{d.label} {d.confidence:.2f}",
                    (x1, max(15, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    .5,
                    (0, 255, 0),
                    1,
                )
            rendered = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            rendered.header = msg.header
            try:
                self.annotated_pub.publish(rendered)
            except Exception as exc:  # ROS may already be shutting down.
                self.get_logger().debug(
                    f"Annotated frame discarded: {exc}"
                )

    def _make_aruco_detector(self):
        """Create a version-compatible ArUco detector when requested."""
        if "aruco_sample" not in self.target_classes or not hasattr(cv2, "aruco"):
            return None
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(
                dictionary, aruco.DetectorParameters()
            )
            return lambda gray: detector.detectMarkers(gray)[:2]
        parameters = aruco.DetectorParameters_create()
        return lambda gray: aruco.detectMarkers(
            gray, dictionary, parameters=parameters
        )[:2]

    def _canonical_label(self, label):
        """Map trained-model class names to project semantic labels."""
        aliases = {}
        for entry in self.get_parameter("class_label_aliases").value:
            source, separator, target = str(entry).partition("=")
            if separator and source.strip() and target.strip():
                aliases[source.strip().lower()] = target.strip()
        return aliases.get(str(label).strip().lower(), str(label).strip())

    def _add_aruco_fallback(self, image, detections):
        """Add deterministic ArUco boxes for Gazebo marker samples.

        The trained YOLO model remains the detector for unmarked objects.  The fallback
        only supplies the explicitly requested ``aruco_sample`` class, whose
        binary code is more reliable than open-vocabulary inference on the
        low-texture Gazebo rendering.
        """
        if self._aruco_detect is None:
            return detections
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, marker_ids = self._aruco_detect(gray)
        scale = 1
        if marker_ids is None:
            # The bench camera sees the 70 mm marker at a small pixel size.
            # Nearest-neighbour scaling preserves its binary cells.
            scale = 2
            enlarged = cv2.resize(
                gray, None, fx=scale, fy=scale,
                interpolation=cv2.INTER_NEAREST
            )
            corners, marker_ids = self._aruco_detect(enlarged)
        if marker_ids is None:
            return detections
        fallback = list(detections)
        for corner in corners:
            points = corner.reshape(-1, 2) / scale
            x1, y1 = points.min(axis=0).astype(int)
            x2, y2 = points.max(axis=0).astype(int)
            if (x2 - x1) * (y2 - y1) >= 25:
                fallback.append(
                    Detection("aruco_sample", 1.0, (x1, y1, x2, y2))
                )
        return fallback

    def destroy_node(self):
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
