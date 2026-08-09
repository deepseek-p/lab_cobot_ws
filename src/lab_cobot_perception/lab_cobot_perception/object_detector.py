#!/usr/bin/env python3
"""YOLO-World and point cloud object detector node."""
from pathlib import Path
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

import tf2_ros

from lab_cobot_perception import pointcloud_ops
from lab_cobot_perception import yolo_backend
from lab_cobot_perception.quat_math import _quat_from_msg, _quat_rotate


DEFAULT_MODEL_PATH = "~/lab_cobot_models/yolo_world_lab_slim.pt"


class ObjectDetector(Node):
    """Publish semantic 3D object detections from RGB-D data."""

    def __init__(self):
        super().__init__("object_detector")
        self._declare_parameters()
        self._read_parameters()
        self._setup_runtime()

    def _declare_parameters(self):
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("device", "auto")
        self.declare_parameter("imgsz", 1280)
        self.declare_parameter("conf_threshold", 0.05)
        self.declare_parameter("infer_period_sec", 0.5)
        self.declare_parameter("z_min", 0.4)
        self.declare_parameter("z_max", 1.4)
        self.declare_parameter("voxel_size", 0.005)
        self.declare_parameter("plane_dist", 0.008)
        self.declare_parameter("cluster_eps", 0.02)
        self.declare_parameter("cluster_min_points", 15)
        self.declare_parameter("aruco_gate_m", 0.06)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("debug_image", False)
        self.declare_parameter("rgb_topic", "/bench_camera/image_raw")
        self.declare_parameter("depth_topic", "/bench_camera/depth/image_raw")
        self.declare_parameter("info_topic", "/bench_camera/camera_info")
        self.declare_parameter("optical_frame", "camera_optical_frame")
        self.declare_parameter("objects_topic", "/perception/objects")
        self.declare_parameter("aruco_pose_topic", "/perception/aruco_0/pose")

    def _read_parameters(self):
        self.model_path = str(
            Path(str(self.get_parameter("model_path").value)).expanduser()
        )
        self.device = str(self.get_parameter("device").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.infer_period_sec = float(self.get_parameter("infer_period_sec").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.plane_dist = float(self.get_parameter("plane_dist").value)
        self.cluster_eps = float(self.get_parameter("cluster_eps").value)
        self.cluster_min_points = int(self.get_parameter("cluster_min_points").value)
        self.aruco_gate_m = float(self.get_parameter("aruco_gate_m").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.debug_image = bool(self.get_parameter("debug_image").value)
        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.info_topic = str(self.get_parameter("info_topic").value)
        self.optical_frame = str(self.get_parameter("optical_frame").value)
        self.objects_topic = str(self.get_parameter("objects_topic").value)
        self.aruco_pose_topic = str(self.get_parameter("aruco_pose_topic").value)

    def _setup_runtime(self):
        self.disabled = False
        if not Path(self.model_path).exists():
            self.disabled = True
            self.get_logger().error(
                f"YOLO-World model not found: {self.model_path}; DL perception disabled"
            )
            return

        try:
            from cv_bridge import CvBridge
        except Exception as ex:  # pragma: no cover - runtime dependency path
            self.disabled = True
            self.get_logger().error(f"cv_bridge unavailable; DL perception disabled: {ex}")
            return

        try:
            self.model = yolo_backend.load_model(self.model_path, self.device)
        except Exception as ex:  # pragma: no cover - runtime dependency path
            self.disabled = True
            self.get_logger().error(f"YOLO-World model load failed: {ex}")
            return

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.fx = self.fy = self.cx = self.cy = None
        self.rgb_img = None
        self.depth_img = None
        self.aruco_xyz = None
        self.object_pub = self.create_publisher(
            Detection3DArray,
            self.objects_topic,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self.info_topic,
            self._info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.rgb_topic,
            self._rgb_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self._depth_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            self.aruco_pose_topic,
            self._aruco_pose_cb,
            10,
        )
        self.create_timer(self.infer_period_sec, self._process)
        self.get_logger().info("object_detector started")

    def _info_cb(self, msg: CameraInfo):
        k = msg.k
        self.fx, self.fy, self.cx, self.cy = k[0], k[4], k[2], k[5]

    def _rgb_cb(self, msg: Image):
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if msg.encoding in ("16UC1", "mono16"):
            self.depth_img = img.astype(np.float32) / 1000.0
        elif msg.encoding == "32FC1":
            self.depth_img = img.astype(np.float32)
        else:
            self.get_logger().warning(
                f"unknown depth encoding {msg.encoding}; assuming meters",
                once=True,
            )
            self.depth_img = img.astype(np.float32)

    def _aruco_pose_cb(self, msg: PoseStamped):
        if msg.header.frame_id != self.target_frame:
            self.get_logger().warning(
                "ignoring ArUco pose in %s; expected %s"
                % (msg.header.frame_id, self.target_frame),
                once=True,
            )
            return
        position = msg.pose.position
        self.aruco_xyz = np.array([position.x, position.y, position.z])

    def _infer(self, image):
        return yolo_backend.infer(
            self.model,
            image,
            conf=self.conf_threshold,
            imgsz=self.imgsz,
            device=self.device,
        )

    def _process(self):
        if (
            getattr(self, "rgb_img", None) is None
            or getattr(self, "depth_img", None) is None
            or getattr(self, "fx", None) is None
            or not hasattr(self, "object_pub")
        ):
            return

        points = pointcloud_ops.depth_to_points(
            self.depth_img,
            self.fx,
            self.fy,
            self.cx,
            self.cy,
            self.z_min,
            self.z_max,
        )
        clusters_camera = pointcloud_ops.segment_objects(
            points,
            self.voxel_size,
            self.plane_dist,
            self.cluster_eps,
            self.cluster_min_points,
        )
        if not clusters_camera:
            return

        transform = self._lookup_transform()
        if transform is None:
            return
        clusters_target = [
            self._cluster_in_target_frame(cluster, transform)
            for cluster in clusters_camera
        ]
        detections_2d = self._infer(self.rgb_img)
        matches = pointcloud_ops.associate(
            clusters_camera,
            detections_2d,
            self.fx,
            self.fy,
            self.cx,
            self.cy,
        )
        aruco_match = pointcloud_ops.match_aruco(
            clusters_target,
            self.aruco_xyz,
            gate_m=self.aruco_gate_m,
        )
        self.object_pub.publish(
            self._detections_msg(clusters_target, detections_2d, matches, aruco_match)
        )

    def _lookup_transform(self):
        if self.target_frame == self.optical_frame:
            transform = type("IdentityTransform", (), {})()
            transform.transform = type("Transform", (), {})()
            transform.transform.translation = type("Translation", (), {})()
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation = type("Rotation", (), {})()
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            return transform
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                self.optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warning(
                "cannot transform object clusters from %s to %s: %s"
                % (self.optical_frame, self.target_frame, ex)
            )
            return None

    def _cluster_in_target_frame(self, cluster, transform):
        target = dict(cluster)
        target["centroid"] = np.array(
            self._transform_point(cluster["centroid"], transform),
            dtype=np.float64,
        )
        return target

    def _transform_point(self, point, transform):
        rotated = _quat_rotate(_quat_from_msg(transform.transform.rotation), point)
        return (
            rotated[0] + transform.transform.translation.x,
            rotated[1] + transform.transform.translation.y,
            rotated[2] + transform.transform.translation.z,
        )

    def _detections_msg(self, clusters, detections_2d, matches, aruco_match):
        msg = Detection3DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        for index, cluster in enumerate(clusters):
            class_id, score, detection_id = self._classification_for_cluster(
                index,
                detections_2d,
                matches,
                aruco_match,
            )
            msg.detections.append(
                self._detection_msg(cluster, class_id, score, detection_id, msg.header)
            )
        return msg

    def _classification_for_cluster(self, index, detections_2d, matches, aruco_match):
        if aruco_match == index:
            return "sample cube", 1.0, "aruco_0"
        match = matches[index]
        if match is None:
            return "unknown", 0.0, f"cluster_{index}"
        detection = detections_2d[match]
        return detection.class_name, float(detection.conf), f"cluster_{index}"

    def _detection_msg(self, cluster, class_id, score, detection_id, header):
        detection = Detection3D()
        detection.header = header
        detection.id = detection_id

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_id
        hypothesis.hypothesis.score = float(score)

        centroid = np.asarray(cluster["centroid"], dtype=np.float64)
        extent = np.asarray(cluster["extent"], dtype=np.float64)
        _set_pose_position(hypothesis.pose.pose, centroid)
        hypothesis.pose.pose.orientation.w = 1.0
        detection.results.append(hypothesis)

        _set_pose_position(detection.bbox.center, centroid)
        detection.bbox.center.orientation.w = 1.0
        detection.bbox.size.x = float(extent[0])
        detection.bbox.size.y = float(extent[1])
        detection.bbox.size.z = float(extent[2])
        return detection


def _set_pose_position(pose, xyz):
    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2])


def main():
    rclpy.init(args=sys.argv)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
