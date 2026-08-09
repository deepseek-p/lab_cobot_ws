#!/usr/bin/env python3
"""Publish eight-class YOLO detections as organized RGB-D 3D poses."""
from __future__ import annotations

from collections import deque
import json
import math

from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
import tf2_ros
from vision_msgs.msg import Detection3D, Detection3DArray
from vision_msgs.msg import ObjectHypothesisWithPose

from image_pkg.pcl_node import centroid_from_box
from image_pkg.yolo_world_detector import select_best_candidate


class YoloPointCloudPoseNode(Node):
    """Associate timestamped YOLO boxes with organized point-cloud pixels."""

    def __init__(self):
        super().__init__("yolo_pointcloud_pose")
        self._declare_parameters()
        self.target_frame = str(self._param("target_frame"))
        self.optical_frame = str(self._param("optical_frame"))
        self.target_label = str(self._param("target_label")).strip()
        self.latest_detections = []
        self.latest_detection_stamp = None
        self.detection_image_size = None
        self.cloud_cache = deque(
            maxlen=int(self._param("pointcloud_cache_size"))
        )
        self.processed_detection_stamp = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.objects_pub = self.create_publisher(
            Detection3DArray, self._param("objects_topic"), 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, self._param("pose_topic"), 10
        )
        self.selected_pose_pub = self.create_publisher(
            PoseStamped, self._param("selected_pose_topic"), 10
        )
        self.json_pose_pub = self.create_publisher(
            String, self._param("json_pose_topic"), 10
        )
        self.create_subscription(
            String, self._param("detection_topic"), self._detections_cb, 10
        )
        self.create_subscription(
            PointCloud2,
            self._param("pointcloud_topic"),
            self._cloud_cb,
            qos_profile_sensor_data,
        )

    def _declare_parameters(self):
        params = {
            "detection_topic": "/yolo/detections",
            "pointcloud_topic": "/image_pkg/wrist_camera_points",
            "optical_frame": "wrist_camera_optical_frame",
            "target_frame": "base_link",
            "objects_topic": "/perception/objects",
            "pose_topic": "/perception/yolo/pose",
            "selected_pose_topic": "/perception/target_pose",
            "json_pose_topic": "/yolo/poses",
            "target_label": "aruco_sample",
            "pose_translation_correction": [0.0, 0.0, 0.0],
            "detection_label_aliases": [""],
            "min_points": 20,
            "pointcloud_cache_size": 180,
            "max_rgbd_sync_delta_sec": 0.10,
        }
        for name, value in params.items():
            self.declare_parameter(name, value)

    def _param(self, name):
        return self.get_parameter(name).value

    def _detections_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            detections = payload.get("detections", [])
            self.latest_detections = [
                item for item in detections if _valid_box(item)
            ]
            default_stamp = self.get_clock().now().nanoseconds / 1e9
            self.latest_detection_stamp = float(
                payload.get("timestamp", default_stamp)
            )
            width = payload.get("image_width")
            height = payload.get("image_height")
            self.detection_image_size = (
                (int(width), int(height)) if width and height else None
            )
            self._process_cached_detection()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Ignoring invalid YOLO detection JSON: {exc}"
            )

    def _cloud_cb(self, msg):
        if msg.height <= 1:
            return
        if msg.header.frame_id != self.optical_frame:
            self.get_logger().warning(
                "Ignoring point cloud in %s; expected %s"
                % (msg.header.frame_id, self.optical_frame),
                throttle_duration_sec=2.0,
            )
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.cloud_cache.append((float(stamp), msg))
        self._process_cached_detection()

    def _process_cached_detection(self):
        if not self.latest_detections or self.latest_detection_stamp is None:
            return
        if self.processed_detection_stamp == self.latest_detection_stamp:
            return
        if not self.cloud_cache:
            return
        cloud_stamp, cloud = min(
            self.cloud_cache,
            key=lambda item: abs(item[0] - self.latest_detection_stamp),
        )
        sync_delta = abs(cloud_stamp - self.latest_detection_stamp)
        if sync_delta > float(self._param("max_rgbd_sync_delta_sec")):
            return
        if self.detection_image_size != (cloud.width, cloud.height):
            self.get_logger().warning(
                "YOLO image size does not match organized point cloud",
                throttle_duration_sec=2.0,
            )
            return
        points = _read_xyz_points(cloud)
        if points is None:
            self.get_logger().warning(
                "Point cloud must contain organized x/y/z fields",
                throttle_duration_sec=2.0,
            )
            return
        transform = self._lookup_transform(cloud.header.frame_id, cloud.header.stamp)
        if transform is None:
            return
        self.processed_detection_stamp = self.latest_detection_stamp
        array = Detection3DArray()
        array.header.stamp = cloud.header.stamp
        array.header.frame_id = self.target_frame
        json_items = []
        candidates = []
        correction = self._param("pose_translation_correction")
        for index, item in enumerate(self.latest_detections):
            centroid = centroid_from_box(
                points,
                cloud.width,
                cloud.height,
                *item["box"],
                min_points=int(self._param("min_points")),
            )
            if centroid is None:
                continue
            xyz = _transform_point(centroid, transform)
            if len(correction) == 3:
                xyz = tuple(
                    value + float(correction[axis])
                    for axis, value in enumerate(xyz)
                )
            label = self._canonical_label(str(item.get("label", "unknown")))
            confidence = float(item.get("confidence", 0.0))
            pose = _pose(cloud.header, self.target_frame, xyz)
            array.detections.append(
                _detection3d(pose, label, confidence, index)
            )
            self.pose_pub.publish(pose)
            candidates.append({
                "label": label,
                "confidence": confidence,
                "pose": pose,
            })
            json_items.append({
                "label": label,
                "confidence": confidence,
                "frame_id": self.target_frame,
                "position": list(xyz),
            })
        if not array.detections:
            return
        self.objects_pub.publish(array)
        selected = select_best_candidate(candidates, self.target_label)
        if selected is not None:
            self.selected_pose_pub.publish(selected["pose"])
        output = String()
        output.data = json.dumps({
            "timestamp": self.latest_detection_stamp,
            "frame_id": self.target_frame,
            "detections": json_items,
        })
        self.json_pose_pub.publish(output)

    def _lookup_transform(self, source_frame, stamp):
        if source_frame == self.target_frame:
            identity = TransformStamped()
            identity.transform.rotation.w = 1.0
            return identity
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.1),
            )
        except Exception as stamped_error:  # noqa: BLE001
            try:
                return self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
            except Exception:  # noqa: BLE001
                self.get_logger().warning(
                    "Cannot transform %s to %s: %s"
                    % (source_frame, self.target_frame, stamped_error),
                    throttle_duration_sec=2.0,
                )
                return None

    def _canonical_label(self, label):
        aliases = {}
        for entry in self._param("detection_label_aliases"):
            source, separator, target = str(entry).partition("=")
            if separator and source.strip() and target.strip():
                aliases[source.strip().lower()] = target.strip()
        return aliases.get(label.strip().lower(), label.strip())


def _read_xyz_points(cloud):
    try:
        raw = point_cloud2.read_points(
            cloud,
            field_names=("x", "y", "z"),
            skip_nans=False,
        )
        if getattr(raw, "dtype", None) is not None:
            names = getattr(raw.dtype, "names", None)
            if names:
                points = np.column_stack(
                    [raw["x"], raw["y"], raw["z"]]
                ).astype(np.float32, copy=False)
            else:
                points = np.asarray(raw, dtype=np.float32)
        else:
            points = np.asarray(list(raw), dtype=np.float32)
        return points.reshape(cloud.height, cloud.width, 3)
    except (KeyError, TypeError, ValueError):
        return None


def _valid_box(item):
    box = item.get("box", []) if isinstance(item, dict) else []
    return len(box) == 4 and all(
        isinstance(value, (int, float)) and math.isfinite(value)
        for value in box
    )


def _transform_point(point, transform):
    rotation = transform.transform.rotation
    vector = np.asarray(point, dtype=float)
    axis = np.asarray((rotation.x, rotation.y, rotation.z), dtype=float)
    cross = np.cross(axis, vector)
    double_cross = np.cross(axis, cross)
    rotated = vector + 2.0 * (rotation.w * cross + double_cross)
    translation = transform.transform.translation
    return tuple(
        float(value)
        for value in rotated + (translation.x, translation.y, translation.z)
    )


def _pose(header, frame_id, xyz):
    pose = PoseStamped()
    pose.header.stamp = header.stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
    pose.pose.orientation.w = 1.0
    return pose


def _detection3d(pose, label, confidence, index):
    detection = Detection3D()
    detection.header = pose.header
    detection.id = f"yolo_{index}_{label.replace(' ', '_')}"
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = label
    result.hypothesis.score = confidence
    result.pose.pose = pose.pose
    detection.results.append(result)
    detection.bbox.center = pose.pose
    return detection


def main(args=None):
    """Run the eight-class point-cloud pose node."""
    rclpy.init(args=args)
    node = YoloPointCloudPoseNode()
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
