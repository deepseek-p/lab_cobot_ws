#!/usr/bin/env python3
"""Publish YOLO-associated RGB-D poses in ``base_link`` and evaluation logs."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
import tf2_ros
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from image_pkg.pcl_node import centroid_from_box


class YoloPointCloudPoseNode(Node):
    """Associate YOLO boxes with their matching organized point-cloud pixels."""

    def __init__(self):
        super().__init__("yolo_pointcloud_pose")
        self._declare_parameters()
        self.target_frame = self._param("target_frame")
        self.pose_source = self._param("pose_source").lower()
        self.latest_detections = []
        self.latest_detection_stamp = None
        self.detection_image_size = None
        self.latest_estimates = {}
        self._last_evaluated_detection_stamp = {}
        self.gazebo_truth = {}
        self._log_path = Path(self._param("evaluation_log_path")).expanduser()
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_path.touch(exist_ok=True)
        self.get_logger().info(f"Pose evaluation log: {self._log_path}")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.objects_pub = self.create_publisher(
            Detection3DArray, self._param("objects_topic"), 10)
        self.pose_pub = self.create_publisher(PoseStamped, self._param("pose_topic"), 10)
        self.selected_pose_pub = self.create_publisher(
            PoseStamped, self._param("selected_pose_topic"), 10)
        self.json_pose_pub = self.create_publisher(String, self._param("json_pose_topic"), 10)
        self.evaluation_pub = self.create_publisher(
            String, self._param("evaluation_topic"), 10)
        self.create_subscription(String, self._param("detection_topic"), self._detections_cb, 10)
        self.create_subscription(PointCloud2, self._param("pointcloud_topic"), self._cloud_cb, 10)
        self.create_subscription(PoseStamped, self._param("aruco_pose_topic"), self._aruco_cb, 10)
        self.create_subscription(String, self._param("grasp_status_topic"), self._grasp_cb, 10)
        # Gazebo truth is evaluation-only: it is never published to the pose
        # topics consumed by manipulation.
        truth_topic = self._param("gazebo_truth_topic")
        if truth_topic:
            try:
                from gazebo_msgs.msg import ModelStates
                self.create_subscription(ModelStates, truth_topic, self._truth_cb, 10)
            except ImportError:
                self.get_logger().warning("gazebo_msgs unavailable; truth logging disabled")

    def _declare_parameters(self):
        params = {
            "detection_topic": "/yolo/detections", "pointcloud_topic": "/kinect2/sd/points",
            "target_frame": "base_link", "objects_topic": "/perception/objects",
            "pose_topic": "/perception/yolo/pose", "json_pose_topic": "/yolo/poses",
            "evaluation_topic": "/perception/yolo/evaluation",
            "selected_pose_topic": "/perception/target_pose", "pose_source": "yolo",
            "aruco_pose_topic": "/perception/aruco_0/pose", "grasp_status_topic": "/gripper/status",
            "gazebo_truth_topic": "/gazebo/model_states",
            "gazebo_model_names": ["aruco_sample", "material_spare_igbt", "material_grease_can", "tooling_fixture_box", "tooling_hand_tools", "aging_rack", "board_test_fixture", "high_voltage_probe_kit"],
            "gazebo_model_labels": ["aruco_sample", "igbt_module_plain", "thermal_grease_can", "fixture_box_plain", "tooling_hand_tools", "aging_rack", "pcb_test_fixture", "safety_probe_kit"],
            "gazebo_truth_frame": "odom",
            # ROS 2 cannot infer an empty string-array parameter type; keep a
            # harmless empty entry so YAML can override it with real aliases.
            "gazebo_robot_model_name": "lab_cobot", "detection_label_aliases": [""],
            "evaluation_log_path": "~/.ros/yolo_pose_evaluation.jsonl", "min_points": 20,
            "max_detection_age_sec": 0.5,
        }
        for name, value in params.items():
            self.declare_parameter(name, value)

    def _param(self, name):
        return self.get_parameter(name).value

    def _detections_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            detections = payload.get("detections", [])
            self.latest_detections = [item for item in detections if _valid_box(item)]
            self.latest_detection_stamp = float(payload.get("timestamp", self.get_clock().now().nanoseconds / 1e9))
            width, height = payload.get("image_width"), payload.get("image_height")
            self.detection_image_size = (int(width), int(height)) if width and height else None
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid YOLO detection JSON: {exc}")

    def _cloud_cb(self, msg):
        if msg.height <= 1 or not self.latest_detections:
            return
        if self.detection_image_size and self.detection_image_size != (msg.width, msg.height):
            self.get_logger().warning(
                "YOLO image size does not match organized point cloud; skipping association",
                throttle_duration_sec=2.0)
            return
        age = self.get_clock().now().nanoseconds / 1e9 - (self.latest_detection_stamp or 0.0)
        if age > float(self._param("max_detection_age_sec")):
            return
        try:
            raw_points = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=False)
            if getattr(raw_points.dtype, "names", None):
                points = np.column_stack([
                    raw_points["x"], raw_points["y"], raw_points["z"],
                ]).astype(np.float32, copy=False)
            else:
                points = np.asarray(list(raw_points), dtype=np.float32)
            points = points.reshape(msg.height, msg.width, 3)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f"Point cloud must contain organized x/y/z fields: {exc}")
            return
        transform = self._lookup_transform(msg.header.frame_id, msg.header.stamp)
        if transform is None:
            return
        array = Detection3DArray()
        array.header.stamp = msg.header.stamp
        array.header.frame_id = self.target_frame
        json_items = []
        for index, item in enumerate(self.latest_detections):
            centroid = centroid_from_box(points, msg.width, msg.height, *item["box"], min_points=int(self._param("min_points")))
            if centroid is None:
                continue
            xyz = _transform_point(centroid, transform)
            pose = _pose(msg.header, self.target_frame, xyz)
            label = self._canonical_label(str(item.get("label", "unknown")))
            self.latest_estimates[label] = pose
            array.detections.append(_detection3d(pose, label, float(item.get("confidence", 0.0)), index))
            json_items.append({"label": label, "confidence": float(item.get("confidence", 0.0)), "frame_id": self.target_frame, "position": list(xyz)})
            self.pose_pub.publish(pose)
            if self.pose_source == "yolo":
                self.selected_pose_pub.publish(pose)
            # A detector frame can be paired with several newer point clouds
            # while it remains inside max_detection_age_sec.  Count its error
            # once, otherwise the position-rate metric is inflated by camera
            # frequency rather than detector observations.
            if self._last_evaluated_detection_stamp.get(label) != self.latest_detection_stamp:
                self._last_evaluated_detection_stamp[label] = self.latest_detection_stamp
                self._record_estimate(label, pose)
        if array.detections:
            self.objects_pub.publish(array)
            output = String()
            output.data = json.dumps({"timestamp": self.latest_detection_stamp, "frame_id": self.target_frame, "detections": json_items})
            self.json_pose_pub.publish(output)

    def _lookup_transform(self, source_frame, stamp=None):
        if source_frame == self.target_frame:
            identity = type("IdentityTransform", (), {})()
            identity.transform = type("Transform", (), {})()
            identity.transform.translation = type("Translation", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
            identity.transform.rotation = type("Rotation", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()
            return identity
        try:
            query_time = Time() if stamp is None else Time.from_msg(stamp)
            return self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, query_time,
                timeout=Duration(seconds=0.1))
        except Exception as exc:  # TF failures are expected while Gazebo starts.
            if stamp is not None:
                try:
                    return self.tf_buffer.lookup_transform(
                        self.target_frame, source_frame, Time(),
                        timeout=Duration(seconds=0.1))
                except Exception:
                    pass
            self.get_logger().warning(f"Cannot transform {source_frame} to {self.target_frame}: {exc}", throttle_duration_sec=2.0)
            return None

    def _aruco_cb(self, pose):
        if self.pose_source == "aruco" and pose.header.frame_id == self.target_frame:
            self.selected_pose_pub.publish(pose)

    def _canonical_label(self, label):
        """Translate visual aliases to the semantic names used by evaluation.

        The aliases preserve the detector's convenient colour prompts while
        making published poses and Gazebo truth records refer to the same
        physical sample name.
        """
        aliases = {}
        for entry in self._param("detection_label_aliases"):
            source, separator, target = str(entry).partition("=")
            if separator and source.strip() and target.strip():
                aliases[source.strip().lower()] = target.strip()
        return aliases.get(label.strip().lower(), label)

    def _truth_cb(self, msg):
        model_labels = dict(zip(
            self._param("gazebo_model_names"),
            self._param("gazebo_model_labels"),
        ))
        poses_by_name = dict(zip(msg.name, msg.pose))
        robot_pose = poses_by_name.get(self._param("gazebo_robot_model_name"))
        transform = None
        if robot_pose is None:
            transform = self._lookup_transform(self._param("gazebo_truth_frame"))
            if transform is None:
                return
        for name, pose in zip(msg.name, msg.pose):
            label = model_labels.get(name)
            if label:
                point = (pose.position.x, pose.position.y, pose.position.z)
                # ModelStates is expressed in Gazebo world coordinates.  Its
                # robot model pose gives an exact world->base_link conversion
                # even when this simulator does not publish odom->base_link.
                self.gazebo_truth[label] = (
                    _inverse_pose_point(point, robot_pose)
                    if robot_pose is not None and self.target_frame == "base_link"
                    else _transform_point(point, transform)
                )

    def _grasp_cb(self, msg):
        text = msg.data.strip()
        success = text.lower().startswith(("attached", "success", "grasped"))
        self._write_record({"event": "grasp", "success": success, "failure_reason": "" if success else text})

    def _record_estimate(self, label, pose):
        truth = self.gazebo_truth.get(label)
        estimated = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        record = {"event": "pose_estimate", "label": label, "frame_id": self.target_frame, "estimated_position": estimated, "gazebo_truth_position": truth}
        if truth is not None:
            error = [estimated[i] - truth[i] for i in range(3)]
            record.update({"error_xyz": error, "total_position_error": math.sqrt(sum(value * value for value in error))})
        self._write_record(record)
        if truth is not None:
            message = String()
            message.data = json.dumps(record)
            self.evaluation_pub.publish(message)

    def _write_record(self, record):
        record["timestamp"] = self.get_clock().now().nanoseconds / 1e9
        with self._log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")


def _valid_box(item):
    box = item.get("box", []) if isinstance(item, dict) else []
    return len(box) == 4 and all(isinstance(value, (int, float)) and math.isfinite(value) for value in box)


def _transform_point(point, transform):
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    px, py, pz = point
    # Quaternion-vector rotation, then translation.
    uv = np.cross((x, y, z), (px, py, pz))
    uuv = np.cross((x, y, z), uv)
    rotated = np.asarray((px, py, pz)) + 2.0 * (w * uv + uuv)
    t = transform.transform.translation
    return tuple(float(value) for value in rotated + (t.x, t.y, t.z))


def _inverse_pose_point(point, pose):
    """Express a Gazebo-world point in the coordinate system of ``pose``."""
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    relative = np.asarray(point, dtype=float) - np.asarray(
        (pose.position.x, pose.position.y, pose.position.z), dtype=float)
    # Inverse unit-quaternion rotation: conjugate(q) * relative * q.
    uv = np.cross((-x, -y, -z), relative)
    uuv = np.cross((-x, -y, -z), uv)
    rotated = relative + 2.0 * (w * uv + uuv)
    return tuple(float(value) for value in rotated)


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
    rclpy.init(args=args)
    node = YoloPointCloudPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Launch may already have shut the context down after SIGINT.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
