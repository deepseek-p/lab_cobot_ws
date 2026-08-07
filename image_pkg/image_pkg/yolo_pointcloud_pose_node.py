#!/usr/bin/env python3
"""Publish YOLO-associated RGB-D poses in ``base_link`` and evaluation logs."""
from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
import tf2_ros
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from image_pkg.pcl_node import centroid_from_box
from image_pkg.yolo_world_detector import select_best_candidate


class YoloPointCloudPoseNode(Node):
    """Associate YOLO boxes with their matching organized point-cloud pixels."""

    def __init__(self):
        super().__init__("yolo_pointcloud_pose")
        self._declare_parameters()
        self.target_frame = self._param("target_frame")
        self.pose_source = self._param("pose_source").lower()
        self.target_label = str(self._param("target_label")).strip()
        self.latest_detections = []
        self.latest_detection_stamp = None
        self.detection_image_size = None
        self._cloud_cache = deque(maxlen=int(self._param("pointcloud_cache_size")))
        self._processed_detection_stamp = None
        self.latest_estimates = {}
        self._last_evaluated_detection_stamp = {}
        # ``gazebo_truth`` retains the label lookup for compatibility, while
        # the item list supports localization-only evaluation.  The latter
        # deliberately does not use a work-zone or the detector's class name
        # to select its reference object.
        self.gazebo_truth = {}
        self.gazebo_truth_items = []
        self._gazebo_robot_pose = None
        # /gazebo/model_states is always expressed in Gazebo ``world``.  The
        # wrist RGB-D estimates are expressed in ``target_frame`` (``odom`` in
        # this project), so retain the corresponding odometry pose in order
        # to construct the simulator's dynamic world -> odom transform.
        self._odom_robot_pose = None
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
        self.create_subscription(Odometry, self._param("odom_topic"), self._odom_cb, 20)
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
            # Mapping/evaluation uses ``target_frame`` (normally odom), while
            # the manipulation API consumes a pose relative to the mobile
            # base.  Publishing the selected target in a different frame
            # prevents an odom pose from being silently rejected by mission.
            "selected_pose_frame": "base_link",
            # A one-pose grasp interface needs an explicit target.  Empty
            # selects the highest-confidence valid object as a documented
            # fallback for exploratory use.
            "target_label": "",
            # Fixed RGB-D extrinsic residual in target_frame.  This is an
            # offline calibration value; Gazebo model truth is never read by
            # the online perception/grasp path.
            "pose_translation_correction": [0.0, 0.0, 0.0],
            "aruco_pose_topic": "/perception/aruco_0/pose", "grasp_status_topic": "/gripper/status",
            "evaluate_aruco_pnp": True,
            "model_to_base_link_translation_m": [0.0, 0.0, 0.155],
            "gazebo_truth_topic": "/gazebo/model_states",
            "gazebo_model_names": ["aruco_sample", "material_spare_igbt", "material_grease_can", "tooling_fixture_box", "tooling_hand_tools", "aging_rack", "board_test_fixture", "high_voltage_probe_kit"],
            "gazebo_model_labels": ["aruco_sample", "material_spare_igbt", "material_grease_can", "tooling_fixture_box", "tooling_hand_tools", "aging_rack", "board_test_fixture", "high_voltage_probe_kit"],
            # Gazebo ModelStates poses are in Gazebo ``world`` (not odom).
            "gazebo_truth_frame": "world",
            "odom_topic": "/odom",
            # ``nearest`` measures 3-D localization independently from 2-D
            # classification: each camera-derived position is compared with
            # the nearest mapped target.  ``label`` preserves class-matched
            # evaluation when classification and localization are assessed
            # together.
            "evaluation_truth_association": "nearest",
            # ROS 2 cannot infer an empty string-array parameter type; keep a
            # harmless empty entry so YAML can override it with real aliases.
            "gazebo_robot_model_name": "lab_cobot", "detection_label_aliases": [""],
            # Fixed base_link -> camera_optical_frame extrinsic from
            # lab_cobot_description/urdf/inc/sensors.xacro and pillar.xacro.
            # Used only when the non-GUI launch omits camera TF publication.
            "camera_optical_translation_base": [0.18, -0.22, 1.225],
            "camera_optical_rpy_base": [-1.57079632679, 0.65, -1.57079632679],
            "evaluation_log_path": "~/.ros/yolo_pose_evaluation.jsonl", "min_points": 20,
            # YOLO finishes after the source RGB frame.  Keep a short history
            # of organized clouds and pair by source timestamp, never by the
            # robot's current pose when inference completes.
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
            self.latest_detections = [item for item in detections if _valid_box(item)]
            self.latest_detection_stamp = float(payload.get("timestamp", self.get_clock().now().nanoseconds / 1e9))
            width, height = payload.get("image_width"), payload.get("image_height")
            self.detection_image_size = (int(width), int(height)) if width and height else None
            self._process_cached_detection()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid YOLO detection JSON: {exc}")

    def _cloud_cb(self, msg):
        if msg.height <= 1:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._cloud_cache.append((float(stamp), msg))
        self._process_cached_detection()

    def _process_cached_detection(self):
        """Use the cloud captured with the RGB image, not a later one.

        Detector inference is slower than the camera, especially on CPU.  A
        latest-frame association combines a box from one robot pose with a
        cloud/TF from another pose.  Timestamped cache matching prevents this
        motion-induced meter-scale error.
        """
        if not self.latest_detections or self.latest_detection_stamp is None:
            return
        if self._processed_detection_stamp == self.latest_detection_stamp:
            return
        if not self._cloud_cache:
            return
        cloud_stamp, msg = min(
            self._cloud_cache,
            key=lambda item: abs(item[0] - self.latest_detection_stamp),
        )
        sync_delta = abs(cloud_stamp - self.latest_detection_stamp)
        if sync_delta > float(self._param("max_rgbd_sync_delta_sec")):
            return
        self._processed_detection_stamp = self.latest_detection_stamp
        if self.detection_image_size and self.detection_image_size != (msg.width, msg.height):
            self.get_logger().warning(
                "YOLO image size does not match organized point cloud; skipping association",
                throttle_duration_sec=2.0)
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
        target_candidates = []
        for index, item in enumerate(self.latest_detections):
            centroid = centroid_from_box(points, msg.width, msg.height, *item["box"], min_points=int(self._param("min_points")))
            if centroid is None:
                continue
            xyz = _transform_point(centroid, transform)
            correction = self._param("pose_translation_correction")
            if len(correction) == 3:
                xyz = tuple(
                    float(xyz[index]) + float(correction[index])
                    for index in range(3)
                )
            pose = _pose(msg.header, self.target_frame, xyz)
            label = self._canonical_label(str(item.get("label", "unknown")))
            self.latest_estimates[label] = pose
            array.detections.append(_detection3d(pose, label, float(item.get("confidence", 0.0)), index))
            json_items.append({"label": label, "confidence": float(item.get("confidence", 0.0)), "frame_id": self.target_frame, "position": list(xyz)})
            target_candidates.append({
                "label": label,
                "confidence": float(item.get("confidence", 0.0)),
                "pose": pose,
            })
            self.pose_pub.publish(pose)
            # Each detection image is paired with exactly one timestamped
            # cloud, so error statistics are detector observations rather
            # than camera-rate duplicates.
            if self._last_evaluated_detection_stamp.get(label) != self.latest_detection_stamp:
                self._last_evaluated_detection_stamp[label] = self.latest_detection_stamp
                self._record_estimate(label, pose)
        if array.detections:
            self.objects_pub.publish(array)
            selected = select_best_candidate(target_candidates, self.target_label)
            if self.pose_source == "yolo" and selected is not None:
                grasp_pose = self._pose_in_selected_frame(selected["pose"])
                if grasp_pose is not None:
                    self.selected_pose_pub.publish(grasp_pose)
            output = String()
            output.data = json.dumps({"timestamp": self.latest_detection_stamp, "frame_id": self.target_frame, "detections": json_items})
            self.json_pose_pub.publish(output)

    def _pose_in_selected_frame(self, pose):
        """Convert only the manipulation target to its required frame.

        ``target_frame`` remains the fixed mapping frame, so calibration and
        evaluation stay in odom.  Mission, however, defines the TCP target in
        ``base_link``.  The fallback uses the already cached Gazebo robot pose
        only when the simulator temporarily lacks the odom/base TF; it is not
        used as a perception estimate.
        """
        desired_frame = str(self._param("selected_pose_frame")).strip()
        if not desired_frame or desired_frame == pose.header.frame_id:
            return pose
        try:
            transform = self.tf_buffer.lookup_transform(
                desired_frame, pose.header.frame_id, Time.from_msg(pose.header.stamp),
                timeout=Duration(seconds=0.1),
            )
            xyz = _transform_point(
                (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                transform,
            )
        except Exception:
            if (
                desired_frame == "base_link"
                and pose.header.frame_id in {"odom", "world"}
                and self._gazebo_robot_pose is not None
            ):
                xyz = _inverse_pose_point(
                    (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                    self._gazebo_robot_pose,
                )
            else:
                self.get_logger().warning(
                    "Cannot convert selected target %s to %s"
                    % (pose.header.frame_id, desired_frame),
                    throttle_duration_sec=2.0,
                )
                return None
        converted = _pose(pose.header, desired_frame, xyz)
        converted.pose.orientation = pose.pose.orientation
        return converted

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
            fallback = self._gazebo_camera_transform(source_frame)
            if fallback is not None:
                return fallback
            self.get_logger().warning(f"Cannot transform {source_frame} to {self.target_frame}: {exc}", throttle_duration_sec=2.0)
            return None

    def _gazebo_camera_transform(self, source_frame):
        """Build odom<-camera from Gazebo robot truth when camera TF is absent."""
        if self.target_frame != "odom" or source_frame not in {
            "camera_optical_frame", "wrist_camera_optical_frame",
        }:
            return None
        robot = self._gazebo_robot_pose
        if robot is None:
            return None
        translation = self._param("camera_optical_translation_base")
        rpy = self._param("camera_optical_rpy_base")
        if len(translation) != 3 or len(rpy) != 3:
            return None
        camera_q = _quaternion_from_rpy(*[float(value) for value in rpy])
        robot_q = (robot.orientation.x, robot.orientation.y,
                   robot.orientation.z, robot.orientation.w)
        world_q = _quaternion_multiply(robot_q, camera_q)
        camera_offset = _rotate_point(tuple(float(v) for v in translation), robot_q)
        position = (
            robot.position.x + camera_offset[0],
            robot.position.y + camera_offset[1],
            robot.position.z + camera_offset[2],
        )
        world_to_odom = self._odom_from_gazebo_world_transform(robot)
        if world_to_odom is None:
            return None
        odom_position = _transform_point(position, world_to_odom)
        odom_q = _quaternion_multiply(
            _transform_quaternion(world_to_odom), world_q,
        )
        return _transform_from_components(odom_position, odom_q)

    def _aruco_cb(self, pose):
        """Use ArUco solvePnP pose directly for the marked sample benchmark."""
        evaluation_pose = self._aruco_pose_in_target_frame(pose)
        if (bool(self._param("evaluate_aruco_pnp"))
                and evaluation_pose is not None):
            self._record_estimate("aruco_sample", evaluation_pose, source="aruco_pnp")
        if self.pose_source == "aruco" and pose.header.frame_id == self.target_frame:
            self.selected_pose_pub.publish(pose)

    def _odom_cb(self, msg):
        """Retain the robot base pose expressed in odom for truth conversion."""
        self._odom_robot_pose = msg.pose.pose

    def _aruco_pose_in_target_frame(self, pose):
        if pose.header.frame_id == self.target_frame:
            return pose
        if (pose.header.frame_id == "base_link" and self.target_frame == "odom"
                and self._odom_robot_pose is not None):
            offset = self._param("model_to_base_link_translation_m")
            local = (
                pose.pose.position.x + float(offset[0]),
                pose.pose.position.y + float(offset[1]),
                pose.pose.position.z + float(offset[2]),
            )
            robot = self._odom_robot_pose
            point = _transform_point(local, _pose_as_transform(robot))
            return _pose(pose.header, self.target_frame, point)
        transform = self._lookup_transform(pose.header.frame_id, pose.header.stamp)
        if transform is None:
            return None
        return _pose(pose.header, self.target_frame, _transform_point(
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z), transform))

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
        self._gazebo_robot_pose = robot_pose
        truth_frame = str(self._param("gazebo_truth_frame")).strip()
        use_robot_relative_truth = robot_pose is not None and self.target_frame == "base_link"
        transform = None
        if not use_robot_relative_truth:
            # ModelStates does not publish a ROS TF frame named ``world``.
            # Derive odom <- world from the *same robot pose* represented in
            # Gazebo world and /odom.  This removes the metre-scale bias that
            # resulted from comparing raw world truth with odom estimates.
            if truth_frame == "world" and self.target_frame == "odom":
                transform = self._odom_from_gazebo_world_transform(robot_pose)
            else:
                transform = self._lookup_transform(truth_frame)
            if transform is None:
                self.get_logger().warning(
                    "Gazebo truth has no valid %s -> %s conversion; error logging is paused"
                    % (truth_frame, self.target_frame),
                    throttle_duration_sec=2.0,
                )
                return
        truth_items = []
        for name, pose in zip(msg.name, msg.pose):
            label = model_labels.get(name)
            if label:
                point = (pose.position.x, pose.position.y, pose.position.z)
                # ModelStates is expressed in Gazebo world coordinates.  Its
                # robot model pose gives an exact world->base_link conversion
                # even when this simulator does not publish odom->base_link.
                mapped_point = (
                    _inverse_pose_point(point, robot_pose)
                    if use_robot_relative_truth
                    else _transform_point(point, transform)
                )
                self.gazebo_truth[label] = mapped_point
                truth_items.append({"label": label, "position": mapped_point})
        self.gazebo_truth_items = truth_items

    def _odom_from_gazebo_world_transform(self, world_robot_pose):
        """Return the rigid transform ``odom <- Gazebo world``.

        At one instant, Gazebo provides ``world <- base`` and Nav2 provides
        ``odom <- base``.  Therefore ``odom <- world`` is
        ``(odom <- base) * inverse(world <- base)``.  A full 3-D quaternion
        composition is used so Z and camera pitch are not discarded.
        """
        odom_robot_pose = self._odom_robot_pose
        if world_robot_pose is None or odom_robot_pose is None:
            return None
        world_q = _pose_quaternion(world_robot_pose)
        odom_q = _pose_quaternion(odom_robot_pose)
        rotation = _quaternion_multiply(odom_q, _quaternion_conjugate(world_q))
        world_origin_in_odom = _rotate_point(
            (-world_robot_pose.position.x, -world_robot_pose.position.y, -world_robot_pose.position.z),
            rotation,
        )
        translation = (
            odom_robot_pose.position.x + world_origin_in_odom[0],
            odom_robot_pose.position.y + world_origin_in_odom[1],
            odom_robot_pose.position.z + world_origin_in_odom[2],
        )
        return _transform_from_components(translation, rotation)

    def _grasp_cb(self, msg):
        text = msg.data.strip()
        success = text.lower().startswith(("attached", "success", "grasped"))
        self._write_record({"event": "grasp", "success": success, "failure_reason": "" if success else text})

    def _record_estimate(self, label, pose, source="yolo_rgbd"):
        estimated = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        matched_label, truth = self._match_truth(label, estimated)
        measurement_stamp_sec = (
            float(pose.header.stamp.sec)
            + float(pose.header.stamp.nanosec) * 1e-9
        )
        record = {
            "event": "pose_estimate", "label": label,
            "matched_truth_label": matched_label,
            "truth_association": self._param("evaluation_truth_association"),
            "estimate_source": source,
            "frame_id": self.target_frame, "estimated_position": estimated,
            # RGB-D capture time; CPU inference may publish this result many
            # seconds later, after the robot has reached another station.
            "measurement_stamp_sec": measurement_stamp_sec,
            "gazebo_truth_position": truth,
            # The RGB-D centroid pipeline estimates translation.  It must not
            # claim an object orientation it did not observe from one box.
            "orientation_error_rad": None,
        }
        if truth is not None:
            error = [estimated[i] - truth[i] for i in range(3)]
            record.update({"error_xyz": error, "total_position_error": math.sqrt(sum(value * value for value in error))})
        self._write_record(record)
        if truth is not None:
            message = String()
            message.data = json.dumps(record)
            self.evaluation_pub.publish(message)

    def _match_truth(self, detected_label, estimated):
        """Return the mapped reference object for this localization sample."""
        mode = str(self._param("evaluation_truth_association")).strip().lower()
        if mode == "label":
            return detected_label, self.gazebo_truth.get(detected_label)
        if not self.gazebo_truth_items:
            return None, None
        item = min(
            self.gazebo_truth_items,
            key=lambda candidate: sum(
                (estimated[index] - candidate["position"][index]) ** 2
                for index in range(3)
            ),
        )
        return item["label"], item["position"]

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


def _rotate_point(point, quaternion):
    x, y, z, w = quaternion
    vector = np.asarray(point, dtype=float)
    uv = np.cross((x, y, z), vector)
    uuv = np.cross((x, y, z), uv)
    return tuple(float(value) for value in vector + 2.0 * (w * uv + uuv))


def _quaternion_multiply(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def _pose_quaternion(pose):
    return (
        float(pose.orientation.x), float(pose.orientation.y),
        float(pose.orientation.z), float(pose.orientation.w),
    )


def _transform_quaternion(transform):
    rotation = transform.transform.rotation
    return (rotation.x, rotation.y, rotation.z, rotation.w)


def _transform_from_components(translation, quaternion):
    """Create the minimal TransformStamped-like object used in this module."""
    result = type("SyntheticTransform", (), {})()
    result.transform = type("Transform", (), {})()
    result.transform.translation = type(
        "Translation", (), dict(zip(("x", "y", "z"), translation))
    )()
    result.transform.rotation = type(
        "Rotation", (), dict(zip(("x", "y", "z", "w"), quaternion))
    )()
    return result


def _quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


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


def _pose_as_transform(pose):
    """Adapt a Gazebo Pose to the minimal TransformStamped interface."""
    result = type("PoseTransform", (), {})()
    result.transform = type("Transform", (), {})()
    result.transform.translation = pose.position
    result.transform.rotation = pose.orientation
    return result


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
