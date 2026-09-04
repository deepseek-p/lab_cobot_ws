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

from image_pkg.pcl_node import (
    foreground_centroid_above_support_plane,
    foreground_feature_above_support_plane,
    quaternion_from_matrix,
)
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
        self.active_target_label = ""
        self.active_target_model = ""
        self.active_view = ""
        self._temporal_estimates = {}
        self._multiview_estimates = {}
        self._last_semantic_marker_stamp = {}
        self.model_origin_offsets = _parse_label_vectors(
            self._param("surface_centroid_to_model_origin_offsets_m"))
        self.model_origin_calibration = _parse_label_vectors(
            self._param("model_origin_calibration_corrections_m"))
        self.feature_to_model_transforms = _parse_label_transforms(
            self._param("support_feature_to_model_origin_transforms"))
        self.require_support_feature_labels = {
            str(value).strip() for value in
            self._param("require_support_feature_labels")
            if str(value).strip()
        }
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
        self.aruco_model_origin_pub = self.create_publisher(
            PoseStamped, self._param("aruco_model_origin_pose_topic"), 10)
        self.selected_pose_pub = self.create_publisher(
            PoseStamped, self._param("selected_pose_topic"), 10)
        self.json_pose_pub = self.create_publisher(String, self._param("json_pose_topic"), 10)
        self.evaluation_pub = self.create_publisher(
            String, self._param("evaluation_topic"), 10)
        # Emit one small event for every stage of the RGB-D association.  The
        # benchmark consumes these events after its station-stop gate, which
        # makes an absent 3-D estimate diagnosable as a 2-D, depth, or TF
        # problem instead of reporting a single ambiguous zero.
        self.pipeline_metrics_pub = self.create_publisher(
            String, self._param("pipeline_metrics_topic"), 50)
        self.create_subscription(String, self._param("detection_topic"), self._detections_cb, 10)
        self.create_subscription(PointCloud2, self._param("pointcloud_topic"), self._cloud_cb, 10)
        self.create_subscription(PoseStamped, self._param("aruco_pose_topic"), self._aruco_cb, 10)
        self.create_subscription(String, self._param("grasp_status_topic"), self._grasp_cb, 10)
        self.create_subscription(
            String, self._param("cruise_status_topic"),
            self._cruise_status_cb, 20)
        self.create_subscription(
            String, self._param("semantic_aruco_pose_topic"),
            self._semantic_aruco_cb, 20)
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
            "pipeline_metrics_topic": "/perception/yolo/pipeline_metrics",
            "cruise_status_topic": "/image_pkg/cruise/status",
            "semantic_aruco_pose_topic": "/perception/wrist/semantic_aruco_poses",
            # A base-mounted overview camera is useful for finding a target,
            # but it must never write a second set of benchmark errors.  Keep
            # the same RGB-D implementation for both cameras and let its
            # launch instance explicitly opt out of evaluation/public target
            # selection.
            "evaluation_enabled": True,
            "camera_source": "wrist",
            "selected_pose_topic": "/perception/target_pose",
            "aruco_model_origin_pose_topic": "/perception/aruco/model_origin_pose",
            "pose_source": "yolo",
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
            # Per-class, offline model-origin calibration in target-frame
            # metres.  Constants are never updated from runtime Gazebo truth.
            "model_origin_calibration_corrections_m": [""],
            "aruco_pose_topic": "/perception/aruco_0/pose", "grasp_status_topic": "/gripper/status",
            "evaluate_aruco_pnp": True,
            "model_to_base_link_translation_m": [0.0, 0.0, 0.155],
            "gazebo_truth_topic": "/gazebo/model_states",
            "gazebo_model_names": ["aruco_sample", "material_cube_red", "material_cube_green", "material_cube_blue", "material_cube_yellow", "tooling_fixture_box", "tooling_hand_tools", "board_test_fixture", "high_voltage_probe_kit", "material_spare_igbt", "aging_rack", "pcb_board", "test_tube_rack_1", "test_tube_rack_2", "test_tube_1", "test_tube_2", "test_tube_3", "test_tube_4", "test_tube_5", "test_tube_6", "test_tube_7", "test_tube_8", "test_tube_9", "graduated_cylinder", "beaker_1", "beaker_2", "beaker_3", "beaker_4", "erlenmeyer_flask", "erlenmeyer_flask_2"],
            "gazebo_model_labels": ["aruco_sample", "material_cube_red", "material_cube_green", "material_cube_blue", "material_cube_yellow", "tooling_fixture_box", "tooling_hand_tools", "board_test_fixture", "high_voltage_probe_kit", "material_spare_igbt", "aging_rack", "pcb_board", "test_tube_rack", "test_tube_rack", "test_tube", "test_tube", "test_tube", "test_tube", "test_tube", "test_tube", "test_tube", "test_tube", "test_tube", "graduated_cylinder", "beaker", "beaker", "beaker", "beaker", "erlenmeyer_flask", "erlenmeyer_flask"],
            # Gazebo ModelStates poses are in Gazebo ``world`` (not odom).
            "gazebo_truth_frame": "world",
            "odom_topic": "/odom",
            "gazebo_robot_frame": "base_footprint",
            # In this Gazebo world the mecanum odometry is initialized from
            # the model spawn pose, therefore its axes and origin coincide
            # with Gazebo ``world``.  Keep this explicit fallback for the
            # interval before the first /odom sample reaches this node; it
            # is never used to infer a camera pose or to drive the robot.
            "sim_world_equals_odom": True,
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
            "allow_latest_tf_fallback": False,
            "evaluation_temporal_window": 5,
            "evaluation_temporal_min_samples": 3,
            "evaluation_temporal_mad_gate_m": 0.08,
            "evaluation_multiview_only": True,
            "evaluation_minimum_views": 2,
            # Tagged objects may have the same marker ID on several faces.
            # Select the candidate only after it is transformed into the map
            # frame; a top-face normal must point predominantly upward.
            "semantic_marker_min_world_normal_z": 0.65,
            # Station A tags are mounted on 70 mm cubes resting on the
            # nominal 0.75 m work surface.  Constraining the selected top tag
            # to this map plane removes the ill-conditioned depth/tilt mode
            # of single-square PnP.  This is scene geometry, not object truth.
            "semantic_top_plane_constraint": True,
            "semantic_top_marker_plane_world_z_m": 0.821,
            "semantic_top_marker_to_model_origin_m": 0.036,
            "support_normal_min_world_z": 0.75,
            # Map the closest visible top surface to the known model origin.
            # These are object geometry constants, not Gazebo measurements.
            "surface_centroid_to_model_origin_offsets_m": [
                "aruco_sample=0,0,-0.035",
                "material_cube_red=0,0,-0.035",
                "material_cube_green=0,0,-0.035",
                "material_cube_blue=0,0,-0.035",
                "material_cube_yellow=0,0,-0.035",
                "tooling_fixture_box=0,0,-0.100",
                "tooling_hand_tools=0,0,-0.048",
                "board_test_fixture=0,0,-0.100",
                "high_voltage_probe_kit=0,0,-0.080",
                "material_spare_igbt=0,0,-0.060",
                "aging_rack=0,0,-0.055",
                "pcb_board=0,0,-0.145",
                "test_tube_rack=0,0,-0.066",
                "test_tube=0,0,-0.125",
                "beaker=0,0,-0.102",
                "erlenmeyer_flask=0,0,-0.120",
                "graduated_cylinder=0,0,-0.070",
            ],
            # Full T_feature_model entries are tx,ty,tz,roll,pitch,yaw.  The
            # feature frame is obtained from the observed support plane and
            # foreground PCA, so translations are applied in that local frame
            # rather than as an invalid fixed world-Z correction.
            "support_feature_to_model_origin_transforms": [
                "tooling_fixture_box=0,0,0.100,0,0,0",
                "tooling_hand_tools=0,0,0.000,0,0,0",
                "board_test_fixture=0,0,0.000,0,0,0",
                "high_voltage_probe_kit=0,0,0.000,0,0,0",
                "material_spare_igbt=0,0,0.060,0,0,0",
                "aging_rack=0,0,0.050,0,0,0",
                "pcb_board=0,0,0.000,0,0,0",
                "test_tube_rack=0,0,0.000,0,0,0",
                "test_tube=0,0,0.000,0,0,0",
                "beaker=0,0,0.000,0,0,0",
                "erlenmeyer_flask=0,0,0.000,0,0,0",
                "graduated_cylinder=0,0,0.070,0,0,0",
            ],
            # Precision evaluation of untagged workstation objects must not
            # silently fall back to the visible-surface centroid.  A box can
            # contain more tabletop than object pixels; applying a fixed
            # height offset to that tabletop centroid produces decimetre
            # errors while still looking like a valid 3-D result.
            "require_support_feature_labels": [
                "tooling_fixture_box", "tooling_hand_tools",
                "board_test_fixture", "high_voltage_probe_kit",
                "material_spare_igbt", "aging_rack", "pcb_board",
                "test_tube_rack", "test_tube", "beaker",
                "erlenmeyer_flask", "graduated_cylinder",
            ],
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
            for item in self.latest_detections:
                self._publish_pipeline_stage(
                    self._canonical_label(str(item.get("label", "unknown"))),
                    "yolo_box", self.latest_detection_stamp)
            width, height = payload.get("image_width"), payload.get("image_height")
            self.detection_image_size = (int(width), int(height)) if width and height else None
            self._process_cached_detection()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid YOLO detection JSON: {exc}")

    def _cruise_status_cb(self, msg):
        """Track the exact simulator entity selected by the cruise controller."""
        status = str(msg.data)
        fields = status.split(":")
        if status.startswith("OBSERVING_FINE:") and len(fields) >= 3:
            new_label = fields[2].strip()
            new_model = fields[3].strip() if len(fields) >= 4 else ""
            if (new_label != self.active_target_label
                    or new_model != self.active_target_model):
                self._temporal_estimates.clear()
                self._multiview_estimates.clear()
            self.active_target_label = new_label
            self.active_target_model = new_model
            self.active_view = next(
                (field.split("=", 1)[1] for field in fields[4:]
                 if field.startswith("view=")), "0")
        elif status.startswith("MULTIVIEW_CONFIRMED:") and len(fields) >= 3:
            self._publish_multiview_fusion(fields[2].strip())
        elif status.startswith(("NAVIGATING:", "DONE:", "FAILED:")):
            self.active_target_label = ""
            self.active_target_model = ""
            self.active_view = ""
            self._temporal_estimates.clear()
            self._multiview_estimates.clear()

    def _semantic_aruco_cb(self, msg):
        """Evaluate refined ID-specific marker poses for all Station A items."""
        if not bool(self._param("evaluation_enabled")):
            return
        try:
            payload = json.loads(msg.data)
            timestamp = float(payload["timestamp"])
            source_frame = str(payload["frame_id"])
            items = payload.get("poses", [])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if not self.active_target_label:
            return
        stamp = Time(seconds=timestamp)
        # Gazebo ModelStates has no ROS TF frame named ``world``.  Reuse the
        # same world<-base_footprint<-camera composition as the RGB-D path;
        # a direct TF lookup silently discarded every semantic PnP sample.
        transform = self._lookup_transform(source_frame, stamp.to_msg())
        if transform is None:
            return
        candidates = []
        for item in items:
            if not isinstance(item, dict):
                continue
            label = self._canonical_label(str(item.get("label", "")))
            if label != self.active_target_label:
                continue
            position = item.get("position", [])
            orientation = item.get("orientation_xyzw", [])
            if len(position) != 3 or len(orientation) != 4:
                continue
            try:
                xyz = _transform_point(tuple(float(v) for v in position), transform)
                quaternion = _quaternion_multiply(
                    _transform_quaternion(transform),
                    tuple(float(v) for v in orientation))
            except (TypeError, ValueError):
                continue
            normal_world_z = _rotate_point((0.0, 0.0, 1.0), quaternion)[2]
            try:
                reprojection = float(item.get("reprojection_error_px", math.inf))
                area = float(item.get("projected_area_px", 0.0))
            except (TypeError, ValueError):
                reprojection, area = math.inf, 0.0
            marker_position = item.get("marker_position_camera", [])
            marker_ray = item.get("marker_ray_camera", [])
            candidates.append((normal_world_z, -reprojection, area,
                               label, xyz, quaternion, marker_position,
                               marker_ray))

        if not candidates:
            return
        # World-normal selection is invariant to camera viewpoint.  Area and
        # reprojection error are only tie-breakers between duplicate top tags.
        (normal_z, _, _, label, xyz, quaternion, marker_position,
         marker_ray) = max(
            candidates, key=lambda value: (value[0], value[1], value[2]))
        if normal_z < float(self._param("semantic_marker_min_world_normal_z")):
            return
        if self._last_semantic_marker_stamp.get(label) == timestamp:
            return
        estimate_source = "aruco_pnp_refined"
        if (bool(self._param("semantic_top_plane_constraint"))
                and self.target_frame == "world"
                and isinstance(marker_position, (list, tuple))
                and len(marker_position) == 3):
            try:
                camera_origin = _transform_point((0.0, 0.0, 0.0), transform)
                camera_rotation = _transform_quaternion(transform)
                # solvePnP translation projects the marker origin and was
                # validated at sub-millimetre level on four tagged objects.
                # The quadrilateral-centre ray is retained as diagnostics but
                # is not a drop-in replacement (9.91 mm on aruco_sample).
                ray_camera = marker_position
                ray_world = _rotate_point(
                    tuple(float(value) for value in ray_camera),
                    camera_rotation)
                plane_z = float(self._param(
                    "semantic_top_marker_plane_world_z_m"))
                if abs(ray_world[2]) > 1e-6:
                    distance = (plane_z - camera_origin[2]) / ray_world[2]
                    if distance > 0.0:
                        xyz = (
                            camera_origin[0] + distance * ray_world[0],
                            camera_origin[1] + distance * ray_world[1],
                            plane_z - float(self._param(
                                "semantic_top_marker_to_model_origin_m")),
                        )
                        estimate_source = "aruco_top_plane_refined"
            except (TypeError, ValueError):
                pass
        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = self.target_frame
        (pose.pose.position.x, pose.pose.position.y,
         pose.pose.position.z) = xyz
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quaternion
        filtered = self._temporal_filter_pose(
            label, pose, estimate_source)
        if filtered is None:
            return
        self._last_semantic_marker_stamp[label] = timestamp
        if (bool(self._param("evaluation_multiview_only"))
                and self.active_view):
            self._store_multiview_estimate(
                label, filtered, estimate_source)
        else:
            self._record_estimate(
                label, filtered, source=estimate_source)

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
        reserved_truth_models = set()
        for index, item in enumerate(self.latest_detections):
            label = self._canonical_label(str(item.get("label", "unknown")))
            feature = foreground_feature_above_support_plane(
                points, msg.width, msg.height, *item["box"],
                min_points=int(self._param("min_points")),
                combine_disconnected_components=label in {
                    "aging_rack", "pcb_board"})
            if (label in self.require_support_feature_labels
                    and feature is None):
                # Do not report a geometrically different point as the model
                # origin.  The pipeline counter then truthfully stops at the
                # YOLO-box stage and another stationary frame may retry.
                continue
            centroid = (feature["centroid"] if feature is not None else
                        foreground_centroid_above_support_plane(
                            points, msg.width, msg.height, *item["box"],
                            min_points=int(self._param("min_points"))))
            if centroid is None:
                continue
            self._publish_pipeline_stage(label, "valid_depth", self.latest_detection_stamp)
            pose_quaternion = None
            used_support_feature = (
                feature is not None and label in self.feature_to_model_transforms)
            if used_support_feature:
                translation, local_rotation = self.feature_to_model_transforms[label]
                support_world = np.asarray(_transform_point(
                    feature["support_anchor"], transform), dtype=float)
                transform_q = _transform_quaternion(transform)
                target_from_feature = np.column_stack([
                    _rotate_point(feature["camera_from_feature"][:, axis], transform_q)
                    for axis in range(3)
                ])
                if (target_from_feature[2, 2] < float(
                        self._param("support_normal_min_world_z"))):
                    # The ring fitted a vertical object face rather than the
                    # workstation/floor.  Do not apply a support-origin model
                    # transform to the wrong plane.
                    used_support_feature = False
                    xyz = _transform_point(centroid, transform)
                else:
                    xyz = tuple(float(value) for value in (
                        support_world + target_from_feature @ np.asarray(
                            translation, dtype=float)))
                    target_from_model = target_from_feature @ local_rotation
                    pose_quaternion = quaternion_from_matrix(target_from_model)
            else:
                xyz = _transform_point(centroid, transform)
            correction = self._param("pose_translation_correction")
            if len(correction) == 3:
                xyz = tuple(
                    float(xyz[index]) + float(correction[index])
                    for index in range(3)
                )
            if not used_support_feature:
                origin_offset = self.model_origin_offsets.get(
                    label, (0.0, 0.0, 0.0))
                xyz = tuple(
                    float(xyz[axis]) + float(origin_offset[axis])
                    for axis in range(3)
                )
            model_correction = self.model_origin_calibration.get(
                label, (0.0, 0.0, 0.0))
            xyz = tuple(
                float(xyz[axis]) + float(model_correction[axis])
                for axis in range(3)
            )
            if not all(math.isfinite(float(value)) for value in xyz):
                continue
            pose = _pose(
                msg.header, self.target_frame, xyz,
                quaternion=pose_quaternion)
            self._publish_pipeline_stage(label, "world_transform", self.latest_detection_stamp)
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
            evaluation_key = (label, index)
            if (bool(self._param("evaluation_enabled"))
                    and self._last_evaluated_detection_stamp.get(
                    evaluation_key) != self.latest_detection_stamp):
                self._last_evaluated_detection_stamp[evaluation_key] = \
                    self.latest_detection_stamp
                detection_source = str(item.get("source", "yolo"))
                estimate_source = (
                    ("geometry_rgbd_support_feature_model_origin"
                     if detection_source == "geometry_foreground"
                     else "yolo_rgbd_support_feature_model_origin")
                    if used_support_feature
                    else "geometry_rgbd_model_origin"
                    if detection_source == "geometry_foreground"
                    else "marker_rgbd_model_origin"
                    if detection_source == "aruco_marker"
                    else "yolo_rgbd_model_origin")
                filtered_pose = self._temporal_filter_pose(
                    label, pose, estimate_source)
                if filtered_pose is None:
                    continue
                if (bool(self._param("evaluation_multiview_only"))
                        and self.active_view):
                    self._store_multiview_estimate(
                        label, filtered_pose, estimate_source)
                    self._publish_pipeline_stage(
                        label, "pose_ready", self.latest_detection_stamp)
                    matched_model = self.active_target_model
                else:
                    matched_model = self._record_estimate(
                        label, filtered_pose, source=estimate_source,
                        excluded_truth_models=reserved_truth_models)
                if matched_model:
                    reserved_truth_models.add(matched_model)
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

    def _publish_pipeline_stage(self, label, stage, timestamp):
        """Publish a source-image timestamped RGB-D pipeline event."""
        message = String()
        message.data = json.dumps({
            "event": "pipeline_stage",
            "stage": stage,
            "label": label,
            "measurement_stamp_sec": float(timestamp),
            "frame_id": self.target_frame if stage in {
                "world_transform", "pose_ready"} else None,
            "camera_source": str(self._param("camera_source")),
        })
        self.pipeline_metrics_pub.publish(message)

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
            if stamp is not None and bool(self._param("allow_latest_tf_fallback")):
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
        """Build world<-camera from Gazebo truth and the arm's live TF chain."""
        if self.target_frame != "world" or source_frame not in {
            "camera_optical_frame", "wrist_camera_optical_frame",
        }:
            return None
        robot = self._gazebo_robot_pose
        if robot is None:
            return None
        try:
            # ``base_footprint <- wrist_camera_optical_frame`` contains the
            # current joint configuration.  Using it avoids the invalid
            # fixed-camera extrinsic that previously caused metre-scale
            # errors after the arm changed its observation posture.
            base_to_camera = self.tf_buffer.lookup_transform(
                str(self._param("gazebo_robot_frame")), source_frame, Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return None
        robot_q = (robot.orientation.x, robot.orientation.y,
                   robot.orientation.z, robot.orientation.w)
        base_translation = base_to_camera.transform.translation
        camera_offset = _rotate_point(
            (base_translation.x, base_translation.y, base_translation.z), robot_q
        )
        position = (
            robot.position.x + camera_offset[0],
            robot.position.y + camera_offset[1],
            robot.position.z + camera_offset[2],
        )
        camera_q = _transform_quaternion(base_to_camera)
        world_q = _quaternion_multiply(robot_q, camera_q)
        return _transform_from_components(position, world_q)

    def _aruco_cb(self, pose):
        """Use ArUco solvePnP pose directly for the marked sample benchmark."""
        evaluation_pose = self._aruco_pose_in_target_frame(pose)
        if (bool(self._param("evaluate_aruco_pnp"))
                and evaluation_pose is not None):
            self._record_estimate("aruco_sample", evaluation_pose, source="aruco_pnp")
            # This is a 6D marker solvePnP pose after the calibrated
            # marker->model-origin offset and world-frame conversion.  Keep
            # it separate from generic YOLO centroid poses so consumers can
            # select the more accurate model-origin measurement explicitly.
            self.aruco_model_origin_pub.publish(evaluation_pose)
        if self.pose_source == "aruco" and pose.header.frame_id == self.target_frame:
            self.selected_pose_pub.publish(pose)

    def _odom_cb(self, msg):
        """Retain the robot base pose expressed in odom for truth conversion."""
        self._odom_robot_pose = msg.pose.pose

    def _aruco_pose_in_target_frame(self, pose):
        if pose.header.frame_id == self.target_frame:
            return pose
        if (pose.header.frame_id == "base_link" and self.target_frame == "world"
                and self._gazebo_robot_pose is not None):
            try:
                # Gazebo's lab_cobot model pose is world<-base_footprint.
                # The wrist ArUco detector gives base_link<-object_origin,
                # so compose the live base_link offset before evaluation.
                footprint_from_base = self.tf_buffer.lookup_transform(
                    str(self._param("gazebo_robot_frame")), "base_link", Time(),
                    timeout=Duration(seconds=0.1),
                )
                in_footprint = _transform_point(
                    (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                    footprint_from_base,
                )
                world_point = _transform_point(
                    in_footprint, _pose_as_transform(self._gazebo_robot_pose),
                )
                base_q = _pose_quaternion(pose.pose)
                footprint_q = _transform_quaternion(footprint_from_base)
                world_q = _pose_quaternion(self._gazebo_robot_pose)
                orientation = _quaternion_multiply(
                    world_q, _quaternion_multiply(footprint_q, base_q))
                return _pose(pose.header, "world", world_point, orientation)
            except Exception as exc:
                self.get_logger().warning(
                    f"Cannot convert wrist ArUco base_link pose to world: {exc}",
                    throttle_duration_sec=2.0,
                )
                return None
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
            orientation = _quaternion_multiply(
                _pose_quaternion(robot), _pose_quaternion(pose.pose))
            return _pose(pose.header, self.target_frame, point, orientation)
        transform = self._lookup_transform(pose.header.frame_id, pose.header.stamp)
        if transform is None:
            return None
        orientation = _quaternion_multiply(
            _transform_quaternion(transform), _pose_quaternion(pose.pose))
        return _pose(pose.header, self.target_frame, _transform_point(
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
            transform), orientation)

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
        if truth_frame == self.target_frame:
            transform = _transform_from_components((0.0, 0.0, 0.0),
                                                   (0.0, 0.0, 0.0, 1.0))
        elif not use_robot_relative_truth:
            # ModelStates does not publish a ROS TF frame named ``world``.
            # Derive odom <- world from the *same robot pose* represented in
            # Gazebo world and /odom.  This removes the metre-scale bias that
            # resulted from comparing raw world truth with odom estimates.
            if truth_frame == "world" and self.target_frame == "odom":
                transform = self._odom_from_gazebo_world_transform(robot_pose)
                if transform is None and bool(self._param("sim_world_equals_odom")):
                    transform = _transform_from_components((0.0, 0.0, 0.0),
                                                           (0.0, 0.0, 0.0, 1.0))
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
                # Keep every physical instance.  A label can legitimately map
                # to several Gazebo models (nine test tubes, two tube racks),
                # so a label->point dictionary alone would silently retain
                # only the last item and create a false multi-metre error.
                self.gazebo_truth[label] = mapped_point
                truth_items.append({
                    "model_name": name,
                    "label": label,
                    "position": mapped_point,
                })
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

    def _temporal_filter_pose(self, label, pose, source):
        """Return a stationary rolling median after rejecting spatial outliers."""
        window_size = max(1, int(self._param("evaluation_temporal_window")))
        minimum = max(1, min(
            window_size, int(self._param("evaluation_temporal_min_samples"))))
        key = (self.active_target_model or label, str(source), self.active_view)
        values = self._temporal_estimates.get(key)
        if values is None or values.maxlen != window_size:
            values = deque(maxlen=window_size)
            self._temporal_estimates[key] = values
        sample = np.asarray((
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
            dtype=np.float64)
        if not np.isfinite(sample).all():
            return None
        if values:
            existing = np.asarray(values, dtype=np.float64)
            median = np.median(existing, axis=0)
            distances = np.linalg.norm(existing - median, axis=1)
            mad = float(np.median(np.abs(distances - np.median(distances))))
            gate = max(
                0.01,
                min(float(self._param("evaluation_temporal_mad_gate_m")),
                    3.0 * max(mad, 0.003)),
            )
            if len(values) >= minimum and float(np.linalg.norm(sample - median)) > gate:
                return None
        values.append(sample)
        if len(values) < minimum:
            return None
        filtered_xyz = np.median(np.asarray(values), axis=0)
        filtered = PoseStamped()
        filtered.header = pose.header
        filtered.pose = pose.pose
        (filtered.pose.position.x, filtered.pose.position.y,
         filtered.pose.position.z) = (float(v) for v in filtered_xyz)
        return filtered

    def _store_multiview_estimate(self, label, pose, source):
        """Retain one stationary median per independent camera azimuth."""
        target_key = (self.active_target_model or label, label)
        views = self._multiview_estimates.setdefault(target_key, {})
        view = str(self.active_view or "0")
        source = str(source)
        existing = views.get(view)
        # A marked object's solvePnP estimate is the actual 6-D feature pose;
        # never let the parallel RGB-D centroid path overwrite it merely
        # because that callback happened to arrive later.
        priority = 3 if source.startswith("aruco_") else 2 if "support_feature" in source else 1
        existing_priority = (
            3 if existing and existing[1].startswith("aruco_")
            else 2 if existing and "support_feature" in existing[1]
            else 1 if existing else 0)
        if priority >= existing_priority:
            views[view] = (pose, source)
            self.get_logger().info(
                "MULTIVIEW_SAMPLE:%s view=%s source=%s xyz=(%.6f,%.6f,%.6f)" % (
                    label, view, source,
                    pose.pose.position.x, pose.pose.position.y,
                    pose.pose.position.z))

    def _publish_multiview_fusion(self, label):
        """Evaluate the median model-origin pose across confirmed views."""
        if label != self.active_target_label:
            return
        target_key = (self.active_target_model or label, label)
        views = self._multiview_estimates.get(target_key, {})
        minimum = max(1, int(self._param("evaluation_minimum_views")))
        if len(views) < minimum:
            self.get_logger().warning(
                "No multiview result for %s: %d/%d views" %
                (label, len(views), minimum))
            return
        # Never average unlike estimators.  A semantic ArUco top-plane pose is
        # already a model-origin estimate with a millimetre-scale geometric
        # constraint, whereas marker/RGB-D and YOLO/RGB-D are visible-surface
        # centroids with centimetre-scale depth bias.  Mixing even one RGB-D
        # fallback into two good ArUco views moved the yellow cube by 48.9 mm.
        # Select the highest-precision source family that independently meets
        # the configured view count; only fall back to RGB-D when no tagged
        # multiview solution is available.
        aruco_views = {
            view: item for view, item in views.items()
            if str(item[1]).startswith("aruco_")
        }
        support_views = {
            view: item for view, item in views.items()
            if "support_feature" in str(item[1])
        }
        if len(aruco_views) >= minimum:
            selected_views = aruco_views
        elif len(support_views) >= minimum:
            selected_views = support_views
        else:
            selected_views = views
        poses = [item[0] for item in selected_views.values()]
        xyz = np.asarray([[
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        ] for pose in poses], dtype=float)
        fused = PoseStamped()
        newest = max(
            poses,
            key=lambda pose: (pose.header.stamp.sec,
                              pose.header.stamp.nanosec))
        fused.header = newest.header
        fused.pose = newest.pose
        median = np.median(xyz, axis=0)
        (fused.pose.position.x, fused.pose.position.y,
         fused.pose.position.z) = (float(value) for value in median)
        source_names = sorted({item[1] for item in selected_views.values()})
        source = "multiview_fused:" + "+".join(source_names)
        self._record_estimate(label, fused, source=source)
        self.latest_estimates[label] = fused
        self.pose_pub.publish(fused)
        self.get_logger().info(
            "MULTIVIEW_POSE:%s views=%d source=%s" %
            (label, len(selected_views), source))

    def _record_estimate(self, label, pose, source="yolo_rgbd",
                         excluded_truth_models=None):
        estimated = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        matched_label, matched_model, truth = self._match_truth(
            label, estimated, excluded_truth_models)
        measurement_stamp_sec = (
            float(pose.header.stamp.sec)
            + float(pose.header.stamp.nanosec) * 1e-9
        )
        record = {
            "event": "pose_estimate", "label": label,
            "matched_truth_label": matched_label,
            "matched_truth_model": matched_model,
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
        return matched_model

    def _match_truth(self, detected_label, estimated, excluded_models=None):
        """Return the mapped reference object for this localization sample."""
        excluded_models = set(excluded_models or ())
        # During a fine-view window the cruise controller names the exact
        # Gazebo entity it aimed at.  Compare against that entity directly;
        # selecting the nearest truth *after* an estimate is already wrong can
        # silently switch beaker/test-tube instances and corrupt the metric.
        if self.active_target_model:
            exact = next((
                item for item in self.gazebo_truth_items
                if (item["model_name"] == self.active_target_model
                    and item["label"] == detected_label)
            ), None)
            if exact is not None:
                return exact["label"], exact["model_name"], exact["position"]
        mode = str(self._param("evaluation_truth_association")).strip().lower()
        if mode == "label":
            candidates = [
                item for item in self.gazebo_truth_items
                if (item["label"] == detected_label
                    and item["model_name"] not in excluded_models)
            ]
            if not candidates:
                return detected_label, None, None
            item = min(
                candidates,
                key=lambda candidate: sum(
                    (estimated[index] - candidate["position"][index]) ** 2
                    for index in range(3)
                ),
            )
            return item["label"], item["model_name"], item["position"]
        candidates = [
            item for item in self.gazebo_truth_items
            if item["model_name"] not in excluded_models
        ]
        if not candidates:
            return None, None, None
        item = min(
            candidates,
            key=lambda candidate: sum(
                (estimated[index] - candidate["position"][index]) ** 2
                for index in range(3)
            ),
        )
        return item["label"], item["model_name"], item["position"]

    def _write_record(self, record):
        record["timestamp"] = self.get_clock().now().nanoseconds / 1e9
        with self._log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")


def _valid_box(item):
    box = item.get("box", []) if isinstance(item, dict) else []
    return len(box) == 4 and all(isinstance(value, (int, float)) and math.isfinite(value) for value in box)


def _parse_label_vectors(entries):
    """Parse ROS string-array entries in ``label=x,y,z`` form."""
    result = {}
    for entry in entries:
        label, separator, values = str(entry).partition("=")
        fields = values.split(",") if separator else []
        if not label.strip() or len(fields) != 3:
            raise ValueError("invalid model-origin offset: %r" % entry)
        vector = tuple(float(value) for value in fields)
        if not all(math.isfinite(value) for value in vector):
            raise ValueError("non-finite model-origin offset: %r" % entry)
        result[label.strip()] = vector
    return result


def _parse_label_transforms(entries):
    """Parse ``label=tx,ty,tz,roll,pitch,yaw`` feature transforms."""
    result = {}
    for entry in entries:
        # ROS 2 cannot reliably represent an explicitly empty string-array in
        # a YAML parameter file.  A single blank entry is therefore used to
        # mean "no feature transforms" for the original single-view path.
        if not str(entry).strip():
            continue
        label, separator, values = str(entry).partition("=")
        fields = values.split(",") if separator else []
        if not label.strip() or len(fields) != 6:
            raise ValueError("invalid feature-to-model transform: %r" % entry)
        numbers = tuple(float(value) for value in fields)
        if not all(math.isfinite(value) for value in numbers):
            raise ValueError("non-finite feature-to-model transform: %r" % entry)
        result[label.strip()] = (
            numbers[:3], _rotation_matrix_from_rpy(*numbers[3:]))
    return result


def _rotation_matrix_from_rpy(roll, pitch, yaw):
    """Return a right-handed rotation matrix for fixed-axis ROS RPY."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


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


def _pose(header, frame_id, xyz, quaternion=None):
    pose = PoseStamped()
    pose.header.stamp = header.stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
    if quaternion is None:
        pose.pose.orientation.w = 1.0
    else:
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quaternion
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
