#!/usr/bin/env python3
"""Trained-YOLO ROS node retaining the legacy /yolo/detections JSON contract."""
import json
import math
import threading

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from image_pkg.yolo_world_detector import (
    Detection,
    YoloDetector,
    detect_center_foreground_geometry,
    deduplicate_detections,
    select_centered_target_detection,
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
                "aruco_sample", "material_cube_red", "material_cube_green",
                "material_cube_blue", "material_cube_yellow", "aging_rack",
                "pcb_board", "test_tube_rack", "test_tube", "beaker",
                "erlenmeyer_flask", "graduated_cylinder", "tooling_fixture_box",
                "tooling_hand_tools", "board_test_fixture",
                "high_voltage_probe_kit", "material_spare_igbt",
            ],
            # Keep the 17 best.pt labels unchanged end-to-end.
            "class_label_aliases": [""],
            # The custom Gazebo model produces valid object confidences in the
            # 0.08--0.28 range.  Keep this in sync with pose_estimation.yaml;
            # otherwise a direct ``ros2 run`` silently rejects every target.
            "confidence_threshold": 0.05,
            "nms_iou_threshold": 0.45,
            "device": "auto",
            "inference_imgsz": 640,
            "enable_multiscale_recovery": True,
            "recovery_imgsz": 960,
            "enable_tiled_recovery": True,
            "recovery_tile_fraction": 0.65,
            # Ten previously unevaluable classes do produce proposals in the
            # aimed wrist views, but several are below the global 0.05 gate.
            # These thresholds are used only while cruise explicitly names a
            # target.  The selected proposal must also pass the central
            # optical-axis association gate below.
            "active_target_recovery_thresholds": [
                "tooling_hand_tools=0.003",
                "board_test_fixture=0.005",
                "high_voltage_probe_kit=0.010",
                "material_spare_igbt=0.010",
                "aging_rack=0.001",
                "pcb_board=0.010",
                "beaker=0.010",
                "erlenmeyer_flask=0.005",
                "graduated_cylinder=0.001",
                "test_tube=0.010",
            ],
            "active_target_recovery_imgsz": 1280,
            "active_target_roi_fraction": 0.70,
            "active_target_center_gate_fraction": 0.38,
            "filter_to_active_target": True,
            "enable_target_geometry_fallback": True,
            "target_geometry_fallback_labels": [
                "tooling_fixture_box", "tooling_hand_tools",
                "board_test_fixture", "high_voltage_probe_kit",
                "material_spare_igbt", "aging_rack", "pcb_board",
                "test_tube_rack", "test_tube", "beaker",
                "erlenmeyer_flask", "graduated_cylinder",
            ],
            "geometry_color_distance_threshold": 18.0,
            "geometry_override_max_yolo_confidence": 0.02,
            # Only marker 0/1 belong to ``aruco_sample``.  The coloured
            # material cubes deliberately carry marker 2--5; treating every
            # visible marker as aruco_sample caused the servo to lock onto a
            # neighbouring cube and produced metre-scale association errors.
            "aruco_sample_marker_ids": [0, 1],
            "semantic_marker_labels": [
                "0=aruco_sample", "1=aruco_sample",
                "2=material_cube_red", "3=material_cube_green",
                "4=material_cube_blue", "5=material_cube_yellow",
            ],
            "cruise_status_topic": "/image_pkg/cruise/status",
            "camera_info_topic": "/wrist_camera/camera_info",
            "semantic_aruco_pose_topic": "/perception/wrist/semantic_aruco_poses",
            # The 312 px texture has a 240 px coded square on a 70 mm face.
            "aruco_coded_marker_size_m": 0.07 * (240.0 / 312.0),
            # ID 0 uses the legacy 312 px texture (240 px coded square);
            # IDs 1--5 use a 640 px texture with a 512 px coded square.
            # PnP scale follows the rendered code rather than its 70 mm plane.
            "aruco_coded_marker_sizes_m": [
                "0=0.05384615384615385", "1=0.056",
                "2=0.056", "3=0.056", "4=0.056", "5=0.056",
            ],
            # The visual is centred 35.5 mm from the cube origin and is a
            # 1.0 mm thick box.  Its textured outer plane is therefore at
            # 36.0 mm from the model origin.
            "aruco_marker_to_model_origin_m": 0.036,
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
        self.enable_recovery = bool(
            self.get_parameter("enable_multiscale_recovery").value)
        self.recovery_imgsz = int(self.get_parameter("recovery_imgsz").value)
        self.enable_tiled_recovery = bool(
            self.get_parameter("enable_tiled_recovery").value)
        self.recovery_tile_fraction = float(
            self.get_parameter("recovery_tile_fraction").value)
        self.active_target_thresholds = self._parse_thresholds(
            self.get_parameter("active_target_recovery_thresholds").value)
        self.active_target_recovery_imgsz = int(
            self.get_parameter("active_target_recovery_imgsz").value)
        self.active_target_roi_fraction = float(
            self.get_parameter("active_target_roi_fraction").value)
        self.active_target_center_gate_fraction = float(
            self.get_parameter("active_target_center_gate_fraction").value)
        self.filter_to_active_target = bool(
            self.get_parameter("filter_to_active_target").value)
        self.enable_target_geometry_fallback = bool(
            self.get_parameter("enable_target_geometry_fallback").value)
        self.target_geometry_fallback_labels = {
            str(value).strip().lower() for value in
            self.get_parameter("target_geometry_fallback_labels").value
            if str(value).strip()
        }
        self.camera_matrix = None
        self.distortion = None
        self.aruco_sample_marker_ids = {
            int(value) for value in
            self.get_parameter("aruco_sample_marker_ids").value
        }
        self.semantic_marker_labels = {}
        for entry in self.get_parameter("semantic_marker_labels").value:
            marker_id, separator, label = str(entry).partition("=")
            if (separator and marker_id.strip().lstrip("-").isdigit()
                    and label.strip()):
                canonical = self._canonical_label(label.strip())
                if canonical.strip().lower() in self.target_classes:
                    self.semantic_marker_labels[int(marker_id)] = canonical
        self.aruco_marker_sizes = {}
        for entry in self.get_parameter("aruco_coded_marker_sizes_m").value:
            marker_id, separator, raw_size = str(entry).partition("=")
            try:
                size = float(raw_size)
            except ValueError:
                continue
            if (separator and marker_id.strip().lstrip("-").isdigit()
                    and math.isfinite(size) and size > 0.0):
                self.aruco_marker_sizes[int(marker_id)] = size
        self.active_target_label = ""
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
        self.semantic_aruco_pub = self.create_publisher(
            String, self.get_parameter("semantic_aruco_pose_topic").value, 20)
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self._camera_info_cb, qos_profile_sensor_data)
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
        self.create_subscription(
            String,
            self.get_parameter("cruise_status_topic").value,
            self._cruise_status_cb,
            10,
        )

    def _cruise_status_cb(self, msg):
        status = str(msg.data)
        fields = status.split(":")
        if status.startswith("FINE_VIEW_ATTEMPT:") and len(fields) >= 3:
            self.active_target_label = fields[2]
        elif status.startswith("OBSERVATION_BASE:") and len(fields) >= 2:
            self.active_target_label = fields[1]
        elif status.startswith("COARSE_POSITIONING:") and len(fields) >= 3:
            self.active_target_label = fields[2]
        elif status.startswith(("NAVIGATING:", "DONE:", "FAILED:")):
            self.active_target_label = ""

    def _camera_info_cb(self, msg):
        if len(msg.k) == 9 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
            self.distortion = (
                np.asarray(msg.d, dtype=np.float64)
                if len(msg.d) else np.zeros(5, dtype=np.float64))

    def _canonical_target_detections(self, detections):
        values = [
            Detection(
                self._canonical_label(detection.label),
                detection.confidence,
                detection.box,
                getattr(detection, "source", "yolo"),
            )
            for detection in detections
        ]
        return [
            detection for detection in values
            if detection.label.strip().lower() in self.target_classes
        ]

    @staticmethod
    def _parse_thresholds(entries):
        values = {}
        for entry in entries:
            label, separator, threshold = str(entry).partition("=")
            try:
                value = float(threshold)
            except (TypeError, ValueError):
                continue
            if separator and label.strip() and 0.0 < value <= 1.0:
                values[label.strip().lower()] = value
        return values

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
            detections = self._canonical_target_detections(detections)
            detections = self._add_aruco_fallback(image, detections, msg)
            expected = self.active_target_label.strip().lower()
            if expected and expected in self.target_classes:
                # A fine-view target is known before inference.  Recover low
                # confidence same-class proposals both from the full image and
                # a centre crop, then associate by optical-axis proximity.
                # No Gazebo coordinate or projected truth box enters the
                # detector result.
                threshold = self.active_target_thresholds.get(
                    expected, self.confidence)
                if self.enable_recovery and threshold < self.confidence:
                    recovered = self.detector.infer(
                        image, threshold, self.nms_iou,
                        self.active_target_recovery_imgsz)
                    detections.extend(
                        self._canonical_target_detections(recovered))
                    roi_values = self.detector.infer_center_region(
                        image, threshold, self.nms_iou,
                        self.active_target_recovery_imgsz,
                        self.active_target_roi_fraction)
                    detections.extend(
                        self._canonical_target_detections(roi_values))
                detections = deduplicate_detections(detections)
                selected = select_centered_target_detection(
                    detections, expected, image.shape[1], image.shape[0],
                    self.active_target_center_gate_fraction)
                if (self.enable_target_geometry_fallback
                        and expected in self.target_geometry_fallback_labels):
                    geometry = detect_center_foreground_geometry(
                        image,
                        expected,
                        float(self.get_parameter(
                            "geometry_color_distance_threshold").value),
                    )
                    if (geometry is not None and (
                            selected is None
                            or float(selected.confidence) < float(
                                self.get_parameter(
                                    "geometry_override_max_yolo_confidence").value))):
                        selected = geometry
                if self.filter_to_active_target:
                    detections = [selected] if selected is not None else []
            elif self.enable_recovery and not detections:
                recovered = self._canonical_target_detections(
                    self.detector.infer(
                        image, self.confidence, self.nms_iou,
                        self.recovery_imgsz))
                detections.extend(recovered)
                if self.enable_tiled_recovery and not detections:
                    detections.extend(self._canonical_target_detections(
                        self.detector.infer_tiled(
                            image, self.confidence, self.nms_iou,
                            self.recovery_imgsz,
                            self.recovery_tile_fraction)))
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
                        "source": str(getattr(detection, "source", "yolo")),
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
        parameters = (
            aruco.DetectorParameters()
            if hasattr(aruco, "DetectorParameters")
            else aruco.DetectorParameters_create())
        # Use one contrast-tolerant geometry path in normal, dark and
        # reflective profiles. These settings change only marker detection;
        # marker size, camera calibration and PnP geometry remain unchanged.
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 53
        parameters.adaptiveThreshWinSizeStep = 4
        parameters.minMarkerPerimeterRate = 0.015
        if hasattr(aruco, "CORNER_REFINE_SUBPIX"):
            parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            parameters.cornerRefinementWinSize = 7
            parameters.cornerRefinementMaxIterations = 50
            parameters.cornerRefinementMinAccuracy = 0.001
        if hasattr(parameters, "detectInvertedMarker"):
            parameters.detectInvertedMarker = True
        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(dictionary, parameters)
            return lambda gray: detector.detectMarkers(gray)[:2]
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

    def _add_aruco_fallback(self, image, detections, image_msg=None):
        """Add deterministic ArUco boxes for Gazebo marker samples.

        The trained YOLO model remains the detector for unmarked objects.  The fallback
        only supplies the explicitly requested ``aruco_sample`` class, whose
        binary code is more reliable than open-vocabulary inference on the
        low-texture Gazebo rendering.
        """
        if self._aruco_detect is None:
            return detections
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Local contrast normalization reduces threshold and corner drift
        # caused by weak illumination and highlights. Apply it in every
        # profile so the estimator itself stays identical across conditions.
        normalized_gray = cv2.createCLAHE(
            clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        corners, marker_ids = self._aruco_detect(normalized_gray)
        scale = 1
        if marker_ids is None:
            # The bench camera sees the 70 mm marker at a small pixel size.
            # Nearest-neighbour scaling preserves its binary cells.
            scale = 2
            enlarged = cv2.resize(
                normalized_gray, None, fx=scale, fy=scale,
                interpolation=cv2.INTER_NEAREST
            )
            corners, marker_ids = self._aruco_detect(enlarged)
        if marker_ids is None:
            return detections
        fallback = list(detections)
        marker_candidates = {}
        # Keep every visible marker face for 6-D refinement.  Several Gazebo
        # cubes deliberately reuse one ID on their top and side faces, so a
        # camera-space "largest area" decision cannot reliably identify the
        # face whose normal points upward in the map.
        marker_pose_candidates = []
        for corner, marker_id in zip(corners, marker_ids.flatten()):
            label = self.semantic_marker_labels.get(int(marker_id))
            if label is None:
                continue
            points = np.asarray(
                corner.reshape(-1, 2) / scale, dtype=np.float32)
            # ArUco provides integer-ish corners.  Refine them on the original
            # 640x480 source before PnP; this materially improves the depth
            # estimate of a 50--100 px planar marker.
            try:
                refined = points.reshape(-1, 1, 2).copy()
                cv2.cornerSubPix(
                    normalized_gray, refined, (7, 7), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                     50, 0.001))
                points = refined.reshape(-1, 2)
            except cv2.error:
                pass
            x1, y1 = points.min(axis=0).astype(int)
            x2, y2 = points.max(axis=0).astype(int)
            area = int((x2 - x1) * (y2 - y1))
            if area >= 25:
                candidate = Detection(
                    label, 1.0, (x1, y1, x2, y2), "aruco_marker")
                marker_pose_candidates.append(
                    (label, int(marker_id), points.copy(), area))
                previous = marker_candidates.get(label)
                # aruco_sample has ID 1 on its top face and ID 0 on its front
                # face.  Our fine poses are oblique-above, so use the top
                # marker whenever it is visible; selecting the marginally
                # larger front marker makes the face-to-model transform
                # viewpoint-dependent.  ID 0 remains a visibility fallback.
                prefer_top = (
                    label == "aruco_sample" and int(marker_id) == 1
                    and previous is not None and previous[2] != 1)
                retain_top = (
                    label == "aruco_sample" and previous is not None
                    and previous[2] == 1 and int(marker_id) != 1)
                if (previous is None or prefer_top
                        or (not retain_top and area > previous[0])):
                    marker_candidates[label] = (
                        area, candidate, int(marker_id), points.copy())
        # Keep one box for YOLO-style visibility gating.  All marker faces are
        # nevertheless published below so the world-frame evaluator can select
        # the top face by its transformed normal instead of projected area.
        fallback.extend(value[1] for value in marker_candidates.values())
        if image_msg is not None and self.camera_matrix is not None:
            pose_items = []
            for label, marker_id, points, area in marker_pose_candidates:
                value = self._refined_marker_model_origin(
                    marker_id, label, points)
                if value is not None:
                    value["projected_area_px"] = int(area)
                    pose_items.append(value)
            if pose_items:
                output = String()
                output.data = json.dumps({
                    "timestamp": float(
                        image_msg.header.stamp.sec
                        + image_msg.header.stamp.nanosec * 1e-9),
                    "frame_id": str(image_msg.header.frame_id),
                    "poses": pose_items,
                })
                self.semantic_aruco_pub.publish(output)
        return fallback

    def _refined_marker_model_origin(self, marker_id, label, image_points):
        """Solve one square marker and return its model-origin 6D pose."""
        marker_size = self.aruco_marker_sizes.get(
            int(marker_id), float(self.get_parameter(
                "aruco_coded_marker_size_m").value))
        half = marker_size / 2.0
        # IPPE_SQUARE requires TL, TR, BR, BL with +Y pointing upward.
        object_points = np.asarray([
            (-half, half, 0.0), (half, half, 0.0),
            (half, -half, 0.0), (-half, -half, 0.0),
        ], dtype=np.float32)
        try:
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                np.asarray(image_points, dtype=np.float32),
                self.camera_matrix,
                self.distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok or float(tvec[2]) <= 0.0:
                return None
            if hasattr(cv2, "solvePnPRefineLM"):
                rvec, tvec = cv2.solvePnPRefineLM(
                    object_points,
                    np.asarray(image_points, dtype=np.float32),
                    self.camera_matrix,
                    self.distortion,
                    rvec,
                    tvec,
                )
            projected, _ = cv2.projectPoints(
                object_points, rvec, tvec,
                self.camera_matrix, self.distortion)
            reprojection = float(np.sqrt(np.mean(np.sum(
                (projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
            if not math.isfinite(reprojection) or reprojection > 2.0:
                return None
            rotation, _ = cv2.Rodrigues(rvec)
            # OpenCV's marker +Z points out of the printed face toward the
            # camera for the TL/TR/BR/BL convention.  The cube model origin is
            # behind that face, so the marker-to-centre distance is negative
            # along marker +Z.  Adding it used to introduce a deterministic
            # 2*35.5 mm bias.
            centre_offset = rotation @ np.asarray(
                (0.0, 0.0, -abs(float(self.get_parameter(
                    "aruco_marker_to_model_origin_m").value))),
                dtype=np.float64)
            position = tvec.reshape(3).astype(np.float64) + centre_offset
            quaternion = _rotation_matrix_to_quaternion(rotation)
            # Translation from a single square solvePnP is ill-conditioned at
            # oblique views even when corner reprojection error is tiny.  The
            # projective image centre, however, defines a stable calibrated
            # bearing.  Publish that bearing so the world-frame evaluator can
            # intersect it with the known horizontal marker plane.
            unit_square = np.asarray(
                ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)),
                dtype=np.float32)
            homography = cv2.getPerspectiveTransform(
                unit_square, np.asarray(image_points, dtype=np.float32))
            image_centre = cv2.perspectiveTransform(
                np.asarray([[[0.5, 0.5]]], dtype=np.float32),
                homography).reshape(1, 1, 2)
            normalized_centre = cv2.undistortPoints(
                image_centre, self.camera_matrix, self.distortion).reshape(2)
            return {
                "marker_id": int(marker_id),
                "label": str(label),
                "position": [float(value) for value in position],
                # Retain the observed marker centre separately.  The world
                # evaluator can intersect this visual ray with a known
                # horizontal support plane without using noisy planar-PnP
                # depth as the final translation.
                "marker_position_camera": [
                    float(value) for value in tvec.reshape(3)],
                "marker_ray_camera": [
                    float(normalized_centre[0]),
                    float(normalized_centre[1]), 1.0],
                "orientation_xyzw": [float(value) for value in quaternion],
                "reprojection_error_px": reprojection,
                "source": "aruco_pnp_refined",
            }
        except cv2.error:
            return None

    def destroy_node(self):
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def _rotation_matrix_to_quaternion(matrix):
    """Convert a proper 3x3 rotation matrix to normalized XYZW."""
    value = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(value))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray((
            (value[2, 1] - value[1, 2]) / scale,
            (value[0, 2] - value[2, 0]) / scale,
            (value[1, 0] - value[0, 1]) / scale,
            0.25 * scale,
        ))
    else:
        index = int(np.argmax(np.diag(value)))
        if index == 0:
            scale = math.sqrt(1.0 + value[0, 0] - value[1, 1] - value[2, 2]) * 2.0
            quaternion = np.asarray((
                0.25 * scale,
                (value[0, 1] + value[1, 0]) / scale,
                (value[0, 2] + value[2, 0]) / scale,
                (value[2, 1] - value[1, 2]) / scale,
            ))
        elif index == 1:
            scale = math.sqrt(1.0 + value[1, 1] - value[0, 0] - value[2, 2]) * 2.0
            quaternion = np.asarray((
                (value[0, 1] + value[1, 0]) / scale,
                0.25 * scale,
                (value[1, 2] + value[2, 1]) / scale,
                (value[0, 2] - value[2, 0]) / scale,
            ))
        else:
            scale = math.sqrt(1.0 + value[2, 2] - value[0, 0] - value[1, 1]) * 2.0
            quaternion = np.asarray((
                (value[0, 2] + value[2, 0]) / scale,
                (value[1, 2] + value[2, 1]) / scale,
                0.25 * scale,
                (value[1, 0] - value[0, 1]) / scale,
            ))
    norm = float(np.linalg.norm(quaternion))
    return quaternion / norm if norm > 0.0 else np.asarray((0.0, 0.0, 0.0, 1.0))


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
