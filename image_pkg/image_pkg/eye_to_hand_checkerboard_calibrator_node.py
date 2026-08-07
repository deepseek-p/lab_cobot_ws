#!/usr/bin/env python3
"""6D eye-to-hand calibration from a Gazebo checkerboard observation.

The bench camera in this project is fixed to the robot base (eye-to-hand), not
mounted on the wrist.  For every chessboard observation this node obtains
``T_camera_board`` via OpenCV solvePnP and combines it with Gazebo's known
``T_world_board`` and ``T_world_base`` to recover::

    T_base_camera = inv(T_world_base) * T_world_board * inv(T_camera_board)

Gazebo truth is used only for the calibration target pose.  It is never sent
to the perception/grasp topics.  The output contains a 6D extrinsic and
repeatability spread across observations.
"""
from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


def _matrix_from_pose(pose):
    q = pose.orientation
    rotation = _rotation_from_quaternion((q.x, q.y, q.z, q.w))
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return matrix


def _rotation_from_quaternion(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = (x / norm, y / norm, z / norm, w / norm)
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def _quaternion_from_rotation(rotation):
    # Stable matrix-to-quaternion conversion, xyzw order used by ROS.
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale)
    axis = int(np.argmax(np.diag(rotation)))
    if axis == 0:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        return (0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[2, 1] - rotation[1, 2]) / scale)
    if axis == 1:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        return ((rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale)
    scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
    return ((rotation[0, 2] + rotation[2, 0]) / scale, (rotation[1, 2] + rotation[2, 1]) / scale,
            0.25 * scale, (rotation[1, 0] - rotation[0, 1]) / scale)


def _rpy_from_rotation(rotation):
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    if abs(math.cos(pitch)) > 1e-7:
        return (math.atan2(rotation[2, 1], rotation[2, 2]), pitch,
                math.atan2(rotation[1, 0], rotation[0, 0]))
    return (math.atan2(-rotation[1, 2], rotation[1, 1]), pitch, 0.0)


def _rotation_error_deg(reference, observed):
    relative = reference.T @ observed
    cosine = min(1.0, max(-1.0, (float(np.trace(relative)) - 1.0) * 0.5))
    return math.degrees(math.acos(cosine))


def _matrix_from_rpy_translation(translation, rpy):
    roll, pitch, yaw = (float(value) for value in rpy)
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ])
    matrix[:3, 3] = [float(value) for value in translation]
    return matrix


def _extrinsic_distance(reference, observed):
    return (np.linalg.norm(observed[:3, 3] - reference[:3, 3])
            + 0.02 * math.radians(
                _rotation_error_deg(reference[:3, :3], observed[:3, :3])))


class EyeToHandCheckerboardCalibrator(Node):
    def __init__(self):
        super().__init__("eye_to_hand_checkerboard_calibrator")
        self.declare_parameter("image_topic", "/bench_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/bench_camera/camera_info")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("board_model_name", "checkerboard_9_7_0_03")
        self.declare_parameter("robot_model_name", "lab_cobot")
        # Gazebo ModelStates reports the robot model root (base_footprint),
        # while the camera extrinsic is defined from base_link.  This URDF has
        # a fixed +155 mm model-root -> base_link offset.
        self.declare_parameter("model_to_base_link_translation_m", [0.0, 0.0, 0.155])
        self.declare_parameter("inner_corners_cols", 8)
        self.declare_parameter("inner_corners_rows", 6)
        self.declare_parameter("square_size_m", 0.03)
        self.declare_parameter("samples", 20)
        self.declare_parameter("output_dir", "~/.ros")
        # A checkerboard is planar and has a 180-degree front/back ambiguity.
        # This nominal URDF transform selects the physically mounted branch;
        # it is not averaged into the measured result.
        self.declare_parameter("nominal_translation_m", [0.18, -0.22, 1.225])
        self.declare_parameter("nominal_rpy_rad", [-2.221, 0.0, -1.571])
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion = None
        self.board_pose = None
        self.base_pose = None
        self.samples = []
        self._reference_extrinsic = None
        self._done = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.object_points = self._make_object_points()
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, str(self.get_parameter("image_topic").value), self._image_cb, qos_profile_sensor_data)
        self.create_subscription(ModelStates, str(self.get_parameter("model_states_topic").value), self._models_cb, 10)
        self.get_logger().info("6D checkerboard calibration ready: 9x7 squares, 8x6 inner corners, 30 mm squares")

    def _make_object_points(self):
        cols = int(self.get_parameter("inner_corners_cols").value)
        rows = int(self.get_parameter("inner_corners_rows").value)
        size = float(self.get_parameter("square_size_m").value)
        # The generator centres the 9x7 square grid at its model origin.
        xs = (np.arange(cols, dtype=np.float32) - (cols - 1) * 0.5) * size
        ys = (np.arange(rows, dtype=np.float32) - (rows - 1) * 0.5) * size
        return np.asarray([(x, y, 0.0) for y in ys for x in xs], dtype=np.float32)

    def _camera_info_cb(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.camera_matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
            self.distortion = np.asarray(msg.d, dtype=float)

    def _models_cb(self, msg):
        poses = dict(zip(msg.name, msg.pose))
        self.board_pose = poses.get(str(self.get_parameter("board_model_name").value))
        self.base_pose = poses.get(str(self.get_parameter("robot_model_name").value))

    def _image_cb(self, msg):
        if self._done or self.camera_matrix is None or self.board_pose is None or self.base_pose is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().warning("image conversion failed: %s" % exc)
            return
        pattern = (int(self.get_parameter("inner_corners_cols").value), int(self.get_parameter("inner_corners_rows").value))
        found, corners = cv2.findChessboardCornersSB(image, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            return
        ok, rvec, tvec = cv2.solvePnP(self.object_points, corners, self.camera_matrix, self.distortion, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return
        camera_board = np.eye(4)
        camera_board[:3, :3], _ = cv2.Rodrigues(rvec)
        camera_board[:3, 3] = tvec.reshape(3)
        world_board = _matrix_from_pose(self.board_pose)
        # A generated plane is double-sided, so evaluate both face conventions
        # and both in-plane 180-degree corner orders before choosing a branch.
        visible_side = np.eye(4)
        visible_side[:3, :3] = np.diag((-1.0, 1.0, -1.0))
        world_base = self._world_base_from_model()
        half_turn = np.eye(4)
        half_turn[:3, :3] = np.diag((-1.0, -1.0, 1.0))
        candidates = tuple(
            np.linalg.inv(world_base) @ (world_board @ face @ turn) @ np.linalg.inv(camera_board)
            for face in (np.eye(4), visible_side)
            for turn in (np.eye(4), half_turn)
        )
        if self._reference_extrinsic is None:
            nominal = _matrix_from_rpy_translation(
                self.get_parameter("nominal_translation_m").value,
                self.get_parameter("nominal_rpy_rad").value)
            base_camera = min(candidates, key=lambda item: _extrinsic_distance(nominal, item))
            self._reference_extrinsic = base_camera
        else:
            base_camera = min(
                candidates,
                key=lambda item: _extrinsic_distance(self._reference_extrinsic, item),
            )
        self.samples.append(base_camera)
        if len(self.samples) >= max(3, int(self.get_parameter("samples").value)):
            self._write_result()
            self._done = True
            rclpy.shutdown()

    def _world_base_from_model(self):
        """Return world<-base_link without mixing Gazebo world and ROS odom."""
        matrix = _matrix_from_pose(self.base_pose)
        offset = self.get_parameter("model_to_base_link_translation_m").value
        if len(offset) == 3:
            local = np.eye(4)
            local[:3, 3] = [float(item) for item in offset]
            matrix = matrix @ local
        return matrix

    def _write_result(self):
        translations = np.asarray([sample[:3, 3] for sample in self.samples])
        translation = np.median(translations, axis=0)
        rotations = [sample[:3, :3] for sample in self.samples]
        reference_rotation = rotations[len(rotations) // 2]
        rotation_errors = [_rotation_error_deg(reference_rotation, item) for item in rotations]
        rpy = _rpy_from_rotation(reference_rotation)
        result = {
            "method": "6D fixed-camera eye-to-hand calibration: inv(T_world_base) * T_world_board * inv(T_camera_board)",
            "board": {"squares": [9, 7], "inner_corners": [8, 6], "square_size_m": 0.03},
            "samples": len(self.samples),
            "base_to_camera_translation_m": translation.tolist(),
            "base_to_camera_quaternion_xyzw": list(_quaternion_from_rotation(reference_rotation)),
            "base_to_camera_rpy_rad": list(rpy),
            "translation_median_absolute_deviation_m": np.median(np.abs(translations - translation), axis=0).tolist(),
            "rotation_max_deviation_deg": max(rotation_errors),
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        }
        path = self.output_dir / ("eye_to_hand_checkerboard_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info("6D extrinsic written: %s" % path)


def main(args=None):
    rclpy.init(args=args)
    node = EyeToHandCheckerboardCalibrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
