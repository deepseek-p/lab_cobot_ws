#!/usr/bin/env python3
"""Validate a Tsai eye-in-hand calibration pipeline in Gazebo."""

import math
import threading
import time

import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from lab_cobot_manipulation.pick_place_node import OBSERVE_CONFIG, PickPlace


MARKER_ID = 1
MARKER_SIZE_M = 0.07 * (512.0 / 640.0)
MIN_VALID_SAMPLES = 15
TARGET_BASE_POSE = (2.0, 0.62, math.pi / 2.0)
MOVE_SETTLE_SEC = 0.6
FRAME_WAIT_SEC = 3.0


def _sample_configs():
    base = list(OBSERVE_CONFIG)
    configs = []
    for pan_delta in (0.04, 0.08, 0.12):
        for wrist_3 in (-0.30, 0.30, 0.0):
            for wrist_1 in (0.0, -0.30, -0.15):
                config = list(base)
                config[0] += pan_delta
                config[3] += wrist_1
                config[5] += wrist_3
                configs.append(config)
    return configs


def _yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _transform_matrix(transform, np):
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [transform.translation.x, transform.translation.y,
                     transform.translation.z]
    return matrix


class CalibrationProbe(Node):
    def __init__(self):
        super().__init__(
            "hand_eye_calibration_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_stamp = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.models = None
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Image, "/wrist_camera/image_raw", self._image_cb, 10)
        self.create_subscription(
            CameraInfo, "/wrist_camera/camera_info", self._info_cb, 10
        )
        self.create_subscription(ModelStates, "/gazebo/model_states", self._models_cb, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _image_cb(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.latest_stamp = msg.header.stamp

    def _info_cb(self, msg):
        import numpy as np

        self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(msg.d, dtype=np.float64)

    def _models_cb(self, msg):
        self.models = msg

    def _model_pose(self, name):
        if self.models is None or name not in self.models.name:
            return None
        index = self.models.name.index(name)
        pose = self.models.pose[index]
        return pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)

    def _publish_cmd(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def drive_to_station_a(self, timeout_sec=80.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            pose = self._model_pose("lab_cobot")
            if pose is None:
                time.sleep(0.05)
                continue
            x, y, yaw = pose
            ex = TARGET_BASE_POSE[0] - x
            ey = TARGET_BASE_POSE[1] - y
            eyaw = _wrap(TARGET_BASE_POSE[2] - yaw)
            if math.hypot(ex, ey) < 0.012 and abs(eyaw) < 0.02:
                for _ in range(10):
                    self._publish_cmd(0.0, 0.0, 0.0)
                    time.sleep(0.05)
                return True
            vx_world = max(-0.28, min(0.28, 0.9 * ex))
            vy_world = max(-0.28, min(0.28, 0.9 * ey))
            cosine = math.cos(yaw)
            sine = math.sin(yaw)
            self._publish_cmd(
                cosine * vx_world + sine * vy_world,
                -sine * vx_world + cosine * vy_world,
                max(-0.55, min(0.55, 1.2 * eyaw)),
            )
            time.sleep(0.05)
        return False

    def lookup_matrix(self, target, source, np):
        transform = self.tf_buffer.lookup_transform(
            target,
            source,
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0),
        )
        return _transform_matrix(transform.transform, np)

    def wait_for_pnp(self, newer_than_sec, cv2, np):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        deadline = time.monotonic() + FRAME_WAIT_SEC
        rotation_vectors = []
        translation_vectors = []
        while time.monotonic() < deadline:
            if self.latest_image is None or self.latest_stamp is None:
                time.sleep(0.03)
                continue
            stamp_sec = self.latest_stamp.sec + self.latest_stamp.nanosec * 1.0e-9
            if stamp_sec <= newer_than_sec:
                time.sleep(0.03)
                continue
            corners, ids, _ = detector.detectMarkers(self.latest_image)
            id_list = [] if ids is None else ids.flatten().tolist()
            if MARKER_ID not in id_list:
                time.sleep(0.03)
                continue
            image_points = corners[id_list.index(MARKER_ID)].reshape(4, 2)
            half = MARKER_SIZE_M / 2.0
            object_points = np.array([
                [-half, half, 0.0], [half, half, 0.0],
                [half, -half, 0.0], [-half, -half, 0.0],
            ], dtype=np.float32)
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points.astype(np.float32),
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not ok:
                time.sleep(0.03)
                continue
            rotation_vectors.append(rvec.reshape(3))
            translation_vectors.append(tvec.reshape(3))
            newer_than_sec = stamp_sec
            if len(rotation_vectors) < 5:
                time.sleep(0.03)
                continue
            mean_rvec = np.mean(np.asarray(rotation_vectors), axis=0)
            mean_tvec = np.mean(np.asarray(translation_vectors), axis=0)
            rotation, _ = cv2.Rodrigues(mean_rvec)
            matrix = np.eye(4, dtype=np.float64)
            matrix[:3, :3] = rotation
            matrix[:3, 3] = mean_tvec
            return matrix
        return None


def _rotation_error_deg(estimated, truth, cv2, np):
    delta = estimated[:3, :3] @ truth[:3, :3].T
    rvec, _ = cv2.Rodrigues(delta)
    return math.degrees(float(np.linalg.norm(rvec)))


def _motion_audit(base_tool_samples, cv2, np):
    axis_total = np.zeros(3, dtype=np.float64)
    step_angles = []
    for previous, current in zip(base_tool_samples, base_tool_samples[1:]):
        relative = np.linalg.inv(previous) @ current
        rvec, _ = cv2.Rodrigues(relative[:3, :3])
        vector = rvec.reshape(3)
        axis_total += np.abs(vector)
        step_angles.append(math.degrees(float(np.linalg.norm(vector))))
    return np.degrees(axis_total), step_angles


def main():
    """Collect informative motions and validate Tsai calibration twice."""
    import cv2
    import numpy as np

    rclpy.init()
    probe = CalibrationProbe()
    pick_place = PickPlace()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(probe)
    executor.add_node(pick_place)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    base_tool_samples = []
    camera_marker_samples = []
    try:
        ready_deadline = time.monotonic() + 20.0
        while time.monotonic() < ready_deadline:
            if probe.camera_matrix is not None and probe._model_pose("lab_cobot"):
                break
            time.sleep(0.1)
        if not probe.drive_to_station_a():
            raise RuntimeError("Failed to drive base to station A")
        print("BASE_READY", probe._model_pose("lab_cobot"), flush=True)

        for index, config in enumerate(_sample_configs(), 1):
            pick_place.moveit2.move_to_configuration(config)
            moved = pick_place.moveit2.wait_until_executed(timeout_sec=45.0)
            if not moved:
                print(f"SAMPLE {index:02d} invalid move_failed", flush=True)
                continue
            time.sleep(MOVE_SETTLE_SEC)
            start_stamp = probe.get_clock().now().to_msg()
            start_sec = start_stamp.sec + start_stamp.nanosec * 1.0e-9
            camera_marker = probe.wait_for_pnp(start_sec, cv2, np)
            if camera_marker is None:
                print(f"SAMPLE {index:02d} invalid marker_not_visible", flush=True)
                continue
            link_optical = probe.lookup_matrix(
                "wrist_camera_link", "wrist_camera_optical_frame", np
            )
            camera_marker = link_optical @ camera_marker
            base_tool = probe.lookup_matrix("base_link", "ur_tool0", np)
            base_tool_samples.append(base_tool)
            camera_marker_samples.append(camera_marker)
            print(
                f"SAMPLE {index:02d} valid t_cam_marker="
                f"{camera_marker[:3, 3].round(6).tolist()}",
                flush=True,
            )

        print("VALID_SAMPLES", len(base_tool_samples), flush=True)
        if len(base_tool_samples) < MIN_VALID_SAMPLES:
            raise RuntimeError("Not enough valid poses; dataset is void and must be recollected")
        axis_total, step_angles = _motion_audit(base_tool_samples, cv2, np)
        print("ROTATION_AXIS_TOTAL_DEG", axis_total.round(2).tolist(), flush=True)
        print("ROTATION_STEP_DEG", [round(value, 2) for value in step_angles], flush=True)
        if max(step_angles, default=0.0) <= 30.0 or np.any(axis_total < 30.0):
            raise RuntimeError("Not enough informative motions; recollect without bypassing")

        rotations_gripper_to_base = [sample[:3, :3] for sample in base_tool_samples]
        translations_gripper_to_base = [sample[:3, 3] for sample in base_tool_samples]
        rotations_target_to_camera = [sample[:3, :3] for sample in camera_marker_samples]
        translations_target_to_camera = [sample[:3, 3] for sample in camera_marker_samples]
        # Eye-in-hand requires end->base (T_base_tool) here. Passing the inverse
        # is the most common calibrateHandEye direction error.
        rotation_camera_to_tool, translation_camera_to_tool = cv2.calibrateHandEye(
            rotations_gripper_to_base,
            translations_gripper_to_base,
            rotations_target_to_camera,
            translations_target_to_camera,
            method=cv2.CALIB_HAND_EYE_TSAI,
        )
        estimated = np.eye(4, dtype=np.float64)
        estimated[:3, :3] = rotation_camera_to_tool
        estimated[:3, 3] = translation_camera_to_tool.reshape(3)
        truth = probe.lookup_matrix("ur_tool0", "wrist_camera_link", np)
        translation_error_mm = 1000.0 * float(
            np.linalg.norm(estimated[:3, 3] - truth[:3, 3])
        )
        rotation_error_deg = _rotation_error_deg(estimated, truth, cv2, np)
        print("ESTIMATED_TOOL_CAMERA", estimated.round(8).tolist(), flush=True)
        print("URDF_TRUTH_TOOL_CAMERA", truth.round(8).tolist(), flush=True)
        print("TRUTH_ERROR_MM_DEG", round(translation_error_mm, 3),
              round(rotation_error_deg, 3), flush=True)

        marker_corner = np.array(
            [-MARKER_SIZE_M / 2.0, MARKER_SIZE_M / 2.0, 0.0, 1.0],
            dtype=np.float64,
        )
        base_points = []
        for base_tool, camera_marker in zip(base_tool_samples, camera_marker_samples):
            camera_point = camera_marker @ marker_corner
            base_points.append((base_tool @ estimated @ camera_point)[:3])
        point_std_mm = 1000.0 * np.std(np.asarray(base_points), axis=0)
        print("FIXED_POINT_STD_MM", point_std_mm.round(3).tolist(), flush=True)
        if translation_error_mm >= 5.0 or rotation_error_deg >= 2.0:
            raise RuntimeError("Calibration exceeds the truth-error acceptance band")
    finally:
        for _ in range(10):
            probe._publish_cmd(0.0, 0.0, 0.0)
            time.sleep(0.03)
        executor.shutdown()
        pick_place.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
