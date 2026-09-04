#!/usr/bin/env python3
"""Independently validate the wrist hand-eye TF against Gazebo link truth.

The perception benchmark must not tune an extrinsic with the same object poses
used for its error score.  During several distinct arm configurations this
node compares the ROS chain ``world<-base_footprint<-wrist_camera_link`` with
Gazebo's independently published ``world<-wrist_camera_link`` pose.  It writes
only validation residuals and never publishes a correction into perception.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from gazebo_msgs.msg import LinkStates, ModelStates
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def _quat_multiply(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _rotate(vector, quaternion):
    xyz = np.asarray(quaternion[:3], dtype=float)
    value = np.asarray(vector, dtype=float)
    uv = np.cross(xyz, value)
    uuv = np.cross(xyz, uv)
    return value + 2.0 * (quaternion[3] * uv + uuv)


def _angle_rad(first, second):
    dot = abs(float(np.dot(first, second)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _pose_quaternion(pose):
    return np.asarray((pose.orientation.x, pose.orientation.y,
                       pose.orientation.z, pose.orientation.w), dtype=float)


class WristHandEyeValidator(Node):
    def __init__(self):
        super().__init__("wrist_hand_eye_validator")
        self.declare_parameter("robot_model_name", "lab_cobot")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("camera_link_frame", "wrist_camera_link")
        # Gazebo collapses the fixed wrist_camera_link into its movable
        # parent, so link_states exposes ur_wrist_3_link.  Compose the fixed
        # parent->camera mount before comparing with the complete ROS chain.
        self.declare_parameter("gazebo_camera_link_suffix", "::ur_wrist_3_link")
        self.declare_parameter("gazebo_truth_link_frame", "ur_wrist_3_link")
        self.declare_parameter("required_distinct_poses", 6)
        self.declare_parameter("min_pose_translation_delta_m", 0.025)
        self.declare_parameter("min_pose_rotation_delta_deg", 5.0)
        self.declare_parameter(
            "output_path", "/tmp/image_pkg_wrist_hand_eye_validation.json")
        self.declare_parameter("max_translation_error_m", 0.002)
        self.declare_parameter("max_rotation_error_deg", 0.25)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("max_stationary_joint_speed_rps", 0.03)
        self.declare_parameter("stationary_dwell_seconds", 0.40)
        self.robot_pose = None
        self.camera_truth_pose = None
        self.samples = []
        self.relative_poses = []
        self.finished = False
        self.stationary_since = None
        self.last_joint_positions = {}
        self.last_joint_time = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.status_pub = self.create_publisher(
            String, "/image_pkg/calibration/wrist_hand_eye_validation", 10)
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self._models_cb, 10)
        self.create_subscription(
            LinkStates, "/gazebo/link_states", self._links_cb, 10)
        self.create_subscription(
            JointState, str(self.get_parameter("joint_state_topic").value),
            self._joint_state_cb, 20)
        self.create_timer(0.2, self._sample)

    def _models_cb(self, msg):
        poses = dict(zip(msg.name, msg.pose))
        self.robot_pose = poses.get(str(
            self.get_parameter("robot_model_name").value))

    def _links_cb(self, msg):
        suffix = str(self.get_parameter("gazebo_camera_link_suffix").value)
        for name, pose in zip(msg.name, msg.pose):
            if str(name).endswith(suffix):
                self.camera_truth_pose = pose
                return

    def _joint_state_cb(self, msg):
        now = time.monotonic()
        dt = (None if self.last_joint_time is None
              else now - self.last_joint_time)
        velocities = []
        current = {}
        for name, position in zip(msg.name, msg.position):
            if not (str(name).startswith("ur_")
                    and str(name).endswith("_joint")):
                continue
            value = float(position)
            current[str(name)] = value
            if dt is not None and dt > 1e-4 and name in self.last_joint_positions:
                delta = math.atan2(
                    math.sin(value - self.last_joint_positions[name]),
                    math.cos(value - self.last_joint_positions[name]))
                velocities.append(abs(delta / dt))
        self.last_joint_positions = current
        self.last_joint_time = now
        stationary = (
            len(velocities) >= 6
            and max(velocities) <= float(self.get_parameter(
                "max_stationary_joint_speed_rps").value))
        if stationary:
            if self.stationary_since is None:
                self.stationary_since = time.monotonic()
        else:
            self.stationary_since = None

    def _sample(self):
        if (self.finished or self.robot_pose is None
                or self.camera_truth_pose is None
                or self.stationary_since is None
                or time.monotonic() - self.stationary_since < float(
                    self.get_parameter("stationary_dwell_seconds").value)):
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("robot_base_frame").value),
                str(self.get_parameter("camera_link_frame").value),
                Time(), timeout=Duration(seconds=0.1))
            truth_link_from_camera = self.tf_buffer.lookup_transform(
                str(self.get_parameter("gazebo_truth_link_frame").value),
                str(self.get_parameter("camera_link_frame").value),
                Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        relative_position = np.asarray(
            (translation.x, translation.y, translation.z), dtype=float)
        relative_quaternion = np.asarray(
            (rotation.x, rotation.y, rotation.z, rotation.w), dtype=float)
        relative_quaternion /= np.linalg.norm(relative_quaternion)
        if self.relative_poses:
            distinct = all(
                (np.linalg.norm(relative_position - old_position)
                 >= float(self.get_parameter(
                     "min_pose_translation_delta_m").value)
                 or math.degrees(_angle_rad(relative_quaternion, old_quaternion))
                 >= float(self.get_parameter(
                     "min_pose_rotation_delta_deg").value))
                for old_position, old_quaternion in self.relative_poses)
            if not distinct:
                return
        robot_q = _pose_quaternion(self.robot_pose)
        robot_q /= np.linalg.norm(robot_q)
        predicted_position = np.asarray((
            self.robot_pose.position.x,
            self.robot_pose.position.y,
            self.robot_pose.position.z,
        )) + _rotate(relative_position, robot_q)
        predicted_q = np.asarray(_quat_multiply(robot_q, relative_quaternion))
        predicted_q /= np.linalg.norm(predicted_q)
        truth_link_position = np.asarray((
            self.camera_truth_pose.position.x,
            self.camera_truth_pose.position.y,
            self.camera_truth_pose.position.z,
        ))
        truth_link_q = _pose_quaternion(self.camera_truth_pose)
        truth_link_q /= np.linalg.norm(truth_link_q)
        fixed_t = truth_link_from_camera.transform.translation
        fixed_q_msg = truth_link_from_camera.transform.rotation
        fixed_position = np.asarray((fixed_t.x, fixed_t.y, fixed_t.z))
        fixed_q = np.asarray((fixed_q_msg.x, fixed_q_msg.y,
                              fixed_q_msg.z, fixed_q_msg.w))
        fixed_q /= np.linalg.norm(fixed_q)
        truth_position = truth_link_position + _rotate(
            fixed_position, truth_link_q)
        truth_q = np.asarray(_quat_multiply(truth_link_q, fixed_q))
        truth_q /= np.linalg.norm(truth_q)
        translation_error = float(np.linalg.norm(
            predicted_position - truth_position))
        rotation_error = math.degrees(_angle_rad(predicted_q, truth_q))
        self.relative_poses.append((relative_position, relative_quaternion))
        self.samples.append({
            "translation_error_m": translation_error,
            "rotation_error_deg": rotation_error,
            "base_to_camera_translation_m": relative_position.tolist(),
            "base_to_camera_quaternion_xyzw": relative_quaternion.tolist(),
        })
        required = max(3, int(self.get_parameter(
            "required_distinct_poses").value))
        self.get_logger().info(
            "hand-eye validation pose %d/%d: %.3f mm, %.4f deg" % (
                len(self.samples), required, translation_error * 1000.0,
                rotation_error))
        if len(self.samples) >= required:
            self._finish()

    def _finish(self):
        translations = np.asarray([
            sample["translation_error_m"] for sample in self.samples])
        rotations = np.asarray([
            sample["rotation_error_deg"] for sample in self.samples])
        result = {
            "method": "independent Gazebo link truth vs ROS wrist hand-eye TF",
            "distinct_pose_samples": len(self.samples),
            "translation_error_mean_m": float(np.mean(translations)),
            "translation_error_max_m": float(np.max(translations)),
            "rotation_error_mean_deg": float(np.mean(rotations)),
            "rotation_error_max_deg": float(np.max(rotations)),
            "translation_threshold_m": float(self.get_parameter(
                "max_translation_error_m").value),
            "rotation_threshold_deg": float(self.get_parameter(
                "max_rotation_error_deg").value),
            "passed": bool(
                np.max(translations) <= float(self.get_parameter(
                    "max_translation_error_m").value)
                and np.max(rotations) <= float(self.get_parameter(
                    "max_rotation_error_deg").value)),
            "samples": self.samples,
        }
        path = Path(str(self.get_parameter("output_path").value)).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        message = String()
        message.data = json.dumps(result, ensure_ascii=False)
        self.status_pub.publish(message)
        self.get_logger().info("hand-eye validation written: %s" % path)
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = WristHandEyeValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
