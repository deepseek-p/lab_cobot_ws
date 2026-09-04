#!/usr/bin/env python3
"""Measure grasp alignment between the robot TCP and a workpiece in Gazebo.

This is an *execution* metric, deliberately separate from the RGB-D/YOLO
metric: on every successful simulated attach it compares the truth workpiece
pose to ``gripper_tcp`` in the TCP coordinate frame.  A sample is therefore
only produced after a real grasp acknowledgement, rather than from arbitrary
camera frames.
"""
from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def _quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def _quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + az * bw + ax * by - ay * bx,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(q, v):
    # q * (v, 0) * conjugate(q), expanded to avoid a geometry dependency.
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _norm(v):
    return math.sqrt(sum(value * value for value in v))


def _angle_deg(q):
    length = math.sqrt(sum(value * value for value in q))
    if length <= 1e-12:
        return float("nan")
    return math.degrees(2.0 * math.acos(min(1.0, abs(q[3] / length))))


class TcpPrecisionBenchmark(Node):
    def __init__(self):
        super().__init__("tcp_precision_benchmark")
        self.declare_parameter("object_name", "aruco_sample")
        self.declare_parameter("tcp_frame", "gripper_tcp")
        self.declare_parameter("reference_frame", "odom")
        self.declare_parameter("attach_status_topic", "/gripper/attach/status")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("nominal_object_offset_tcp", [0.0, 0.0, -0.06])
        self.declare_parameter("nominal_object_orientation_tcp", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("pass_threshold_mm", 1.0)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("exit_after_samples", 1)

        self.object_name = str(self.get_parameter("object_name").value)
        self.tcp_frame = str(self.get_parameter("tcp_frame").value)
        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.nominal_offset = tuple(float(x) for x in self.get_parameter("nominal_object_offset_tcp").value)
        self.nominal_orientation = tuple(float(x) for x in self.get_parameter("nominal_object_orientation_tcp").value)
        self.threshold_mm = float(self.get_parameter("pass_threshold_mm").value)
        self.exit_after = max(1, int(self.get_parameter("exit_after_samples").value))
        configured_dir = str(self.get_parameter("output_dir").value)
        default_dir = Path(__file__).resolve().parents[1] / "tcp_precision_results"
        self.output_dir = Path(configured_dir) if configured_dir else default_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.model_pose = None
        self.samples = []
        self._finished = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(ModelStates, str(self.get_parameter("model_states_topic").value), self._on_models, 10)
        self.create_subscription(String, str(self.get_parameter("attach_status_topic").value), self._on_attach_status, 10)
        self.get_logger().info("TCP precision benchmark ready: object=%s, nominal offset=%s m" % (self.object_name, self.nominal_offset))

    def _on_models(self, msg):
        try:
            self.model_pose = msg.pose[msg.name.index(self.object_name)]
        except ValueError:
            return

    def _on_attach_status(self, msg):
        if self._finished or not msg.data.startswith("attached ") or self.object_name not in msg.data:
            return
        if self.model_pose is None:
            self.get_logger().warn("attach acknowledged but no workpiece truth pose received")
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame, self.tcp_frame, rclpy.time.Time(), timeout=Duration(seconds=0.25)
            )
        except TransformException as exc:
            self.get_logger().warn("cannot sample TCP transform: %s" % exc)
            return

        tcp_t = transform.transform.translation
        tcp_q_msg = transform.transform.rotation
        tcp_q = (tcp_q_msg.x, tcp_q_msg.y, tcp_q_msg.z, tcp_q_msg.w)
        obj = self.model_pose
        delta_world = (obj.position.x - tcp_t.x, obj.position.y - tcp_t.y, obj.position.z - tcp_t.z)
        actual_offset = _rotate(_quat_conjugate(tcp_q), delta_world)
        obj_q = (obj.orientation.x, obj.orientation.y, obj.orientation.z, obj.orientation.w)
        actual_relative_q = _quat_multiply(_quat_conjugate(tcp_q), obj_q)
        # Relative orientation residual: inv(nominal) * actual.
        residual_q = _quat_multiply(_quat_conjugate(self.nominal_orientation), actual_relative_q)
        residual = tuple(actual_offset[i] - self.nominal_offset[i] for i in range(3))
        error_mm = _norm(residual) * 1000.0
        sample = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "translation_error_mm": error_mm,
            "translation_error_xyz_mm": [value * 1000.0 for value in residual],
            "orientation_error_deg": _angle_deg(residual_q),
            "actual_object_offset_tcp_m": list(actual_offset),
            "nominal_object_offset_tcp_m": list(self.nominal_offset),
            "pass_1mm": error_mm <= self.threshold_mm,
        }
        self.samples.append(sample)
        self.get_logger().info("grasp sample %d: %.3f mm, %.3f deg" % (len(self.samples), error_mm, sample["orientation_error_deg"]))
        if len(self.samples) >= self.exit_after:
            self._write_summary()
            self._finished = True
            rclpy.shutdown()

    def _write_summary(self):
        errors = [item["translation_error_mm"] for item in self.samples]
        angles = [item["orientation_error_deg"] for item in self.samples]
        result = {
            "metric": "TCP-to-workpiece relative grasp pose error",
            "reference": "Gazebo ModelStates workpiece truth; TCP from TF",
            "object_name": self.object_name,
            "sample_condition": "only /gripper/attach/status=attached",
            "pass_threshold_mm": self.threshold_mm,
            "samples": len(self.samples),
            "mean_translation_error_mm": sum(errors) / len(errors),
            "max_translation_error_mm": max(errors),
            "mean_orientation_error_deg": sum(angles) / len(angles),
            "pass_rate": sum(item["pass_1mm"] for item in self.samples) / len(self.samples),
            "details": self.samples,
        }
        path = self.output_dir / ("tcp_precision_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info("result written: %s" % path)


def main(args=None):
    rclpy.init(args=args)
    node = TcpPrecisionBenchmark()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
