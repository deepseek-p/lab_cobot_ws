#!/usr/bin/env python3
"""Estimate the translation residual of the camera-to-base transform.

The node pairs an online RGB-D target pose with the corresponding Gazebo model
truth.  It is intentionally an *offline calibration aid*: truth is never
published to the manipulation target topic.  A 6D hand-eye rotation cannot be
identified from a YOLO centroid, so this tool only estimates the observable
three-dimensional translation residual.
"""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class CameraBaseResidualCalibrator(Node):
    def __init__(self):
        super().__init__("camera_base_residual_calibrator")
        self.declare_parameter("pose_topic", "/perception/target_pose")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("object_name", "aruco_sample")
        self.declare_parameter("samples", 30)
        self.declare_parameter("max_residual_m", 0.20)
        self.declare_parameter("output_dir", "")
        self.object_name = str(self.get_parameter("object_name").value)
        self.sample_limit = max(3, int(self.get_parameter("samples").value))
        self.max_residual_m = float(self.get_parameter("max_residual_m").value)
        configured_dir = str(self.get_parameter("output_dir").value)
        self.output_dir = Path(configured_dir) if configured_dir else Path(__file__).resolve().parents[1] / "calibration_results"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.truth = None
        self.residuals = []
        self._done = False
        self.create_subscription(ModelStates, str(self.get_parameter("model_states_topic").value), self._truth_cb, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self._pose_cb, 10)
        self.get_logger().info("camera/base residual calibration: object=%s samples=%d" % (self.object_name, self.sample_limit))

    def _truth_cb(self, msg):
        try:
            pose = msg.pose[msg.name.index(self.object_name)]
        except ValueError:
            return
        self.truth = np.asarray((pose.position.x, pose.position.y, pose.position.z), dtype=float)

    def _pose_cb(self, msg):
        if self._done or self.truth is None or msg.header.frame_id not in {"odom", "world"}:
            return
        estimate = np.asarray((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z), dtype=float)
        residual = self.truth - estimate
        if float(np.linalg.norm(residual)) > self.max_residual_m:
            self.get_logger().warning("rejecting outlier residual %s m" % residual.tolist())
            return
        self.residuals.append(residual)
        if len(self.residuals) >= self.sample_limit:
            self._write_result()
            self._done = True
            rclpy.shutdown()

    def _write_result(self):
        values = np.asarray(self.residuals)
        median = np.median(values, axis=0)
        mad = np.median(np.abs(values - median), axis=0)
        result = {
            "metric": "observable camera-to-base translation residual",
            "reference": "Gazebo ModelStates truth paired with RGB-D target pose",
            "object_name": self.object_name,
            "samples": int(len(values)),
            "recommended_pose_translation_correction_m": median.tolist(),
            "median_absolute_deviation_m": mad.tolist(),
            "note": "This estimates translation only. Full 6D hand-eye rotation requires multiple robot poses and an oriented calibration target.",
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        }
        path = self.output_dir / ("camera_base_residual_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info("calibration written: %s; correction=%s" % (path, median.tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = CameraBaseResidualCalibrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
