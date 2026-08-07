#!/usr/bin/env python3
"""Score YOLO 3D localizations against Gazebo map truth during a condition."""
import json
import math
import time
from collections import defaultdict
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

from image_pkg.benchmark_report import DEFAULT_CONDITIONS, build_aggregate_report


WORKSTATION_LABELS = {
    "station_a": {
        "aruco_sample", "material_spare_igbt", "material_grease_can"},
    "tooling_zone": {"tooling_fixture_box", "tooling_hand_tools"},
    "aging_zone": {"aging_rack"},
    "station_b": {"board_test_fixture"},
    "inspection_zone": {"high_voltage_probe_kit"},
}


class LightingBenchmark(Node):
    """Record position-error success rates; no fixed frame-count stop is used."""

    def __init__(self):
        super().__init__("lighting_benchmark")
        defaults = {
            "condition": "normal_visible",
            "image_topic": "/bench_camera/image_raw",
            "detection_topic": "/yolo/detections",
            "evaluation_topic": "/perception/yolo/evaluation",
            "target_labels": [
                "material_spare_igbt", "aruco_sample", "material_grease_can",
                "aging_rack", "board_test_fixture", "tooling_fixture_box",
                "tooling_hand_tools", "high_voltage_probe_kit",
            ],
            "position_error_threshold_m": 0.15,
            "output_dir": "image_pkg/lighting_benchmark_results",
            "failure_images_per_label": 3,
            "expected_conditions": list(DEFAULT_CONDITIONS),
            "minimum_evaluations_per_condition": 100,
            "station_status_topic": "/image_pkg/cruise/status",
            "odom_topic": "/odom",
            "settle_seconds": 1.0,
            "max_station_speed_mps": 0.02,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.condition = str(self.get_parameter("condition").value)
        self.labels = [str(label) for label in self.get_parameter("target_labels").value]
        self.threshold = float(self.get_parameter("position_error_threshold_m").value)
        if self.threshold <= 0.0:
            raise ValueError("position_error_threshold_m must be positive")
        self.output_dir = Path(str(self.get_parameter("output_dir").value)) / self.condition
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.failure_limit = int(self.get_parameter("failure_images_per_label").value)
        self.expected_conditions = [
            str(value) for value in self.get_parameter("expected_conditions").value]
        self.minimum_evaluations = int(
            self.get_parameter("minimum_evaluations_per_condition").value)
        if self.minimum_evaluations <= 0:
            raise ValueError("minimum_evaluations_per_condition must be positive")
        self.bridge, self.latest_image = CvBridge(), None
        self.total = defaultdict(int)
        self.success = defaultdict(int)
        self.errors = defaultdict(list)
        self.axis_errors = defaultdict(list)
        self.failure_images = defaultdict(int)
        self.coarse_frames = 0
        self.coarse_detected_frames = 0
        self.active_station = None
        self.active_label = None
        self.station_arrival_time = None
        self.station_arrival_measurement_time = None
        self.speed_mps = float("inf")
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._image_cb, 1)
        self.create_subscription(
            String, str(self.get_parameter("detection_topic").value),
            self._coarse_detection_cb, 50)
        self.create_subscription(
            String, str(self.get_parameter("evaluation_topic").value), self._evaluation_cb, 50)
        self.create_subscription(
            String, str(self.get_parameter("station_status_topic").value), self._station_cb, 20)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 20)
        self.get_logger().info(
            f"Benchmark {self.condition}: success if 3D error <= {self.threshold:.3f} m; "
            "stop after the cruise returns home (Ctrl-C)."
        )

    def _image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # Image snapshots are diagnostic only.
            self.get_logger().warning(f"Ignoring image conversion error: {exc}")

    def _coarse_detection_cb(self, msg):
        """Record YOLO proposal success separately from 3-D localization."""
        try:
            payload = json.loads(msg.data)
            detections = payload.get("detections", [])
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if (not self._station_is_stable()
                or not self._measurement_belongs_to_active_station(
                    payload.get("timestamp"))):
            return
        expected = ({self.active_label} if self.active_label
                    else WORKSTATION_LABELS[self.active_station])
        self.coarse_frames += 1
        self.coarse_detected_frames += int(any(
            str(item.get("label", "")) in expected for item in detections
            if isinstance(item, dict)))

    def _station_cb(self, msg):
        status = str(msg.data)
        if status.startswith("ARRIVED:"):
            station = status.split(":", 2)[1]
            self.active_station = station if station in WORKSTATION_LABELS else None
            self.station_arrival_time = time.monotonic() if self.active_station else None
            self.station_arrival_measurement_time = (
                self._clock_seconds() if self.active_station else None)
            self.active_label = None
        elif status.startswith("OBSERVING:"):
            _, station, label = status.split(":", 2)
            if station == self.active_station and label in WORKSTATION_LABELS[station]:
                self.active_label = label
        elif status.startswith(("NAVIGATING:", "NAV2_FAILED_FALLBACK_ODOM:", "FAILED:", "DONE:")):
            self.active_station = None
            self.station_arrival_time = None
            self.station_arrival_measurement_time = None
            self.active_label = None

    def _odom_cb(self, msg):
        twist = msg.twist.twist.linear
        self.speed_mps = math.sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z)

    def _station_is_stable(self):
        return (
            self.active_station in WORKSTATION_LABELS
            and self.station_arrival_time is not None
            and time.monotonic() - self.station_arrival_time >= float(
                self.get_parameter("settle_seconds").value)
            and self.speed_mps <= float(self.get_parameter("max_station_speed_mps").value)
        )

    def _clock_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _measurement_belongs_to_active_station(self, timestamp):
        """Reject results whose source image predates this stopped station."""
        if self.station_arrival_measurement_time is None:
            return False
        try:
            measurement_time = float(timestamp)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(measurement_time):
            return False
        return measurement_time >= (
            self.station_arrival_measurement_time
            + float(self.get_parameter("settle_seconds").value)
        )

    def _evaluation_cb(self, msg):
        try:
            item = json.loads(msg.data)
            label = str(item["label"])
            error = float(item["total_position_error"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring invalid evaluation event: {exc}")
            return
        if (label not in self.labels or not math.isfinite(error)
                or not self._station_is_stable()
                or not self._measurement_belongs_to_active_station(
                    item.get("measurement_stamp_sec"))
                or (self.active_label is not None and label != self.active_label)
                or label not in WORKSTATION_LABELS[self.active_station]):
            return
        self.total[label] += 1
        self.errors[label].append(error)
        axis_error = _valid_axis_error(item.get("error_xyz"))
        if axis_error is not None:
            self.axis_errors[label].append(axis_error)
        passed = error <= self.threshold
        self.success[label] += int(passed)
        if not passed:
            self._save_failure(label, error)
        if self.total[label] % 10 == 0:
            rate = self.success[label] / self.total[label]
            self.get_logger().info(
                f"{self.condition} {label}: {self.total[label]} estimates, "
                f"{rate:.1%} within {self.threshold:.3f} m"
            )

    def _save_failure(self, label, error):
        if self.latest_image is None or self.failure_images[label] >= self.failure_limit:
            return
        index = self.failure_images[label] + 1
        path = self.output_dir / f"{label}_error_{error:.3f}m_{index}.png"
        if cv2.imwrite(str(path), self.latest_image):
            self.failure_images[label] += 1

    def finalize(self):
        """Write a condition summary when the cruise/test operator stops us."""
        per_label = {}
        all_errors, all_axis_errors, all_success = [], [], 0
        for label in self.labels:
            values = self.errors[label]
            axis_values = self.axis_errors[label]
            count = self.total[label]
            all_errors.extend(values)
            all_axis_errors.extend(axis_values)
            all_success += self.success[label]
            per_label[label] = {
                "evaluations": count,
                "successes": self.success[label],
                "recognition_rate": self.success[label] / count if count else None,
                "mean_position_error_m": sum(values) / count if count else None,
                "max_position_error_m": max(values) if values else None,
                **_axis_error_summary(axis_values),
            }
        total = len(all_errors)
        result = {
            "condition": self.condition,
            "metric": "3D position error against Gazebo map truth",
            "sampling_gate": "matching station labels with RGB-D source timestamps after arrival, settle time, and zero-speed check",
            "coarse_metric": "YOLO proposal frames with one or more target boxes",
            "coarse_frames": self.coarse_frames,
            "coarse_detected_frames": self.coarse_detected_frames,
            "coarse_detection_rate": (
                self.coarse_detected_frames / self.coarse_frames
                if self.coarse_frames else None
            ),
            "position_error_threshold_m": self.threshold,
            "evaluations": total,
            "successes": all_success,
            "recognition_rate": all_success / total if total else None,
            "mean_position_error_m": sum(all_errors) / total if total else None,
            **_axis_error_summary(all_axis_errors),
            "per_label": per_label,
            "failure_images": dict(self.failure_images),
            "schema_version": 3,
            "minimum_evaluations_per_condition": self.minimum_evaluations,
            "complete": total >= self.minimum_evaluations,
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self._write_aggregate_report()
        self.get_logger().info(f"Saved position-error benchmark to {self.output_dir}")

    def _write_aggregate_report(self):
        root = self.output_dir.parent
        results = [json.loads(path.read_text(encoding="utf-8"))
                   for path in sorted(root.glob("*/summary.json"))]
        (root / "lighting_benchmark_report.md").write_text(
            build_aggregate_report(
                results, self.expected_conditions, self.minimum_evaluations),
            encoding="utf-8")


def _valid_axis_error(value):
    """Return a finite (dx, dy, dz) tuple, or ``None`` for legacy events."""
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        result = tuple(float(component) for component in value)
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(component) for component in result) else None


def _axis_error_summary(values):
    """Summarize signed and absolute XYZ residuals without hiding bias."""
    if not values:
        return {
            "axis_evaluations": 0,
            "mean_error_xyz_m": None,
            "mean_absolute_error_xyz_m": None,
            "max_absolute_error_xyz_m": None,
        }
    count = len(values)
    return {
        "axis_evaluations": count,
        "mean_error_xyz_m": [sum(row[index] for row in values) / count
                             for index in range(3)],
        "mean_absolute_error_xyz_m": [
            sum(abs(row[index]) for row in values) / count
            for index in range(3)],
        "max_absolute_error_xyz_m": [
            max(abs(row[index]) for row in values)
            for index in range(3)],
    }


def main(args=None):
    rclpy.init(args=args)
    node = LightingBenchmark()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
