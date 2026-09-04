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
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from image_pkg.benchmark_report import DEFAULT_CONDITIONS, build_aggregate_report


WORKSTATION_LABELS = {
    "station_a": {
        "aruco_sample", "material_cube_red", "material_cube_green",
        "material_cube_blue", "material_cube_yellow"},
    "tooling_zone": {"tooling_fixture_box", "tooling_hand_tools",
                     "board_test_fixture", "high_voltage_probe_kit",
                     "material_spare_igbt"},
    "aging_zone": {"aging_rack", "pcb_board"},
    "station_b": {"test_tube_rack", "test_tube", "beaker",
                  "erlenmeyer_flask", "graduated_cylinder"},
}
MARKED_LABELS = {
    "aruco_sample", "material_cube_red", "material_cube_green",
    "material_cube_blue", "material_cube_yellow",
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
            "pipeline_metrics_topic": "/perception/yolo/pipeline_metrics",
            # The six-condition millimetre benchmark is scoped to the five
            # tagged grasp objects. Untagged YOLO+RGB-D centroids are a
            # different metric and must not be mixed into this acceptance.
            "target_labels": sorted(MARKED_LABELS),
            "position_error_threshold_m": 0.001,
            # Runtime summaries are intermediate evidence. The maintained
            # project result is image_pkg/测试报告.md, not another result tree.
            "output_dir": "/tmp/image_pkg_lighting_benchmark",
            "failure_images_per_label": 3,
            "expected_conditions": list(DEFAULT_CONDITIONS),
            # One stopped, three-view fused estimate for each tagged object.
            "minimum_evaluations_per_condition": len(MARKED_LABELS),
            "station_status_topic": "/image_pkg/cruise/status",
            "odom_topic": "/odom",
            "settle_seconds": 1.0,
            "max_station_speed_mps": 0.02,
            "joint_state_topic": "/joint_states",
            "max_arm_joint_speed_rps": 0.20,
            "joint_state_timeout_seconds": 0.5,
            # station_cruise publishes OBSERVING_FINE only after base stop,
            # MoveIt completion and its own continuous joint-settle check.
            # Trust that authoritative gate by default; the duplicated local
            # QoS/timing gate can still be enabled for standalone experiments.
            "trust_cruise_stability_gate": True,
            "aruco_source_only": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.condition = str(self.get_parameter("condition").value)
        self.finished = False
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
        self.item_total = defaultdict(int)
        self.item_success = defaultdict(int)
        self.item_errors = defaultdict(list)
        self.item_axis_errors = defaultdict(list)
        self.failure_images = defaultdict(int)
        self.pipeline_counts = defaultdict(lambda: defaultdict(int))
        # ``PIPELINE_COUNTS`` is emitted by station_cruise after a view has
        # passed its own stopped/centred-target gate.  Prefer those explicit
        # counters over asynchronous per-stage DDS samples when available.
        self._explicit_pipeline_labels = set()
        # A diagnostic pose node may be deliberately run beside the primary
        # node during an investigation.  Count a source image only once even
        # if DDS delivers the same stage/event from both processes.
        self._seen_pipeline_events = set()
        self._seen_accepted_events = set()
        self.coarse_frames = 0
        self.coarse_detected_frames = 0
        self.active_station = None
        self.active_label = None
        self.active_model = None
        self.fine_observation_active = False
        self.station_arrival_time = None
        self.station_arrival_measurement_time = None
        self.speed_mps = float("inf")
        self.max_arm_joint_speed_rps = float("inf")
        self.last_joint_state_time = None
        self._last_arm_positions = {}
        self._last_arm_sample_time = None
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._image_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.get_parameter("detection_topic").value),
            self._coarse_detection_cb, 50)
        self.create_subscription(
            String, str(self.get_parameter("evaluation_topic").value), self._evaluation_cb, 50)
        self.create_subscription(
            String, str(self.get_parameter("pipeline_metrics_topic").value),
            self._pipeline_metrics_cb, 100)
        self.create_subscription(
            String, str(self.get_parameter("station_status_topic").value), self._station_cb, 20)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            JointState, str(self.get_parameter("joint_state_topic").value),
            self._joint_state_cb, qos_profile_sensor_data)
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
        if status.startswith("PIPELINE_COUNTS:"):
            fields = status.split(":")
            if len(fields) < 4:
                return
            station, label = fields[1], fields[2]
            if label not in self.labels or station not in WORKSTATION_LABELS:
                return
            if label not in self._explicit_pipeline_labels:
                self.pipeline_counts[label].clear()
                self._explicit_pipeline_labels.add(label)
            for field in fields[4:]:
                key, separator, raw_value = field.partition("=")
                if separator and key in {"yolo_box", "valid_depth", "world_transform"}:
                    try:
                        self.pipeline_counts[label][key] += int(raw_value)
                    except ValueError:
                        continue
        elif status.startswith("ARRIVED:"):
            station = status.split(":", 2)[1]
            self.active_station = station if station in WORKSTATION_LABELS else None
            self.station_arrival_time = time.monotonic() if self.active_station else None
            self.station_arrival_measurement_time = (
                self._clock_seconds() if self.active_station else None)
            self.active_label = None
            self.active_model = None
            self.fine_observation_active = False
        elif status.startswith("OBSERVING_FINE:"):
            fields = status.split(":")
            if len(fields) < 3:
                return
            station, label = fields[1], fields[2]
            if station == self.active_station and label in WORKSTATION_LABELS[station]:
                self.active_label = label
                self.active_model = fields[3] if len(fields) >= 4 else None
                self.fine_observation_active = True
                # Reject any delayed result captured during coarse aiming or
                # base motion before this exact per-object fine window.
                self.station_arrival_measurement_time = self._clock_seconds()
                self.station_arrival_time = time.monotonic()
        elif status.startswith(("COARSE_", "OBSERVATION_BASE:", "CAMERA_ABOVE:")):
            self.active_label = None
            self.active_model = None
            self.fine_observation_active = False
        elif status.startswith(("NAVIGATING:", "NAV2_FAILED_FALLBACK_ODOM:", "FAILED:", "DONE:")):
            self.active_station = None
            self.station_arrival_time = None
            self.station_arrival_measurement_time = None
            self.active_label = None
            self.active_model = None
            self.fine_observation_active = False
            if status.startswith("DONE:"):
                self.finished = True

    def _odom_cb(self, msg):
        twist = msg.twist.twist.linear
        self.speed_mps = math.sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z)

    def _joint_state_cb(self, msg):
        arm_prefix = "ur_"
        now = time.monotonic()
        dt = (None if self._last_arm_sample_time is None
              else now - self._last_arm_sample_time)
        velocities = []
        for name, position in zip(msg.name, msg.position):
            if not (name.startswith(arm_prefix) and name.endswith("_joint")):
                continue
            value = float(position)
            if dt is not None and dt > 1e-4 and name in self._last_arm_positions:
                delta = math.atan2(
                    math.sin(value - self._last_arm_positions[name]),
                    math.cos(value - self._last_arm_positions[name]))
                velocities.append(abs(delta / dt))
            self._last_arm_positions[name] = value
        self.max_arm_joint_speed_rps = max(velocities) if velocities else float("inf")
        self._last_arm_sample_time = now
        self.last_joint_state_time = now

    def _station_is_stable(self):
        trusted = bool(
            self.get_parameter("trust_cruise_stability_gate").value)
        settle_seconds = 0.0 if trusted else float(
            self.get_parameter("settle_seconds").value)
        core_ready = (
            self.active_station in WORKSTATION_LABELS
            and self.fine_observation_active
            and self.station_arrival_time is not None
            and time.monotonic() - self.station_arrival_time >= settle_seconds
        )
        if not core_ready:
            return False
        if trusted:
            return True
        return (
            self.speed_mps <= float(self.get_parameter("max_station_speed_mps").value)
            and self.last_joint_state_time is not None
            and time.monotonic() - self.last_joint_state_time <= float(
                self.get_parameter("joint_state_timeout_seconds").value)
            and self.max_arm_joint_speed_rps <= float(
                self.get_parameter("max_arm_joint_speed_rps").value)
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
            + (0.0 if bool(self.get_parameter(
                "trust_cruise_stability_gate").value) else float(
                    self.get_parameter("settle_seconds").value))
        )

    def _evaluation_cb(self, msg):
        try:
            item = json.loads(msg.data)
            label = str(item["label"])
            error = float(item["total_position_error"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring invalid evaluation event: {exc}")
            return
        source = str(item.get("estimate_source", ""))
        # A multiview event is produced only after station_cruise has acquired
        # the requested independent stopped views.  ROS topic delivery can
        # reorder this terminal event with the next station-status message;
        # retain it rather than falsely reporting zero measurements.
        cruise_fused = source.startswith("multiview_fused:")
        active_window = (
            self._station_is_stable()
            and self._measurement_belongs_to_active_station(
                item.get("measurement_stamp_sec"))
            and (self.active_label is None or label == self.active_label)
            and self.active_station in WORKSTATION_LABELS
            and label in WORKSTATION_LABELS[self.active_station]
        )
        if label not in self.labels or not math.isfinite(error) or not (cruise_fused or active_window):
            return
        if (label in MARKED_LABELS
                and bool(self.get_parameter("aruco_source_only").value)
                and not _is_aruco_precision_source(source)):
            return
        if (not cruise_fused and self.active_model
                and str(item.get("matched_truth_model", "")).strip()
                != self.active_model):
            return
        try:
            measurement_stamp = float(item.get("measurement_stamp_sec"))
        except (TypeError, ValueError):
            return
        event_key = (label, measurement_stamp, str(item.get("estimate_source", "")))
        if event_key in self._seen_accepted_events:
            return
        self._seen_accepted_events.add(event_key)
        self.total[label] += 1
        self.errors[label].append(error)
        axis_error = _valid_axis_error(item.get("error_xyz"))
        if axis_error is not None:
            self.axis_errors[label].append(axis_error)
        passed = error <= self.threshold
        self.success[label] += int(passed)
        model_name = str(item.get("matched_truth_model", "")).strip()
        if model_name:
            self.item_total[model_name] += 1
            self.item_success[model_name] += int(passed)
            self.item_errors[model_name].append(error)
            if axis_error is not None:
                self.item_axis_errors[model_name].append(axis_error)
        if not passed:
            self._save_failure(label, error)
        if self.total[label] % 10 == 0:
            rate = self.success[label] / self.total[label]
            self.get_logger().info(
                f"{self.condition} {label}: {self.total[label]} estimates, "
                f"{rate:.1%} within {self.threshold:.3f} m"
            )

    def _pipeline_metrics_cb(self, msg):
        """Count each localization stage only in its stopped target window."""
        try:
            item = json.loads(msg.data)
            label = str(item["label"])
            stage = str(item["stage"])
            timestamp = item["measurement_stamp_sec"]
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if stage not in {"yolo_box", "valid_depth", "world_transform"}:
            return
        if label in self._explicit_pipeline_labels:
            return
        if (label not in self.labels
                or not self._station_is_stable()
                or not self._measurement_belongs_to_active_station(timestamp)
                or (self.active_label is not None and label != self.active_label)
                or label not in WORKSTATION_LABELS[self.active_station]):
            return
        try:
            event_key = (label, stage, float(timestamp))
        except (TypeError, ValueError):
            return
        if event_key in self._seen_pipeline_events:
            return
        self._seen_pipeline_events.add(event_key)
        self.pipeline_counts[label][stage] += 1

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
        per_item = {}
        all_errors, all_axis_errors, all_success = [], [], 0
        for label in self.labels:
            values = self.errors[label]
            axis_values = self.axis_errors[label]
            count = self.total[label]
            all_errors.extend(values)
            all_axis_errors.extend(axis_values)
            all_success += self.success[label]
            per_label[label] = {
                "pipeline_counts": {
                    "yolo_boxes": self.pipeline_counts[label]["yolo_box"],
                    "valid_depth": self.pipeline_counts[label]["valid_depth"],
                    "world_transforms": self.pipeline_counts[label]["world_transform"],
                    "station_accepted": count,
                },
                "evaluations": count,
                "successes": self.success[label],
                "recognition_rate": self.success[label] / count if count else None,
                "mean_position_error_m": sum(values) / count if count else None,
                "max_position_error_m": max(values) if values else None,
                **_axis_error_summary(axis_values),
            }
        for model_name in sorted(self.item_total):
            values = self.item_errors[model_name]
            count = self.item_total[model_name]
            per_item[model_name] = {
                "evaluations": count,
                "successes": self.item_success[model_name],
                "recognition_rate": self.item_success[model_name] / count if count else None,
                "mean_position_error_m": sum(values) / count if count else None,
                "max_position_error_m": max(values) if values else None,
                **_axis_error_summary(self.item_axis_errors[model_name]),
            }
        total = len(all_errors)
        result = {
            "condition": self.condition,
            "metric": "3D position error against Gazebo map truth",
            "sampling_gate": "matching station labels with RGB-D source timestamps after arrival, settle time, stopped base, and settled arm-joint checks",
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
            "per_gazebo_model": per_item,
            "failure_images": dict(self.failure_images),
            "pipeline_metric": (
                "Counts are station-scoped: YOLO boxes -> boxes with valid RGB-D "
                "foreground -> finite world-frame transforms -> estimates accepted "
                "after arrival/settle/zero-speed gate"),
            "schema_version": 5,
            "minimum_evaluations_per_condition": self.minimum_evaluations,
            "required_labels": sorted(self.labels),
            "evaluated_labels": sorted(
                label for label in self.labels if self.total[label] > 0),
            "complete": (
                total >= self.minimum_evaluations
                and all(self.total[label] > 0 for label in self.labels)),
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


def _is_aruco_precision_source(source):
    """Accept direct or fused results only when every source is ArUco based."""
    value = str(source).strip()
    if value.startswith("aruco_"):
        return True
    prefix = "multiview_fused:"
    if not value.startswith(prefix):
        return False
    components = [part.strip() for part in value[len(prefix):].split("+")]
    return bool(components) and all(part.startswith("aruco_") for part in components)


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
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
