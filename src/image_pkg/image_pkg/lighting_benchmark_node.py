#!/usr/bin/env python3
"""Score YOLO 3D localizations against Gazebo map truth during a condition."""
import json
import math
from collections import defaultdict
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class LightingBenchmark(Node):
    """Record position-error success rates; no fixed frame-count stop is used."""

    def __init__(self):
        super().__init__("lighting_benchmark")
        defaults = {
            "condition": "normal_visible",
            "image_topic": "/bench_camera/image_raw",
            "evaluation_topic": "/perception/yolo/evaluation",
            "target_labels": [
                "aruco_sample", "igbt_module_plain", "thermal_grease_can",
                "fixture_box_plain", "tooling_hand_tools", "aging_rack",
                "pcb_test_fixture", "safety_probe_kit",
            ],
            "position_error_threshold_m": 0.15,
            "output_dir": "image_pkg/lighting_benchmark_results",
            "failure_images_per_label": 3,
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
        self.bridge, self.latest_image = CvBridge(), None
        self.total = defaultdict(int)
        self.success = defaultdict(int)
        self.errors = defaultdict(list)
        self.failure_images = defaultdict(int)
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._image_cb, 1)
        self.create_subscription(
            String, str(self.get_parameter("evaluation_topic").value), self._evaluation_cb, 50)
        self.get_logger().info(
            f"Benchmark {self.condition}: success if 3D error <= {self.threshold:.3f} m; "
            "stop after the cruise returns home (Ctrl-C)."
        )

    def _image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # Image snapshots are diagnostic only.
            self.get_logger().warning(f"Ignoring image conversion error: {exc}")

    def _evaluation_cb(self, msg):
        try:
            item = json.loads(msg.data)
            label = str(item["label"])
            error = float(item["total_position_error"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring invalid evaluation event: {exc}")
            return
        if label not in self.labels or not math.isfinite(error):
            return
        self.total[label] += 1
        self.errors[label].append(error)
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
        all_errors, all_success = [], 0
        for label in self.labels:
            values = self.errors[label]
            count = self.total[label]
            all_errors.extend(values)
            all_success += self.success[label]
            per_label[label] = {
                "evaluations": count,
                "successes": self.success[label],
                "recognition_rate": self.success[label] / count if count else None,
                "mean_position_error_m": sum(values) / count if count else None,
                "max_position_error_m": max(values) if values else None,
            }
        total = len(all_errors)
        result = {
            "condition": self.condition,
            "metric": "3D position error against Gazebo map truth",
            "position_error_threshold_m": self.threshold,
            "evaluations": total,
            "successes": all_success,
            "recognition_rate": all_success / total if total else None,
            "mean_position_error_m": sum(all_errors) / total if total else None,
            "per_label": per_label,
            "failure_images": dict(self.failure_images),
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self._write_aggregate_report()
        self.get_logger().info(f"Saved position-error benchmark to {self.output_dir}")

    def _write_aggregate_report(self):
        root = self.output_dir.parent
        results = [json.loads(path.read_text(encoding="utf-8"))
                   for path in sorted(root.glob("*/summary.json"))]
        rows = [
            "# 六工况三维定位识别率", "",
            "识别成功定义：检测三维坐标与 Gazebo 建图真值的欧氏误差不超过工况阈值。", "",
            "| 工况 | 有效定位数 | 阈值 (m) | 识别率 | 平均位置误差 (m) |",
            "|---|---:|---:|---:|---:|",
        ]
        for item in results:
            rate = item["recognition_rate"]
            mean = item["mean_position_error_m"]
            rows.append(
                f"| {item['condition']} | {item['evaluations']} | "
                f"{item['position_error_threshold_m']:.3f} | "
                f"{'—' if rate is None else f'{rate:.1%}'} | "
                f"{'—' if mean is None else f'{mean:.3f}'} |"
            )
        rows.extend([
            "", "## 口径", "",
            "- 不按固定 100 帧结束；完整巡航返回 home 后停止采集器以生成结果。",
            "- 没有生成三维坐标的漏检不会产生误差事件，应与失败截图一并人工复核。",
            "- 每类物品的成功率和位置误差见各工况的 `summary.json`。",
        ])
        (root / "lighting_benchmark_report.md").write_text(
            "\n".join(rows) + "\n", encoding="utf-8")


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
