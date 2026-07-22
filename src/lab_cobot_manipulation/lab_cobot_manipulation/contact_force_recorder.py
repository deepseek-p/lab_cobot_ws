#!/usr/bin/env python3
"""Record the two tactile-probe contact forces and render a grasp curve."""
from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ContactsState
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from lab_cobot_manipulation.gripper_driver import (
    DEFAULT_TARGET_OBJECT,
    LEFT_FINGER_CONTACTS_TOPIC,
    RIGHT_FINGER_CONTACTS_TOPIC,
)


CONTACT_FORCE_TOPIC = "/gripper/contact/force"


def vector_length(vector) -> float:
    """Return a geometry_msgs Vector3 magnitude without requiring ROS at test time."""
    return math.sqrt(
        float(vector.x) ** 2 + float(vector.y) ** 2 + float(vector.z) ** 2
    )


def force_for_target(msg, target_object: str) -> float:
    """Sum force magnitudes for contacts involving the requested Gazebo model."""
    prefix = f"{target_object}::"
    total = 0.0
    for state in getattr(msg, "states", []):
        names = (str(state.collision1_name), str(state.collision2_name))
        if not any(name.startswith(prefix) for name in names):
            continue
        total += vector_length(state.total_wrench.force)
    return total


class ContactForceRecorder(Node):
    """Record every plugin force sample, including millisecond-scale contact peaks."""

    def __init__(self, target_object: str = DEFAULT_TARGET_OBJECT):
        super().__init__("contact_force_recorder")
        self.target_object = str(target_object)
        self._started_at = time.monotonic()
        self._left_force = 0.0
        self._right_force = 0.0
        self.samples: list[tuple[float, float, float]] = []
        self.create_subscription(
            Float64MultiArray,
            CONTACT_FORCE_TOPIC,
            self._on_contact_force,
            10,
        )
        self.create_subscription(
            ContactsState,
            LEFT_FINGER_CONTACTS_TOPIC,
            self._on_left_contacts,
            10,
        )
        self.create_subscription(
            ContactsState,
            RIGHT_FINGER_CONTACTS_TOPIC,
            self._on_right_contacts,
            10,
        )

    def _on_contact_force(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 2:
            self._left_force = max(0.0, float(msg.data[0]))
            self._right_force = max(0.0, float(msg.data[1]))
            self._append_sample()

    def _on_left_contacts(self, msg: ContactsState) -> None:
        self._left_force = max(0.0, force_for_target(msg, self.target_object))
        self._append_sample()

    def _on_right_contacts(self, msg: ContactsState) -> None:
        self._right_force = max(0.0, force_for_target(msg, self.target_object))
        self._append_sample()

    def _append_sample(self) -> None:
        # 逐点保存，防止短接触峰值被定时采样遗漏。
        self.samples.append(
            (
                time.monotonic() - self._started_at,
                self._left_force,
                self._right_force,
            )
        )

    def write_artifacts(self, output_dir: Path, stem: str) -> tuple[Path, Path]:
        """Write raw CSV and a report-ready PNG. Returns their paths."""
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / f"{stem}.csv"
        png_path = output_dir / f"{stem}.png"
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["elapsed_sec", "left_force_n", "right_force_n", "sum_force_n"])
            for elapsed, left, right in self.samples:
                writer.writerow([elapsed, left, right, left + right])

        # 延迟导入使纯数据处理/单元测试不依赖 GUI 后端。
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        elapsed = [sample[0] for sample in self.samples]
        left = [sample[1] for sample in self.samples]
        right = [sample[2] for sample in self.samples]
        total = [left_value + right_value for left_value, right_value in zip(left, right)]
        figure, axis = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
        axis.plot(elapsed, left, label="left tactile probe", color="#e67e22")
        axis.plot(elapsed, right, label="right tactile probe", color="#1677ff")
        axis.plot(elapsed, total, label="sum", color="#222222", linewidth=1.4)
        axis.set_title(f"Grasp contact force: {self.target_object}")
        axis.set_xlabel("Elapsed time (s)")
        axis.set_ylabel("Contact force magnitude (N)")
        axis.grid(alpha=0.3)
        axis.legend()
        figure.savefig(png_path, dpi=160)
        plt.close(figure)
        return csv_path, png_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record Gazebo tactile contact force during a grasp."
    )
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--target-object", default=DEFAULT_TARGET_OBJECT)
    parser.add_argument("--output-dir", default="g4_artifacts")
    parser.add_argument("--stem", default="contact_force_curve")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ContactForceRecorder(target_object=args.target_object)
    deadline = time.monotonic() + max(float(args.duration), 0.0)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        csv_path, png_path = node.write_artifacts(Path(args.output_dir), args.stem)
        node.get_logger().info(f"contact-force CSV: {csv_path}")
        node.get_logger().info(f"contact-force plot: {png_path}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
