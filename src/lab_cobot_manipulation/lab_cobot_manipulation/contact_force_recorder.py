#!/usr/bin/env python3
"""Record the two tactile-probe contact forces and render a grasp curve."""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.utilities import remove_ros_args
from gazebo_msgs.msg import ContactsState
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from lab_cobot_manipulation.gripper_driver import (
    DEFAULT_TARGET_OBJECT,
    LEFT_FINGER_CONTACTS_TOPIC,
    RIGHT_FINGER_CONTACTS_TOPIC,
)


CONTACT_FORCE_TOPIC = "/gripper/contact/force"


def finite_nonnegative(value) -> float:
    """Return a finite non-negative force value for plotting/statistics."""
    value = float(value)
    if not math.isfinite(value):
        return 0.0
    return max(0.0, value)


def percentile(values, fraction: float) -> float:
    """Return a simple percentile from a non-empty numeric sequence."""
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    index = min(
        max(int((len(ordered) - 1) * float(fraction)), 0),
        len(ordered) - 1,
    )
    return ordered[index]


def plot_y_limit(total_force_values) -> float:
    """Choose a readable y-axis cap without letting solver spikes dominate."""
    positive = [
        finite_nonnegative(value)
        for value in total_force_values
        if finite_nonnegative(value) > 0.0
    ]
    if not positive:
        return 1.0
    typical_peak = percentile(positive, 0.90)
    return max(5.0, typical_peak * 1.35)


def contact_window(elapsed, total_force_values, padding_sec: float = 5.0):
    """Return x-limits around the non-zero contact region."""
    active_times = [
        float(time_value)
        for time_value, force_value in zip(elapsed, total_force_values)
        if finite_nonnegative(force_value) > 0.0
    ]
    if not active_times:
        return None
    start = max(min(active_times) - padding_sec, min(elapsed))
    end = min(max(active_times) + padding_sec, max(elapsed))
    if end <= start:
        return None
    return start, end


def binned_peak_series(elapsed, *series, max_points: int = 900):
    """Downsample dense force samples by keeping each time-bin peak."""
    if not elapsed or not series:
        return [], *([] for _ in series)
    start = min(float(value) for value in elapsed)
    end = max(float(value) for value in elapsed)
    if end <= start or len(elapsed) <= max_points:
        return list(elapsed), *[list(values) for values in series]

    bin_width = (end - start) / float(max(max_points, 1))
    centers = []
    peaks = [[] for _ in series]
    current_bin = None
    current_peaks = [0.0 for _ in series]

    def flush(bin_index):
        centers.append(start + (float(bin_index) + 0.5) * bin_width)
        for peak_values, value in zip(peaks, current_peaks):
            peak_values.append(value)

    for index, time_value in enumerate(elapsed):
        bin_index = min(int((float(time_value) - start) / bin_width), max_points - 1)
        if current_bin is None:
            current_bin = bin_index
        if bin_index != current_bin:
            flush(current_bin)
            current_bin = bin_index
            current_peaks = [0.0 for _ in series]
        for series_index, values in enumerate(series):
            current_peaks[series_index] = max(
                current_peaks[series_index],
                finite_nonnegative(values[index]),
            )
    if current_bin is not None:
        flush(current_bin)
    return centers, *peaks


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
            self._left_force = finite_nonnegative(msg.data[0])
            self._right_force = finite_nonnegative(msg.data[1])
            self._append_sample()

    def _on_left_contacts(self, msg: ContactsState) -> None:
        self._left_force = finite_nonnegative(
            force_for_target(msg, self.target_object)
        )
        self._append_sample()

    def _on_right_contacts(self, msg: ContactsState) -> None:
        self._right_force = finite_nonnegative(
            force_for_target(msg, self.target_object)
        )
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
        y_limit = plot_y_limit(total)
        clipped_left = [min(finite_nonnegative(value), y_limit) for value in left]
        clipped_right = [min(finite_nonnegative(value), y_limit) for value in right]
        clipped_total = [min(finite_nonnegative(value), y_limit) for value in total]
        clipped_count = sum(
            1 for value in total if finite_nonnegative(value) > y_limit
        )
        window = contact_window(elapsed, total)

        overview_elapsed, overview_left, overview_right, overview_total = (
            binned_peak_series(
                elapsed,
                clipped_left,
                clipped_right,
                clipped_total,
                max_points=900,
            )
        )
        detail_elapsed, detail_left, detail_right, detail_total = (
            binned_peak_series(
                elapsed,
                clipped_left,
                clipped_right,
                clipped_total,
                max_points=1600,
            )
        )

        figure, axes = plt.subplots(
            2,
            1,
            figsize=(11.5, 7.2),
            constrained_layout=True,
            sharey=True,
        )

        def draw_axis(axis, title, x_values, left_values, right_values, total_values):
            axis.plot(
                x_values,
                total_values,
                label="sum",
                color="#202020",
                linewidth=1.0,
                alpha=0.55,
                zorder=1,
            )
            axis.plot(
                x_values,
                left_values,
                label="left tactile probe",
                color="#d55e00",
                linewidth=1.4,
                zorder=2,
            )
            axis.plot(
                x_values,
                right_values,
                label="right tactile probe",
                color="#0072b2",
                linewidth=1.4,
                zorder=3,
            )
            axis.set_title(title)
            axis.set_ylabel("Force magnitude (N)")
            axis.set_ylim(bottom=0.0, top=y_limit)
            axis.grid(alpha=0.25)
            axis.legend(loc="upper right")

        draw_axis(
            axes[0],
            f"G4 contact force overview: {self.target_object}",
            overview_elapsed,
            overview_left,
            overview_right,
            overview_total,
        )
        draw_axis(
            axes[1],
            "Contact window detail",
            detail_elapsed,
            detail_left,
            detail_right,
            detail_total,
        )
        axes[0].set_xlabel("Elapsed time (s)")
        axes[1].set_xlabel("Elapsed time (s)")
        if window is not None:
            axes[1].set_xlim(*window)
        if clipped_count:
            axes[0].text(
                0.01,
                0.92,
                f"{clipped_count} solver spike sample(s) clipped above {y_limit:.1f} N",
                transform=axes[0].transAxes,
                fontsize=9,
                bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cccccc"},
            )
        figure.savefig(png_path, dpi=240)
        plt.close(figure)
        return csv_path, png_path


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record Gazebo tactile contact force during a grasp."
    )
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--target-object", default=DEFAULT_TARGET_OBJECT)
    parser.add_argument("--output-dir", default="g4_artifacts")
    parser.add_argument("--stem", default="contact_force_curve")
    clean_argv = remove_ros_args(args=list(sys.argv if argv is None else argv))
    return parser.parse_args(clean_argv[1:])


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
