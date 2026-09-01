#!/usr/bin/env python3
"""Measure simulated TCP repeatability from actual joint-state FK."""
from __future__ import annotations

import csv
import math
import statistics
import time
from pathlib import Path
from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor

from lab_cobot_manipulation.pick_place_node import (
    DEFAULT_MOVE_TIMEOUT_SEC,
    GRIPPER_TCP_LINK,
    PickPlace,
)


DEFAULT_RESULTS_DIR = "results/manipulation_validation/repeatability"
DEFAULT_TARGET_POSITION = [0.62, 0.0, 0.72]
DEFAULT_TARGET_QUAT = [1.0, 0.0, 0.0, 0.0]
INITIAL_CONFIGS = (
    [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
    [0.18, -1.42, 1.42, -1.58, -1.45, 0.10],
    [-0.18, -1.68, 1.68, -1.45, -1.72, -0.10],
)


class ManipulationRepeatabilityExperiment:
    """Runs the experiment with a PickPlace node and writes raw/stat CSV."""

    def __init__(self, pick_place: PickPlace):
        self.pp = pick_place
        self.node = pick_place
        self.node.declare_parameter("repeatability_trials", 20)
        self.node.declare_parameter("repeatability_output_dir", DEFAULT_RESULTS_DIR)
        self.node.declare_parameter("repeatability_target_position", DEFAULT_TARGET_POSITION)
        self.node.declare_parameter("repeatability_target_quat", DEFAULT_TARGET_QUAT)
        self.trials = int(self.node.get_parameter("repeatability_trials").value)
        self.output_dir = Path(
            str(self.node.get_parameter("repeatability_output_dir").value)
        ).expanduser()
        self.target_position = [
            float(value)
            for value in self.node.get_parameter("repeatability_target_position").value
        ]
        self.target_quat = [
            float(value)
            for value in self.node.get_parameter("repeatability_target_quat").value
        ]

    def run(self) -> Path:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        rows = []
        for trial_id in range(1, self.trials + 1):
            config = INITIAL_CONFIGS[(trial_id - 1) % len(INITIAL_CONFIGS)]
            self.pp._move_to_configuration(config, local_speed=True)  # noqa: SLF001
            started = time.monotonic()
            ok = self.pp._move(  # noqa: SLF001
                self.target_position,
                quat=self.target_quat,
                frame_id="base_link",
                target_link=GRIPPER_TCP_LINK,
                tolerance_position=0.001,
                tolerance_orientation=0.05,
                timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
                cartesian=False,
                stabilize_wrist=True,
                local_speed=False,
                fallback_to_ompl=True,
            )
            time.sleep(0.25)
            pose = self._actual_tcp_pose()
            duration = time.monotonic() - started
            rows.append(
                {
                    "trial_id": trial_id,
                    "command_x": self.target_position[0],
                    "command_y": self.target_position[1],
                    "command_z": self.target_position[2],
                    "command_qx": self.target_quat[0],
                    "command_qy": self.target_quat[1],
                    "command_qz": self.target_quat[2],
                    "command_qw": self.target_quat[3],
                    "actual_x": pose[0],
                    "actual_y": pose[1],
                    "actual_z": pose[2],
                    "actual_qx": pose[3],
                    "actual_qy": pose[4],
                    "actual_qz": pose[5],
                    "actual_qw": pose[6],
                    "move_success": int(ok),
                    "duration": duration,
                    "measurement_source": "MoveIt FK from current joint_state",
                }
            )
        self._add_error_columns(rows)
        csv_path = self.output_dir / "repeatability.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        self._write_summary(rows)
        self.node.get_logger().info(f"wrote {csv_path}")
        return csv_path

    def _actual_tcp_pose(self) -> tuple[float, float, float, float, float, float, float]:
        pose_stamped = self.pp.moveit2.compute_fk(fk_link_names=[GRIPPER_TCP_LINK])
        if isinstance(pose_stamped, list):
            pose_stamped = pose_stamped[0]
        pose = pose_stamped.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )

    def _add_error_columns(self, rows: list[dict]) -> None:
        xs = [float(row["actual_x"]) for row in rows if int(row["move_success"])]
        ys = [float(row["actual_y"]) for row in rows if int(row["move_success"])]
        zs = [float(row["actual_z"]) for row in rows if int(row["move_success"])]
        mean = (
            statistics.fmean(xs) if xs else math.nan,
            statistics.fmean(ys) if ys else math.nan,
            statistics.fmean(zs) if zs else math.nan,
        )
        for row in rows:
            dx = float(row["actual_x"]) - mean[0]
            dy = float(row["actual_y"]) - mean[1]
            dz = float(row["actual_z"]) - mean[2]
            error = math.sqrt(dx * dx + dy * dy + dz * dz)
            row["dx_mm"] = dx * 1000.0
            row["dy_mm"] = dy * 1000.0
            row["dz_mm"] = dz * 1000.0
            row["repeatability_error_mm"] = error * 1000.0

    def _write_summary(self, rows: list[dict]) -> None:
        ok_rows = [row for row in rows if int(row["move_success"])]
        errors = [float(row["repeatability_error_mm"]) for row in ok_rows]
        if errors:
            rms = math.sqrt(statistics.fmean(value * value for value in errors))
            summary = [
                f"trials: {len(rows)}",
                f"successful_moves: {len(ok_rows)}",
                "measurement_source: MoveIt FK from current joint_state",
                f"mean_error_mm: {statistics.fmean(errors):.6f}",
                f"std_error_mm: {(statistics.pstdev(errors) if len(errors) > 1 else 0.0):.6f}",
                f"rms_error_mm: {rms:.6f}",
                f"max_error_mm: {max(errors):.6f}",
            ]
        else:
            summary = ["trials: %d" % len(rows), "successful_moves: 0"]
        (self.output_dir / "repeatability_summary.txt").write_text(
            "\n".join(summary) + "\n",
            encoding="utf-8",
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    pp = PickPlace(node_name="manipulation_repeatability_node")
    executor = MultiThreadedExecutor()
    executor.add_node(pp)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        ManipulationRepeatabilityExperiment(pp).run()
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        pp.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
