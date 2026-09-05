#!/usr/bin/env python3
"""Measure simulated TCP repeatability from actual joint-state FK."""
from __future__ import annotations

import csv
import copy
from collections import deque
import math
import statistics
import time
from types import MethodType
from pathlib import Path
from threading import Thread

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformException, TransformListener

from lab_cobot_manipulation.pick_place_node import (
    DEFAULT_MOVE_TIMEOUT_SEC,
    GRIPPER_TCP_LINK,
    PickPlace,
    UR_JOINTS,
)
import lab_cobot_manipulation.pick_place_node as pick_place_module


DEFAULT_RESULTS_DIR = "results/manipulation_validation/repeatability"
DEFAULT_TARGET_POSITION = [0.62, 0.0, 0.72]
DEFAULT_TARGET_QUAT = [1.0, 0.0, 0.0, 0.0]
DEFAULT_TARGET_FRAME = "base_link"
DEFAULT_TARGET_MODE = "pose"
DEFAULT_JOINT_GOAL_TOLERANCE = 0.001
MOVEIT_READINESS_TIMEOUT_SEC = 30.0
DEFAULT_TARGET_JOINTS = [
    -0.227129372998,
    -0.007090025542,
    -1.361750437475,
    -0.202464053725,
    -1.542899096459,
    -1.771548628147,
]
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
        self.node.declare_parameter("repeatability_initial_config_index", 0)
        self.node.declare_parameter("repeatability_target_mode", DEFAULT_TARGET_MODE)
        self.node.declare_parameter("repeatability_target_joints", DEFAULT_TARGET_JOINTS)
        self.node.declare_parameter(
            "repeatability_joint_goal_tolerance",
            DEFAULT_JOINT_GOAL_TOLERANCE,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)
        self._controller_state_history = deque(maxlen=3000)
        self._target_move_capture = None
        self.node.create_subscription(
            JointTrajectoryControllerState,
            "/joint_trajectory_controller/controller_state",
            self._controller_state_callback,
            20,
        )
        self._install_trajectory_capture()
        self.trials = int(self.node.get_parameter("repeatability_trials").value)
        self.initial_config_index = int(
            self.node.get_parameter("repeatability_initial_config_index").value
        )
        if self.initial_config_index < 0 or self.initial_config_index > len(INITIAL_CONFIGS):
            raise ValueError(
                "repeatability_initial_config_index must be 0 or 1..%d"
                % len(INITIAL_CONFIGS)
            )
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
        self.target_mode = str(
            self.node.get_parameter("repeatability_target_mode").value
        ).strip().lower()
        if self.target_mode not in ("pose", "joint"):
            raise ValueError("repeatability_target_mode must be 'pose' or 'joint'")
        self.target_joints = [
            float(value)
            for value in self.node.get_parameter("repeatability_target_joints").value
        ]
        if len(self.target_joints) != len(UR_JOINTS):
            raise ValueError(
                "repeatability_target_joints must contain %d values" % len(UR_JOINTS)
            )
        self.joint_goal_tolerance = float(
            self.node.get_parameter("repeatability_joint_goal_tolerance").value
        )
        if self.joint_goal_tolerance <= 0.0:
            raise ValueError("repeatability_joint_goal_tolerance must be positive")

    def run(self) -> Path:
        if not self._wait_for_moveit_ready():
            self.node.get_logger().error(
                "MoveIt readiness barrier failed; no repeatability trial started"
            )
            return self.output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        rows = []
        for trial_id in range(1, self.trials + 1):
            config_index, config = self._initial_config_for_trial(trial_id)
            self._target_move_capture = None
            started = time.monotonic()
            start_ok = self.pp._move_to_configuration(config, local_speed=True)  # noqa: SLF001
            if not start_ok:
                self.node.get_logger().warn(
                    "repeatability initial configuration failed trial=%d" % trial_id
                )
                target_ok = False
                samples = self._empty_terminal_samples()
            else:
                started = time.monotonic()
                target_ok = self._move_to_target()
                samples = self._sample_terminal_state()
            pose, actual_frame, measurement_source = samples["025"]["tcp"]
            duration = time.monotonic() - started
            ok = bool(start_ok and target_ok)
            target_error_mm = self._target_error_mm(pose, actual_frame)
            joint_final = samples["025"]["joints"]
            rows.append(
                {
                    "trial_id": trial_id,
                    "target_mode": self.target_mode,
                    "target_frame": DEFAULT_TARGET_FRAME,
                    "target_link": GRIPPER_TCP_LINK,
                    "initial_config_index": config_index,
                    "repeatability_joint_goal_tolerance": self.joint_goal_tolerance,
                    "initial_config_success": int(start_ok),
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
                    "actual_frame": actual_frame,
                    "target_move_success": int(target_ok),
                    "move_success": int(ok),
                    "failure_stage": (
                        "SUCCESS"
                        if ok
                        else (
                            "START_CONFIGURATION"
                            if not start_ok
                            else "TARGET_MOVE"
                        )
                    ),
                    "duration": duration,
                    "measurement_source": measurement_source,
                    "target_position_error_mm": target_error_mm,
                    **self._target_joint_columns(),
                    **self._target_joint_error_columns(samples["100"]["joints"]),
                    **self._trajectory_capture_columns(),
                    **self._tcp_sample_columns(samples),
                    **self._joint_sample_columns(joint_final, "final"),
                    **self._joint_sample_columns(samples["025"]["joints"], "t025"),
                    **self._joint_sample_columns(samples["100"]["joints"], "t100"),
                    **self._joint_sample_columns(samples["200"]["joints"], "t200"),
                    "max_abs_joint_velocity_t025": self._max_abs_joint_velocity(
                        samples["025"]["joints"]
                    ),
                    "max_abs_joint_velocity_t100": self._max_abs_joint_velocity(
                        samples["100"]["joints"]
                    ),
                    "max_abs_joint_velocity_t200": self._max_abs_joint_velocity(
                        samples["200"]["joints"]
                    ),
                    "drift_025_to_100_mm": self._tcp_distance_mm(
                        samples["025"]["tcp"][0], samples["100"]["tcp"][0]
                    ),
                    "drift_100_to_200_mm": self._tcp_distance_mm(
                        samples["100"]["tcp"][0], samples["200"]["tcp"][0]
                    ),
                    "drift_025_to_200_mm": self._tcp_distance_mm(
                        samples["025"]["tcp"][0], samples["200"]["tcp"][0]
                    ),
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

    def _wait_for_moveit_ready(self) -> bool:
        """Wait for the existing MoveIt planning and execution clients."""
        deadline = time.monotonic() + MOVEIT_READINESS_TIMEOUT_SEC
        planning_client = getattr(
            self.pp.moveit2, "_plan_kinematic_path_service", None
        )
        if planning_client is None:
            self.node.get_logger().error(
                "MoveIt planning service client is unavailable"
            )
            return False

        self.node.get_logger().info(
            "Waiting for MoveIt planning and execution interfaces before trial 1"
        )
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            if not planning_client.wait_for_service(
                timeout_sec=min(0.25, remaining)
            ):
                continue

            execution_client = getattr(self.pp, "_execute_trajectory_client", None)
            if execution_client is not None:
                remaining = max(0.0, deadline - time.monotonic())
                if not execution_client.wait_for_server(
                    timeout_sec=min(0.25, remaining)
                ):
                    continue
            self.node.get_logger().info("MoveIt planning and execution interfaces ready")
            return True

        self.node.get_logger().error(
            "MoveIt readiness timeout after %.1f s" % MOVEIT_READINESS_TIMEOUT_SEC
        )
        return False

    def _empty_terminal_samples(self) -> dict:
        nan_pose = (math.nan,) * 7
        nan_joints = {
            name: (math.nan, math.nan)
            for name in UR_JOINTS
        }
        return {
            label: {
                "tcp": (nan_pose, "", ""),
                "joints": dict(nan_joints),
            }
            for label in ("025", "100", "200")
        }

    def _controller_state_callback(self, message) -> None:
        def vector(field):
            return dict(zip(message.joint_names, getattr(message, field).positions))

        self._controller_state_history.append(
            (
                time.monotonic(),
                vector("desired"),
                vector("actual"),
                vector("error"),
            )
        )

    def _install_trajectory_capture(self) -> None:
        def snapshot(trajectory):
            points = list(getattr(trajectory, "points", []))
            final = points[-1] if points else None
            if final is None:
                return {}
            return {
                "joint_names": copy.deepcopy(list(trajectory.joint_names)),
                "positions": copy.deepcopy(list(final.positions)),
                "velocities": copy.deepcopy(list(final.velocities)),
                "accelerations": copy.deepcopy(list(final.accelerations)),
                "time_from_start": (
                    final.time_from_start.sec
                    + final.time_from_start.nanosec * 1e-9
                ),
            }

        def capture_joint_constraints(moveit2):
            goal = getattr(moveit2, "_MoveIt2__move_action_goal", None)
            constraints = []
            if goal is not None and goal.request.goal_constraints:
                for constraint in goal.request.goal_constraints[-1].joint_constraints:
                    constraints.append(
                        {
                            "joint_name": str(constraint.joint_name),
                            "position": float(constraint.position),
                            "tolerance_above": float(constraint.tolerance_above),
                            "tolerance_below": float(constraint.tolerance_below),
                            "weight": float(constraint.weight),
                        }
                    )
            return constraints

        original_set_joint_goal = self.pp.moveit2.set_joint_goal

        def capture_set_joint_goal(_moveit2, *args, **kwargs):
            result = original_set_joint_goal(*args, **kwargs)
            if self._target_move_capture is not None:
                self._target_move_capture["joint_constraints"] = (
                    capture_joint_constraints(_moveit2)
                )
            return result

        self.pp.moveit2.set_joint_goal = MethodType(
            capture_set_joint_goal, self.pp.moveit2
        )

        original_plan = self.pp.moveit2.plan

        def capture_plan(_moveit2, *args, **kwargs):
            trajectory = original_plan(*args, **kwargs)
            if self._target_move_capture is not None and trajectory is not None:
                self._target_move_capture["raw_plan"] = snapshot(trajectory)
            return trajectory

        self.pp.moveit2.plan = MethodType(capture_plan, self.pp.moveit2)

        original_normalize = self.pp._normalize_wrist_trajectory_to_current

        def capture_normalize(_pp, trajectory):
            result = original_normalize(trajectory)
            if self._target_move_capture is not None:
                self._target_move_capture["after_normalization"] = snapshot(trajectory)
            return result

        self.pp._normalize_wrist_trajectory_to_current = MethodType(
            capture_normalize, self.pp
        )

        original_timing = pick_place_module._ensure_trajectory_timing

        def capture_timing(trajectory, step_duration_sec=0.05):
            result = original_timing(trajectory, step_duration_sec)
            if self._target_move_capture is not None:
                self._target_move_capture["after_timing"] = snapshot(trajectory)
            return result

        pick_place_module._ensure_trajectory_timing = capture_timing

        original = self.pp._execute_trajectory_via_moveit

        def capture_execute(_pp, trajectory, timeout_sec):
            is_target = self._target_move_capture is not None
            if is_target:
                execute_input = snapshot(trajectory)
                self._target_move_capture["execute_input"] = execute_input
                # Preserve the existing planned_* CSV fields as the actual execute input.
                self._target_move_capture["planned"] = execute_input
            result = original(trajectory, timeout_sec)
            if is_target:
                self._target_move_capture["controller_success_time"] = time.monotonic()
            return result

        self.pp._execute_trajectory_via_moveit = MethodType(capture_execute, self.pp)

    def _trajectory_capture_columns(self) -> dict[str, float | str]:
        capture = self._target_move_capture or {}
        planned = capture.get("planned", {})
        names = planned.get("joint_names", [])
        index = {name: i for i, name in enumerate(names)}
        columns: dict[str, float | str] = {
            "planned_final_joint_names": ";".join(names),
            "planned_final_time_from_start": planned.get("time_from_start", math.nan),
        }
        constraints = {
            item["joint_name"]: item
            for item in capture.get("joint_constraints", [])
        }
        for label in ("raw_plan", "after_normalization", "after_timing", "execute_input"):
            layer = capture.get(label, {})
            layer_names = layer.get("joint_names", [])
            layer_index = {name: i for i, name in enumerate(layer_names)}
            layer_positions = layer.get("positions", [])
            layer_velocities = layer.get("velocities", [])
            layer_accelerations = layer.get("accelerations", [])
            columns[f"{label}_final_joint_names"] = ";".join(layer_names)
            columns[f"{label}_final_time_from_start"] = layer.get(
                "time_from_start", math.nan
            )
            for name in UR_JOINTS:
                safe = name.removesuffix("_joint")
                i = layer_index.get(name)
                columns[f"{label}_final_position_{safe}"] = (
                    layer_positions[i]
                    if i is not None and i < len(layer_positions)
                    else math.nan
                )
                columns[f"{label}_final_velocity_{safe}"] = (
                    layer_velocities[i]
                    if i is not None and i < len(layer_velocities)
                    else math.nan
                )
                columns[f"{label}_final_acceleration_{safe}"] = (
                    layer_accelerations[i]
                    if i is not None and i < len(layer_accelerations)
                    else math.nan
                )
        success_time = capture.get("controller_success_time", math.nan)
        nearest_success = None
        nearest_plus = None
        if not math.isnan(success_time):
            history = list(self._controller_state_history)
            before = [item for item in history if item[0] <= success_time]
            after = [item for item in history if item[0] >= success_time + 0.25]
            if before:
                nearest_success = before[-1]
            if after:
                nearest_plus = min(after, key=lambda item: abs(item[0] - (success_time + 0.25)))
        for name in UR_JOINTS:
            safe = name.removesuffix("_joint")
            constraint = constraints.get(name, {})
            columns[f"motion_plan_constraint_position_{safe}"] = constraint.get(
                "position", math.nan
            )
            columns[f"motion_plan_constraint_tolerance_above_{safe}"] = constraint.get(
                "tolerance_above", math.nan
            )
            columns[f"motion_plan_constraint_tolerance_below_{safe}"] = constraint.get(
                "tolerance_below", math.nan
            )
            columns[f"motion_plan_constraint_weight_{safe}"] = constraint.get(
                "weight", math.nan
            )
            i = index.get(name)
            columns[f"planned_final_position_{safe}"] = (
                planned.get("positions", [])[i] if i is not None else math.nan
            )
            columns[f"planned_final_velocity_{safe}"] = (
                planned.get("velocities", [])[i] if i is not None and i < len(planned.get("velocities", [])) else math.nan
            )
            columns[f"planned_final_acceleration_{safe}"] = (
                planned.get("accelerations", [])[i] if i is not None and i < len(planned.get("accelerations", [])) else math.nan
            )
            for label, state in (("controller_success", nearest_success), ("controller_success_plus_025", nearest_plus)):
                columns[f"{label}_desired_{safe}"] = state[1].get(name, math.nan) if state else math.nan
                columns[f"{label}_actual_{safe}"] = state[2].get(name, math.nan) if state else math.nan
                columns[f"{label}_error_{safe}"] = state[3].get(name, math.nan) if state else math.nan
            target = self.target_joints[UR_JOINTS.index(name)]
            planned_position = columns[f"planned_final_position_{safe}"]
            columns[f"requested_to_planned_error_{safe}"] = planned_position - target if not math.isnan(planned_position) else math.nan
            desired = columns[f"controller_success_desired_{safe}"]
            actual = columns[f"controller_success_actual_{safe}"]
            columns[f"planned_to_desired_error_{safe}"] = desired - planned_position if not math.isnan(desired) and not math.isnan(planned_position) else math.nan
            columns[f"desired_to_actual_error_{safe}"] = actual - desired if not math.isnan(actual) and not math.isnan(desired) else math.nan
            columns[f"requested_to_actual_error_{safe}"] = actual - target if not math.isnan(actual) else math.nan
        return columns

    def _move_to_target(self) -> bool:
        if self.target_mode == "joint":
            self._target_move_capture = {}
            try:
                return self._move_to_joint_target()
            finally:
                # Keep the captured target execution and its controller snapshots for CSV output.
                pass
        return self.pp._move(  # noqa: SLF001
            self.target_position,
            quat=self.target_quat,
            frame_id=DEFAULT_TARGET_FRAME,
            target_link=GRIPPER_TCP_LINK,
            tolerance_position=0.001,
            tolerance_orientation=0.05,
            timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
            cartesian=False,
            stabilize_wrist=True,
            local_speed=False,
            fallback_to_ompl=True,
        )

    def _move_to_joint_target(self) -> bool:
        send_name = "_MoveIt2__send_goal_future_follow_joint_trajectory"
        result_name = "_MoveIt2__get_result_future_follow_joint_trajectory"
        previous_send_future = getattr(self.pp.moveit2, send_name, None)
        previous_result_future = getattr(self.pp.moveit2, result_name, None)

        def plan_and_execute():
            if getattr(self.pp, "_execute_trajectory_client", None) is not None:
                trajectory = self.pp.moveit2.plan(
                    joint_positions=list(self.target_joints),
                    tolerance_joint_position=self.joint_goal_tolerance,
                )
                if trajectory is None:
                    return False
                self.pp._normalize_wrist_trajectory_to_current(trajectory)  # noqa: SLF001
                pick_place_module._ensure_trajectory_timing(trajectory)  # noqa: SLF001
                return self.pp._execute_trajectory_via_moveit(  # noqa: SLF001
                    trajectory,
                    DEFAULT_MOVE_TIMEOUT_SEC,
                )
            self.pp.moveit2.move_to_configuration(
                list(self.target_joints),
                tolerance=self.joint_goal_tolerance,
            )
            return pick_place_module._wait_for_moveit_result(  # noqa: SLF001
                self.pp.moveit2,
                DEFAULT_MOVE_TIMEOUT_SEC,
                previous_send_future=previous_send_future,
                previous_result_future=previous_result_future,
                action_future_names=[(send_name, result_name)],
            )

        return self.pp._with_local_arm_scaling(False, plan_and_execute)  # noqa: SLF001

    def _initial_config_for_trial(self, trial_id: int) -> tuple[int, list[float]]:
        if self.initial_config_index:
            index = self.initial_config_index
        else:
            index = (trial_id - 1) % len(INITIAL_CONFIGS) + 1
        return index, list(INITIAL_CONFIGS[index - 1])

    def _actual_tcp_pose(
        self,
    ) -> tuple[tuple[float, float, float, float, float, float, float], str, str]:
        try:
            transform = self._tf_buffer.lookup_transform(
                DEFAULT_TARGET_FRAME,
                GRIPPER_TCP_LINK,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return (
                (
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                ),
                DEFAULT_TARGET_FRAME,
                "TF %s -> %s" % (DEFAULT_TARGET_FRAME, GRIPPER_TCP_LINK),
            )
        except TransformException as exc:
            self.node.get_logger().warn(
                "repeatability TF lookup failed; falling back to MoveIt FK: %s" % exc
            )
        pose_stamped = self.pp.moveit2.compute_fk(fk_link_names=[GRIPPER_TCP_LINK])
        if isinstance(pose_stamped, list):
            pose_stamped = pose_stamped[0]
        pose = pose_stamped.pose
        return (
            (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
            str(getattr(pose_stamped.header, "frame_id", "") or "moveit_fk"),
            "MoveIt FK from current joint_state",
        )

    def _sample_terminal_state(self) -> dict:
        samples = {}
        for label, delay in (("025", 0.25), ("100", 0.75), ("200", 1.0)):
            time.sleep(delay)
            samples[label] = {
                "tcp": self._actual_tcp_pose(),
                "joints": self._current_arm_joint_state(),
            }
        return samples

    def _current_arm_joint_state(self) -> dict[str, tuple[float, float]]:
        joint_state = getattr(self.pp.moveit2, "joint_state", None)
        positions = {}
        velocities = {}
        if joint_state is not None:
            positions = {
                name: float(position)
                for name, position in zip(joint_state.name, joint_state.position)
            }
            velocities = {
                name: float(velocity)
                for name, velocity in zip(joint_state.name, joint_state.velocity)
            }
        return {
            name: (
                positions.get(name, math.nan),
                velocities.get(name, math.nan),
            )
            for name in UR_JOINTS
        }

    def _tcp_sample_columns(self, samples: dict) -> dict[str, float | str]:
        columns = {}
        for label in ("025", "100", "200"):
            pose, frame, source = samples[label]["tcp"]
            columns.update(
                {
                    f"tcp_{label}_x": pose[0],
                    f"tcp_{label}_y": pose[1],
                    f"tcp_{label}_z": pose[2],
                    f"tcp_{label}_qx": pose[3],
                    f"tcp_{label}_qy": pose[4],
                    f"tcp_{label}_qz": pose[5],
                    f"tcp_{label}_qw": pose[6],
                    f"tcp_{label}_frame": frame,
                    f"tcp_{label}_source": source,
                }
            )
        return columns

    def _joint_sample_columns(
        self,
        joints: dict[str, tuple[float, float]],
        label: str,
    ) -> dict[str, float]:
        columns = {}
        for name in UR_JOINTS:
            safe_name = name.removesuffix("_joint")
            position, velocity = joints[name]
            columns[f"{label}_joint_position_{safe_name}"] = position
            columns[f"{label}_joint_velocity_{safe_name}"] = velocity
        return columns

    def _target_joint_columns(self) -> dict[str, float]:
        return {
            "target_joint_position_%s" % name.removesuffix("_joint"): value
            for name, value in zip(UR_JOINTS, self.target_joints)
        }

    def _target_joint_error_columns(
        self,
        joints: dict[str, tuple[float, float]],
    ) -> dict[str, float]:
        columns = {}
        for name, target in zip(UR_JOINTS, self.target_joints):
            safe_name = name.removesuffix("_joint")
            actual, _ = joints[name]
            columns[f"target_joint_error_{safe_name}"] = actual - target
        return columns

    def _max_abs_joint_velocity(self, joints: dict[str, tuple[float, float]]) -> float:
        velocities = [abs(velocity) for _, velocity in joints.values() if not math.isnan(velocity)]
        return max(velocities) if velocities else math.nan

    def _tcp_distance_mm(
        self,
        a: tuple[float, float, float, float, float, float, float],
        b: tuple[float, float, float, float, float, float, float],
    ) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0

    def _target_error_mm(
        self,
        pose: tuple[float, float, float, float, float, float, float],
        actual_frame: str,
    ) -> float:
        if actual_frame != DEFAULT_TARGET_FRAME:
            return math.nan
        dx = pose[0] - self.target_position[0]
        dy = pose[1] - self.target_position[1]
        dz = pose[2] - self.target_position[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0

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
        measurement_source = (
            str(ok_rows[0]["measurement_source"])
            if ok_rows
            else str(rows[0].get("measurement_source", ""))
        )
        if errors:
            rms = math.sqrt(statistics.fmean(value * value for value in errors))
            drift_025_to_100 = [
                float(row["drift_025_to_100_mm"]) for row in rows if int(row["move_success"])
            ]
            drift_100_to_200 = [
                float(row["drift_100_to_200_mm"]) for row in rows if int(row["move_success"])
            ]
            drift_025_to_200 = [
                float(row["drift_025_to_200_mm"]) for row in rows if int(row["move_success"])
            ]
            summary = [
                f"trials: {len(rows)}",
                f"successful_moves: {len(ok_rows)}",
                "measurement_source: %s" % measurement_source,
                "target_mode: %s" % self.target_mode,
                "target_frame: %s" % DEFAULT_TARGET_FRAME,
                "target_link: %s" % GRIPPER_TCP_LINK,
                f"mean_error_mm: {statistics.fmean(errors):.6f}",
                f"std_error_mm: {(statistics.pstdev(errors) if len(errors) > 1 else 0.0):.6f}",
                f"rms_error_mm: {rms:.6f}",
                f"max_error_mm: {max(errors):.6f}",
                "drift_025_to_100_mean_mm: %.6f" % statistics.fmean(drift_025_to_100),
                "drift_025_to_100_max_mm: %.6f" % max(drift_025_to_100),
                "drift_100_to_200_mean_mm: %.6f" % statistics.fmean(drift_100_to_200),
                "drift_100_to_200_max_mm: %.6f" % max(drift_100_to_200),
                "drift_025_to_200_mean_mm: %.6f" % statistics.fmean(drift_025_to_200),
                "drift_025_to_200_max_mm: %.6f" % max(drift_025_to_200),
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
