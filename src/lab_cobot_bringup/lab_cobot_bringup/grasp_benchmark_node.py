#!/usr/bin/env python3
"""Run repeated grasp-validation trials and save manipulation validation CSV evidence."""
from __future__ import annotations

import csv
import math
import time
from pathlib import Path

from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState, ModelStates
from geometry_msgs.msg import Twist
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from lab_cobot_bringup.grasp_target_config import get_target_config, target_names
from lab_cobot_manipulation.pick_place_node import HOME_CONFIG, UR_JOINTS


TARGET_TOPIC = "/grasp_validation/target"
STATUS_TOPIC = "/grasp_validation/status"
CONTACT_STATUS_TOPIC = "/gripper/contact/status"
CONTACT_FINGERS_TOPIC = "/gripper/contact/fingers"
CONTACT_FORCE_TOPIC = "/gripper/contact/force"
CONTACT_FORCE_SOURCE_TOPIC = "/gripper/contact/force_source"
FORCE_CONTROL_STATUS_TOPIC = "/gripper/force_control/status"
CONTACT_RELEASE_TOPIC = "/gripper/contact/release"
SET_ENTITY_STATE_SERVICE = "/gazebo/set_entity_state"
JOINT_TRAJECTORY_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
GRIPPER_COMMAND_TOPIC = "/gripper_position_controller/commands"
TERMINAL_PREFIXES = ("SUCCESS", "FAILED_", "BUSY")
DEFAULT_RESULTS_DIR = "results/manipulation_validation"
GRIPPER_OPEN_POSITIONS = [0.0, 0.0]
GRIPPER_OPEN_TOLERANCE = 0.0015
SETTLE_POSITION_TOLERANCE_M = 0.001
RESET_POSE_TOLERANCE_M = 0.004
SETTLE_LINEAR_VELOCITY_MPS = 0.015
SETTLE_ANGULAR_VELOCITY_RPS = 0.05
MIN_SETTLE_OBSERVATION_SEC = 0.5
ARM_RESET_TIMEOUT_SEC = 90.0
ARM_RESET_DURATION_SEC = 6
ARM_RESET_JOINT_TOLERANCE_RAD = 0.006
INITIAL_STATE_TIMEOUT_SEC = 45.0


class GraspBenchmarkNode(Node):
    """Publishes validation targets, records terminal status, and writes CSV."""

    def __init__(self):
        super().__init__("grasp_benchmark_node")
        self.declare_parameter("target", "material_spare_igbt")
        self.declare_parameter("trials", 10)
        self.declare_parameter("output_dir", DEFAULT_RESULTS_DIR)
        self.declare_parameter("trial_timeout_sec", 420.0)
        self.declare_parameter("reset_only", False)
        self.declare_parameter("reset_settle_timeout_sec", 8.0)
        self.declare_parameter("record_force_timeseries", False)
        self.target = str(self.get_parameter("target").value).strip()
        self.trials = int(self.get_parameter("trials").value)
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.trial_timeout_sec = float(self.get_parameter("trial_timeout_sec").value)
        self.reset_only = bool(self.get_parameter("reset_only").value)
        self.reset_settle_timeout_sec = float(
            self.get_parameter("reset_settle_timeout_sec").value
        )
        self.record_force_timeseries = bool(
            self.get_parameter("record_force_timeseries").value
        )
        if self.target not in target_names():
            raise ValueError(f"unknown manipulation validation grasp target: {self.target}")
        if self.trials <= 0:
            raise ValueError("trials must be positive")
        self._status_events: list[tuple[float, str]] = []
        self._contact_status = ""
        self._contact_left_seen = False
        self._contact_right_seen = False
        self._left_force = 0.0
        self._right_force = 0.0
        self._current_left_force = 0.0
        self._current_right_force = 0.0
        self._force_source = "INVALID"
        self._force_valid_left = False
        self._force_valid_right = False
        self._force_valid_left_seen = False
        self._force_valid_right_seen = False
        self._force_samples = 0
        self._active_trial_id = 0
        self._active_trial_start = None
        self._force_timeseries_rows: list[dict] = []
        self._force_control_rows: list[dict] = []
        self._last_fingers = ""
        self._last_joint_state: JointState | None = None
        self._latest_model_states: ModelStates | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._target_pub = self.create_publisher(String, TARGET_TOPIC, 10)
        self._release_pub = self.create_publisher(Empty, CONTACT_RELEASE_TOPIC, 10)
        self._gripper_pub = self.create_publisher(
            Float64MultiArray,
            GRIPPER_COMMAND_TOPIC,
            10,
        )
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            JOINT_TRAJECTORY_ACTION,
        )
        self._set_entity_client = self.create_client(
            SetEntityState,
            SET_ENTITY_STATE_SERVICE,
        )
        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        self.create_subscription(String, CONTACT_STATUS_TOPIC, self._on_contact_status, 10)
        self.create_subscription(String, CONTACT_FINGERS_TOPIC, self._on_contact_fingers, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._on_model_states, 10)
        self.create_subscription(
            Float64MultiArray,
            CONTACT_FORCE_TOPIC,
            self._on_contact_force,
            10,
        )
        self.create_subscription(
            String,
            CONTACT_FORCE_SOURCE_TOPIC,
            self._on_contact_force_source,
            10,
        )
        self.create_subscription(
            String,
            FORCE_CONTROL_STATUS_TOPIC,
            self._on_force_control_status,
            10,
        )

    def run(self) -> Path:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = self.output_dir / (
            "reset_snapshots.csv" if self.reset_only else "grasp_trials.csv"
        )
        self._wait_initial_state()
        rows = []
        for trial_id in range(1, self.trials + 1):
            if self.reset_only:
                rows.append(self._run_reset_only_trial(trial_id))
            else:
                rows.append(self._run_trial(trial_id))
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            fieldnames = (
                self._reset_fieldnames()
                if self.reset_only
                else self._trial_fieldnames()
            )
            writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        self.get_logger().info(f"wrote {csv_path}")
        if self.record_force_timeseries:
            force_path = self.output_dir / "contact_force_timeseries.csv"
            with force_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=self._force_timeseries_fieldnames(),
                    extrasaction="ignore",
                )
                writer.writeheader()
                writer.writerows(self._force_timeseries_rows)
            self.get_logger().info(f"wrote {force_path}")
            control_path = self.output_dir / "force_control_timeseries.csv"
            with control_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=self._force_control_fieldnames(),
                    extrasaction="ignore",
                )
                writer.writeheader()
                writer.writerows(self._force_control_rows)
            self.get_logger().info(f"wrote {control_path}")
        return csv_path

    def _wait_initial_state(self) -> bool:
        deadline = time.monotonic() + INITIAL_STATE_TIMEOUT_SEC
        while time.monotonic() < deadline:
            joint_ready = self._last_joint_state is not None
            model_ready = (
                self._latest_model_states is not None
                and self.target in self._latest_model_states.name
            )
            arm_ready = self._arm_client.wait_for_server(timeout_sec=0.05)
            set_entity_ready = self._set_entity_client.wait_for_service(timeout_sec=0.05)
            if joint_ready and model_ready and arm_ready and set_entity_ready:
                self.get_logger().info(
                    "INITIAL_STATE_READY target=%s joint_state=1 model_state=1 arm_action=1 set_entity=1"
                    % self.target
                )
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn(
            "INITIAL_STATE_TIMEOUT target=%s joint_state=%d model_state=%d arm_action=%d set_entity=%d"
            % (
                self.target,
                int(self._last_joint_state is not None),
                int(
                    self._latest_model_states is not None
                    and self.target in self._latest_model_states.name
                ),
                int(self._arm_client.wait_for_server(timeout_sec=0.05)),
                int(self._set_entity_client.wait_for_service(timeout_sec=0.05)),
            )
        )
        return False

    @classmethod
    def _trial_fieldnames(cls) -> list[str]:
        fields = [
                "target",
                "trial",
                "trial_id",
                "planning_success",
                "cartesian_success",
                "cartesian_fraction",
                "contact_success",
                "contact_left",
                "contact_right",
                "force_left",
                "force_right",
                "force_peak",
                "force_source",
                "force_valid_left",
            "force_valid_right",
            "force_samples",
            "force_control_samples",
            "force_control_final_state",
            "force_control_success",
            "attach_success",
                "holding_success",
                "lift_success",
                "failure_stage",
                "failure_reason",
                "duration",
                "stage_planning_sec",
                "stage_descend_sec",
                "stage_contact_sec",
                "stage_attach_sec",
                "stage_lift_sec",
                "lift_attempt",
                "lift_fraction",
                "lift_duration",
                "lift_distance",
                "lift_start_tcp_x",
                "lift_start_tcp_y",
                "lift_start_tcp_z",
                "lift_target_tcp_x",
                "lift_target_tcp_y",
                "lift_target_tcp_z",
                "lift_final_tcp_x",
                "lift_final_tcp_y",
                "lift_final_tcp_z",
                "max_start_joint_error",
                "lift_error_code",
                "lift_error_text",
                "lift_primary_success",
                "lift_primary_error_code",
                "lift_primary_error_text",
                "lift_recovery_attempted",
                "lift_recovery_fraction",
                "lift_recovery_success",
                "lift_recovery_error_code",
                "lift_recovery_error_text",
                "lift_state_verified",
                "lift_state_final_target_error",
                "lift_state_z_error",
                "lift_final_success",
                "settle_sec",
                "reset_ok",
        ]
        fields.extend(field for field in cls._reset_fieldnames() if field not in fields)
        return fields

    def _run_trial(self, trial_id: int) -> dict:
        self.get_logger().info(f"MANIPULATION_VALIDATION_GRASP_TRIAL_RESET target={self.target} trial={trial_id}")
        reset = self._prepare_trial_reset(trial_id)
        if int(reset.get("reset_ok", 0)) != 1:
            row = self._reset_failure_row(trial_id, reset)
            self.get_logger().warn(
                "MANIPULATION_VALIDATION_GRASP_TRIAL_ABORT_RESET target=%s trial=%d"
                % (self.target, trial_id)
            )
            return row
        self.get_logger().info(f"MANIPULATION_VALIDATION_GRASP_TRIAL_START target={self.target} trial={trial_id}")
        self._status_events.clear()
        self._contact_status = ""
        self._contact_left_seen = False
        self._contact_right_seen = False
        self._left_force = 0.0
        self._right_force = 0.0
        self._current_left_force = 0.0
        self._current_right_force = 0.0
        self._force_valid_left = False
        self._force_valid_right = False
        self._force_valid_left_seen = False
        self._force_valid_right_seen = False
        self._force_samples = 0
        force_control_start_count = len(self._force_control_rows)
        start = time.monotonic()
        self._active_trial_id = trial_id
        self._active_trial_start = start
        msg = String()
        msg.data = self.target
        self._publish_until_first_status(msg, start)
        terminal = self._wait_terminal(start)
        duration = time.monotonic() - start
        statuses = [status for _, status in self._status_events]
        seen = lambda prefix: any(status.startswith(prefix) for status in statuses)
        failure_reason = "none" if terminal.startswith("SUCCESS") else terminal
        contact_left = int(self._contact_left_seen or self._left_force > 0.0)
        contact_right = int(self._contact_right_seen or self._right_force > 0.0)
        row = {
            "target": self.target,
            "trial": trial_id,
            "trial_id": trial_id,
            "planning_success": int(seen("PRE_GRASP_REACHED")),
            "cartesian_success": int(seen("GRIPPING") or seen("CONTACT_OK")),
            "cartesian_fraction": "",
            "contact_success": int(contact_left and contact_right),
            "contact_left": contact_left,
            "contact_right": contact_right,
            "force_left": f"{self._left_force:.6f}",
            "force_right": f"{self._right_force:.6f}",
            "force_peak": f"{max(self._left_force, self._right_force):.6f}",
            "force_source": self._force_source,
            "force_valid_left": int(self._force_valid_left_seen),
            "force_valid_right": int(self._force_valid_right_seen),
            "force_samples": self._force_samples,
            "force_control_samples": len(self._force_control_rows)
            - force_control_start_count,
            "force_control_final_state": self._force_control_final_state(
                trial_id
            ),
            "force_control_success": int(
                self._force_control_final_state(trial_id) == "FORCE_HOLD"
            ),
            "attach_success": int(seen("ATTACHED")),
            "holding_success": int(seen("HOLDING") or terminal.startswith("SUCCESS")),
            "lift_success": int(seen("HOLDING") or terminal.startswith("SUCCESS")),
            "failure_stage": self._failure_stage(failure_reason, contact_left, contact_right),
            "failure_reason": failure_reason,
            "duration": f"{duration:.6f}",
        }
        row.update(self._stage_durations())
        row.update(self._lift_diagnostics())
        row.update(reset)
        self.get_logger().info(
            "MANIPULATION_VALIDATION_GRASP_TRIAL_DONE target=%s trial=%d terminal=%s duration=%.3f"
            % (self.target, trial_id, terminal, duration)
        )
        self._active_trial_id = 0
        self._active_trial_start = None
        return row

    def _reset_failure_row(self, trial_id: int, reset: dict) -> dict:
        row = {
            "target": self.target,
            "trial": trial_id,
            "trial_id": trial_id,
            "planning_success": 0,
            "cartesian_success": 0,
            "cartesian_fraction": "",
            "contact_success": 0,
            "contact_left": int(self._finger_contacts()[0]),
            "contact_right": int(self._finger_contacts()[1]),
            "force_left": f"{self._left_force:.6f}",
            "force_right": f"{self._right_force:.6f}",
            "force_peak": f"{max(self._left_force, self._right_force):.6f}",
            "force_source": self._force_source,
            "force_valid_left": int(self._force_valid_left),
            "force_valid_right": int(self._force_valid_right),
            "force_samples": self._force_samples,
            "force_control_samples": 0,
            "force_control_final_state": "",
            "force_control_success": 0,
            "attach_success": 0,
            "holding_success": 0,
            "lift_success": 0,
            "failure_stage": "RESET_NOT_CLEAN",
            "failure_reason": "RESET_NOT_CLEAN",
            "duration": "0.000000",
            "stage_planning_sec": "",
            "stage_descend_sec": "",
            "stage_contact_sec": "",
            "stage_attach_sec": "",
            "stage_lift_sec": "",
            "lift_attempt": "",
            "lift_fraction": "",
            "lift_duration": "",
            "lift_distance": "",
            "lift_start_tcp_x": "",
            "lift_start_tcp_y": "",
            "lift_start_tcp_z": "",
            "lift_target_tcp_x": "",
            "lift_target_tcp_y": "",
            "lift_target_tcp_z": "",
            "lift_final_tcp_x": "",
            "lift_final_tcp_y": "",
            "lift_final_tcp_z": "",
            "max_start_joint_error": "",
            "lift_error_code": "",
            "lift_error_text": "",
            "lift_primary_success": "",
            "lift_primary_error_code": "",
            "lift_primary_error_text": "",
            "lift_recovery_attempted": "0",
            "lift_recovery_fraction": "",
            "lift_recovery_success": "",
            "lift_recovery_error_code": "",
            "lift_recovery_error_text": "",
            "lift_state_verified": "0",
            "lift_state_final_target_error": "",
            "lift_state_z_error": "",
            "lift_final_success": "",
        }
        row.update(reset)
        return row

    @staticmethod
    def _force_timeseries_fieldnames() -> list[str]:
        return [
            "target",
            "trial",
            "elapsed_sec",
            "sim_time",
            "left_contact",
            "right_contact",
            "left_force_raw",
            "right_force_raw",
            "force_source",
            "force_valid_left",
            "force_valid_right",
            "gripper_left",
            "gripper_right",
            "gripper_gap_mm",
        ]

    @staticmethod
    def _force_control_fieldnames() -> list[str]:
        return [
            "target",
            "trial",
            "elapsed_sec",
            "sim_time",
            "control_state",
            "gripper_command_left",
            "gripper_command_right",
            "gripper_position_left",
            "gripper_position_right",
            "force_left_raw",
            "force_right_raw",
            "force_left_filtered",
            "force_right_filtered",
            "force_mean",
            "force_error",
            "force_balance_error",
            "delta_q",
            "force_source",
            "force_valid_left",
            "force_valid_right",
        ]

    def _run_reset_only_trial(self, trial_id: int) -> dict:
        self.get_logger().info(
            f"MANIPULATION_VALIDATION_RESET_ONLY_START target={self.target} trial={trial_id}"
        )
        reset = self._prepare_trial_reset(trial_id)
        snapshot = self._snapshot(trial_id, "TRIAL_START")
        self._log_snapshot(snapshot)
        row = {"target": self.target, "trial": trial_id, "trial_id": trial_id}
        row.update(self._flatten_snapshot("trial_start", snapshot))
        row.update(reset)
        self.get_logger().info(
            f"MANIPULATION_VALIDATION_RESET_ONLY_DONE target={self.target} trial={trial_id}"
        )
        return row

    @staticmethod
    def _reset_fieldnames() -> list[str]:
        prefixes = ("reset_before", "reset_after", "trial_start")
        fields = ["target", "trial", "trial_id"]
        suffixes = (
            "object_x",
            "object_y",
            "object_z",
            "object_yaw",
            "object_v",
            "object_w",
            "tcp_x",
            "tcp_y",
            "tcp_z",
            "arm_max_abs_err",
            "gripper_left",
            "gripper_right",
            "gripper_gap_mm",
            "contact_left",
            "contact_right",
            "force_left",
            "force_right",
            "force_source",
            "force_valid_left",
            "force_valid_right",
            "attached",
            "holding",
            "sim_time",
        )
        for prefix in prefixes:
            fields.extend(f"{prefix}_{suffix}" for suffix in suffixes)
        fields.append("settle_sec")
        fields.append("reset_ok")
        return fields

    def _prepare_trial_reset(self, trial_id: int) -> dict:
        before = self._snapshot(trial_id, "RESET_BEFORE")
        self._log_snapshot(before)
        self._release_pub.publish(Empty())
        self._command_gripper_open()
        if self.target == "tooling_fixture_box":
            arm_reset_ok = self._move_arm_home()
            self._reset_target_pose()
        else:
            self._reset_target_pose()
            arm_reset_ok = self._move_arm_home()
        settle_sec = self._wait_object_settled()
        self._wait_gripper_open()
        after = self._snapshot(trial_id, "RESET_AFTER")
        self._log_snapshot(after)
        start = self._snapshot(trial_id, "TRIAL_START")
        self._log_snapshot(start)
        reset_ok = (
            not after["attached"]
            and not after["holding"]
            and not after["contact_left"]
            and not after["contact_right"]
            and after["gripper_gap_mm"] >= 88.0
            and after["object_v"] <= SETTLE_LINEAR_VELOCITY_MPS
            and after["object_w"] <= SETTLE_ANGULAR_VELOCITY_RPS
            and arm_reset_ok
            and after["arm_max_abs_err"] <= ARM_RESET_JOINT_TOLERANCE_RAD
        )
        if reset_ok:
            self.get_logger().info(
                "RESET_OK target=%s trial=%d settle_sec=%.3f"
                % (self.target, trial_id, settle_sec)
            )
        else:
            self.get_logger().warn(
                "RESET_NOT_CLEAN target=%s trial=%d settle_sec=%.3f"
                % (self.target, trial_id, settle_sec)
            )
        row = {}
        row.update(self._flatten_snapshot("reset_before", before))
        row.update(self._flatten_snapshot("reset_after", after))
        row.update(self._flatten_snapshot("trial_start", start))
        row["settle_sec"] = f"{settle_sec:.6f}"
        row["reset_ok"] = int(reset_ok)
        return row

    def _wait_terminal(self, start: float) -> str:
        deadline = start + self.trial_timeout_sec
        while time.monotonic() < deadline:
            for _, status in reversed(self._status_events):
                if status.startswith(TERMINAL_PREFIXES):
                    return status
            rclpy.spin_once(self, timeout_sec=0.1)
        return "FAILED_TIMEOUT"

    def _publish_until_first_status(self, msg: String, start: float) -> None:
        deadline = min(start + 30.0, start + self.trial_timeout_sec)
        next_publish = start
        while time.monotonic() < deadline:
            if self._status_events:
                return
            now = time.monotonic()
            if now >= next_publish:
                self._target_pub.publish(msg)
                next_publish = now + 1.0
            rclpy.spin_once(self, timeout_sec=0.1)
        self._target_pub.publish(msg)

    def _stage_durations(self) -> dict[str, str]:
        times = {status.split()[0]: stamp for stamp, status in self._status_events}

        def delta(a: str, b: str) -> str:
            if a not in times or b not in times:
                return ""
            return f"{max(0.0, times[b] - times[a]):.6f}"

        return {
            "stage_planning_sec": delta("PLANNING", "PRE_GRASP_REACHED"),
            "stage_descend_sec": delta("DESCENDING", "GRIPPING"),
            "stage_contact_sec": delta("GRIPPING", "CONTACT_OK"),
            "stage_attach_sec": delta("CONTACT_OK", "ATTACHED"),
            "stage_lift_sec": delta("LIFTING", "HOLDING"),
        }

    def _lift_diagnostics(self) -> dict[str, str]:
        fields = {
            "lift_fraction": "",
            "lift_duration": "",
            "lift_distance": "",
            "lift_start_tcp_x": "",
            "lift_start_tcp_y": "",
            "lift_start_tcp_z": "",
            "lift_target_tcp_x": "",
            "lift_target_tcp_y": "",
            "lift_target_tcp_z": "",
            "lift_final_tcp_x": "",
            "lift_final_tcp_y": "",
            "lift_final_tcp_z": "",
            "max_start_joint_error": "",
            "lift_error_code": "",
            "lift_error_text": "",
            "lift_attempt": "",
            "lift_primary_success": "",
            "lift_primary_error_code": "",
            "lift_primary_error_text": "",
            "lift_recovery_attempted": "0",
            "lift_recovery_fraction": "",
            "lift_recovery_success": "",
            "lift_recovery_error_code": "",
            "lift_recovery_error_text": "",
            "lift_state_verified": "0",
            "lift_state_final_target_error": "",
            "lift_state_z_error": "",
            "lift_final_success": "",
        }
        for _, status in self._status_events:
            token = status.split()[0] if status else ""
            if token not in ("LIFT_DIAGNOSTIC", "LIFT_EXECUTION_RESULT", "LIFT_STATE_VERIFIED"):
                continue
            parsed = self._parse_status_fields(status)
            if token == "LIFT_STATE_VERIFIED":
                ok = parsed.get("ok", "")
                fields["lift_state_verified"] = ok or "0"
                fields["lift_state_final_target_error"] = parsed.get(
                    "final_target_error", ""
                )
                fields["lift_state_z_error"] = parsed.get("z_error", "")
                if ok in ("0", "1"):
                    fields["lift_final_success"] = ok
                continue
            for key in fields:
                if key in parsed:
                    fields[key] = parsed[key]
            if token != "LIFT_EXECUTION_RESULT":
                continue
            attempt = parsed.get("lift_attempt", "")
            ok = parsed.get("ok", "")
            if attempt == "primary":
                fields["lift_primary_success"] = ok
                fields["lift_primary_error_code"] = parsed.get("lift_error_code", "")
                fields["lift_primary_error_text"] = parsed.get("lift_error_text", "")
            elif attempt == "scene_detached":
                fields["lift_recovery_attempted"] = "1"
                fields["lift_recovery_fraction"] = parsed.get("lift_fraction", "")
                fields["lift_recovery_success"] = ok
                fields["lift_recovery_error_code"] = parsed.get("lift_error_code", "")
                fields["lift_recovery_error_text"] = parsed.get("lift_error_text", "")
            if ok in ("0", "1"):
                fields["lift_final_success"] = ok
        return fields

    @staticmethod
    def _parse_status_fields(status: str) -> dict[str, str]:
        parsed = {}
        for part in status.split()[2:]:
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            parsed[key] = value
        return parsed

    @staticmethod
    def _failure_stage(failure_reason: str, contact_left: int = 0, contact_right: int = 0) -> str:
        if failure_reason == "none":
            return "SUCCESS"
        token = failure_reason.split()[0] if failure_reason else "UNKNOWN"
        if token == "FAILED_CONTACT":
            if contact_left and not contact_right:
                return "CONTACT_LEFT_ONLY"
            if contact_right and not contact_left:
                return "CONTACT_RIGHT_ONLY"
            if not contact_left and not contact_right:
                return "CONTACT_NONE"
            return "CONTACT_TIMEOUT"
        if token == "FAILED_ATTACH":
            if contact_left and contact_right:
                return "ATTACH_TIMEOUT"
            return "ATTACH_NO_FINGER_CONTACT"
        if token == "FAILED_DESCEND":
            return "DESCEND_EXECUTION"
        if token == "FAILED_HOLD_LOST":
            return "HOLD_LOST"
        if token == "FAILED_LIFT":
            return "LIFT_EXECUTION"
        if token == "FAILED_LIFT_PATH":
            return "LIFT_PATH"
        if token == "FAILED_LIFT_FRACTION":
            return "LIFT_FRACTION"
        if token == "FAILED_LIFT_EXECUTION":
            return "LIFT_EXECUTION"
        if token.startswith("FAILED_"):
            return token.removeprefix("FAILED_")
        return token

    def _reset_target_pose(self) -> None:
        self._set_target_pose_zero_twist("primary")
        if self.target == "tooling_fixture_box":
            self._spin_for(0.20)
            self._set_target_pose_zero_twist("post_physics_zero_twist")
            self._spin_for(0.10)

    def _set_target_pose_zero_twist(self, step: str) -> None:
        config = get_target_config(self.target)
        if config is None or not self._set_entity_client.wait_for_service(timeout_sec=0.5):
            return
        pose = config["world_pose"]
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.target
        request.state.reference_frame = "world"
        request.state.pose.position.x = float(pose["x"])
        request.state.pose.position.y = float(pose["y"])
        request.state.pose.position.z = float(pose["z"])
        yaw = float(pose["yaw"])
        request.state.pose.orientation.z = math.sin(0.5 * yaw)
        request.state.pose.orientation.w = math.cos(0.5 * yaw)
        request.state.twist = Twist()
        future = self._set_entity_client.call_async(request)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        ok = bool(future.done() and future.result() is not None and future.result().success)
        snap = self._object_state()
        self.get_logger().info(
            "RESET_SET_ENTITY target=%s step=%s entity=%s ok=%d v=%s w=%s"
            % (
                self.target,
                step,
                request.state.name,
                int(ok),
                self._fmt(snap[1]),
                self._fmt(snap[2]),
            )
        )

    def _spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _command_gripper_open(self) -> None:
        msg = Float64MultiArray()
        msg.data = list(GRIPPER_OPEN_POSITIONS)
        self._gripper_pub.publish(msg)
        self.get_logger().info("OPEN_COMMAND target=%s" % self.target)

    def _wait_gripper_open(self) -> bool:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            left, right = self._gripper_positions()
            if (
                left is not None
                and right is not None
                and abs(left) <= GRIPPER_OPEN_TOLERANCE
                and abs(right) <= GRIPPER_OPEN_TOLERANCE
                and not self._finger_contacts()[0]
                and not self._finger_contacts()[1]
            ):
                self.get_logger().info(
                    "GRIPPER_RESET_OK target=%s left=%.6f right=%.6f"
                    % (self.target, left, right)
                )
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        left, right = self._gripper_positions()
        self.get_logger().warn(
            "GRIPPER_RESET_TIMEOUT target=%s left=%s right=%s contacts=%s/%s"
            % (
                self.target,
                self._fmt(left),
                self._fmt(right),
                int(self._finger_contacts()[0]),
                int(self._finger_contacts()[1]),
            )
        )
        return False

    def _move_arm_home(self) -> bool:
        if not self._arm_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("ARM_RESET_SKIPPED action server unavailable")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(UR_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(HOME_CONFIG)
        point.time_from_start = Duration(sec=ARM_RESET_DURATION_SEC)
        goal.trajectory.points = [point]
        future = self._arm_client.send_goal_async(goal)
        send_deadline = time.monotonic() + 3.0
        while time.monotonic() < send_deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.result() is None or not future.result().accepted:
            self.get_logger().warn("ARM_RESET_GOAL_REJECTED_OR_TIMEOUT")
            return False
        result_future = future.result().get_result_async()
        deadline = time.monotonic() + ARM_RESET_TIMEOUT_SEC
        while time.monotonic() < deadline:
            arm_err = self._arm_max_abs_err()
            if arm_err <= ARM_RESET_JOINT_TOLERANCE_RAD:
                self.get_logger().info(
                    "ARM_RESET_OK target=%s arm_home_err=%.6f"
                    % (self.target, arm_err)
                )
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().warn(
            "ARM_RESET_TIMEOUT target=%s arm_home_err=%.6f"
            % (self.target, self._arm_max_abs_err())
        )
        return False

    def _wait_object_settled(self) -> float:
        start = time.monotonic()
        stable_count = 0
        last_pose = None
        expected = (get_target_config(self.target) or {}).get("world_pose", {})
        while time.monotonic() - start < self.reset_settle_timeout_sec:
            snap = self._object_state()
            pose = snap[0]
            v = snap[1]
            w = snap[2]
            elapsed = time.monotonic() - start
            pose_stable = True
            if last_pose is not None and pose is not None:
                pose_stable = (
                    abs(pose["x"] - last_pose["x"]) <= SETTLE_POSITION_TOLERANCE_M
                    and abs(pose["y"] - last_pose["y"]) <= SETTLE_POSITION_TOLERANCE_M
                    and abs(pose["z"] - last_pose["z"]) <= SETTLE_POSITION_TOLERANCE_M
                )
            pose_near_reset = True
            if pose is not None and expected:
                pose_near_reset = (
                    abs(pose["x"] - float(expected["x"])) <= RESET_POSE_TOLERANCE_M
                    and abs(pose["y"] - float(expected["y"])) <= RESET_POSE_TOLERANCE_M
                    and abs(pose["z"] - float(expected["z"])) <= RESET_POSE_TOLERANCE_M
                )
            if (
                pose is not None
                and elapsed >= MIN_SETTLE_OBSERVATION_SEC
                and pose_near_reset
                and pose_stable
                and v <= SETTLE_LINEAR_VELOCITY_MPS
                and w <= SETTLE_ANGULAR_VELOCITY_RPS
            ):
                stable_count += 1
                if stable_count >= 3:
                    self.get_logger().info(
                        "OBJECT_SETTLED target=%s settle_sec=%.3f v=%.6f w=%.6f"
                        % (self.target, elapsed, v, w)
                    )
                    return elapsed
            else:
                stable_count = 0
            last_pose = pose
            rclpy.spin_once(self, timeout_sec=0.1)
        elapsed = time.monotonic() - start
        self.get_logger().warn(
            "OBJECT_SETTLE_TIMEOUT target=%s settle_sec=%.3f" % (self.target, elapsed)
        )
        return elapsed

    def _on_status(self, msg: String) -> None:
        self._status_events.append((time.monotonic(), str(msg.data)))

    def _on_contact_status(self, msg: String) -> None:
        self._contact_status = str(msg.data)

    def _on_contact_fingers(self, msg: String) -> None:
        data = str(msg.data)
        self._last_fingers = data
        if "left=1" in data:
            self._contact_left_seen = True
        if "right=1" in data:
            self._contact_right_seen = True

    def _on_contact_force(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 2:
            left = max(0.0, float(msg.data[0]))
            right = max(0.0, float(msg.data[1]))
            if len(msg.data) >= 5:
                source_code = int(float(msg.data[2]))
                self._force_source = {
                    1: "RAW_GAZEBO_WRENCH",
                    2: "VIRTUAL_ESTIMATE",
                }.get(source_code, "INVALID")
                self._force_valid_left = float(msg.data[3]) >= 0.5
                self._force_valid_right = float(msg.data[4]) >= 0.5
                self._force_valid_left_seen = (
                    self._force_valid_left_seen or self._force_valid_left
                )
                self._force_valid_right_seen = (
                    self._force_valid_right_seen or self._force_valid_right
                )
            self._current_left_force = left
            self._current_right_force = right
            self._left_force = max(self._left_force, left)
            self._right_force = max(self._right_force, right)
            self._force_samples += 1
            if self.record_force_timeseries and self._active_trial_id > 0:
                gripper_left, gripper_right = self._gripper_positions()
                contact_left, contact_right = self._finger_contacts()
                elapsed = 0.0
                if self._active_trial_start is not None:
                    elapsed = max(0.0, time.monotonic() - self._active_trial_start)
                self._force_timeseries_rows.append(
                    {
                        "target": self.target,
                        "trial": self._active_trial_id,
                        "elapsed_sec": f"{elapsed:.6f}",
                        "sim_time": f"{self.get_clock().now().nanoseconds * 1.0e-9:.6f}",
                        "left_contact": int(contact_left),
                        "right_contact": int(contact_right),
                        "left_force_raw": f"{left:.6f}",
                        "right_force_raw": f"{right:.6f}",
                        "force_source": self._force_source,
                        "force_valid_left": int(self._force_valid_left),
                        "force_valid_right": int(self._force_valid_right),
                        "gripper_left": self._fmt(gripper_left),
                        "gripper_right": self._fmt(gripper_right),
                        "gripper_gap_mm": self._fmt(
                            self._gripper_gap_mm(gripper_left, gripper_right)
                        ),
                    }
                )

    def _on_contact_force_source(self, msg: String) -> None:
        parsed = {}
        for part in str(msg.data).split():
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            parsed[key] = value
        source = parsed.get("source", "INVALID")
        if source not in ("RAW_GAZEBO_WRENCH", "VIRTUAL_ESTIMATE", "INVALID"):
            source = "INVALID"
        self._force_source = source
        self._force_valid_left = parsed.get("left_valid", "0") == "1"
        self._force_valid_right = parsed.get("right_valid", "0") == "1"

    def _on_force_control_status(self, msg: String) -> None:
        if self._active_trial_id <= 0:
            return
        parsed = {}
        for part in str(msg.data).split():
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            parsed[key] = value
        elapsed = 0.0
        if self._active_trial_start is not None:
            elapsed = max(0.0, time.monotonic() - self._active_trial_start)
        self._force_control_rows.append(
            {
                "target": parsed.get("target", self.target),
                "trial": self._active_trial_id,
                "elapsed_sec": f"{elapsed:.6f}",
                "sim_time": f"{self.get_clock().now().nanoseconds * 1.0e-9:.6f}",
                "control_state": parsed.get("state", ""),
                "gripper_command_left": parsed.get("command_left", ""),
                "gripper_command_right": parsed.get("command_right", ""),
                "gripper_position_left": parsed.get("left_actual", ""),
                "gripper_position_right": parsed.get("right_actual", ""),
                "force_left_raw": parsed.get("force_left_raw", ""),
                "force_right_raw": parsed.get("force_right_raw", ""),
                "force_left_filtered": parsed.get("force_left_filtered", ""),
                "force_right_filtered": parsed.get("force_right_filtered", ""),
                "force_mean": parsed.get("force_mean", ""),
                "force_error": parsed.get("force_error", ""),
                "force_balance_error": parsed.get("force_balance_error", ""),
                "delta_q": parsed.get("delta_q", ""),
                "force_source": parsed.get("force_source", ""),
                "force_valid_left": parsed.get("left_valid", ""),
                "force_valid_right": parsed.get("right_valid", ""),
            }
        )

    def _force_control_final_state(self, trial_id: int) -> str:
        states = [
            row.get("control_state", "")
            for row in self._force_control_rows
            if int(row.get("trial", 0)) == int(trial_id)
        ]
        return states[-1] if states else ""

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = msg

    def _on_model_states(self, msg: ModelStates) -> None:
        self._latest_model_states = msg

    def _object_state(self) -> tuple[dict | None, float, float]:
        msg = self._latest_model_states
        if msg is None or self.target not in msg.name:
            return None, float("inf"), float("inf")
        index = msg.name.index(self.target)
        pose = msg.pose[index]
        twist = msg.twist[index]
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z),
        )
        linear = math.sqrt(
            twist.linear.x * twist.linear.x
            + twist.linear.y * twist.linear.y
            + twist.linear.z * twist.linear.z
        )
        angular = math.sqrt(
            twist.angular.x * twist.angular.x
            + twist.angular.y * twist.angular.y
            + twist.angular.z * twist.angular.z
        )
        return (
            {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "yaw": float(yaw),
            },
            float(linear),
            float(angular),
        )

    def _tcp_pose(self) -> dict | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                "gripper_tcp",
                rclpy.time.Time(),
                timeout=RclpyDuration(seconds=0.2),
            )
        except TransformException:
            return None
        t = transform.transform.translation
        return {"x": float(t.x), "y": float(t.y), "z": float(t.z)}

    def _gripper_positions(self) -> tuple[float | None, float | None]:
        msg = self._last_joint_state
        if msg is None:
            return None, None
        values = dict(zip(msg.name, msg.position))
        return (
            float(values["gripper_left_finger_joint"])
            if "gripper_left_finger_joint" in values
            else None,
            float(values["gripper_right_finger_joint"])
            if "gripper_right_finger_joint" in values
            else None,
        )

    def _arm_max_abs_err(self) -> float:
        msg = self._last_joint_state
        if msg is None:
            return float("nan")
        values = dict(zip(msg.name, msg.position))
        errors = [
            abs(float(values[name]) - float(expected))
            for name, expected in zip(UR_JOINTS, HOME_CONFIG)
            if name in values
        ]
        return max(errors) if errors else float("nan")

    def _finger_contacts(self) -> tuple[bool, bool]:
        return "left=1" in self._last_fingers, "right=1" in self._last_fingers

    def _snapshot(self, trial_id: int, phase: str) -> dict:
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.05)
        object_pose, object_v, object_w = self._object_state()
        tcp = self._tcp_pose()
        left, right = self._gripper_positions()
        contact_left, contact_right = self._finger_contacts()
        return {
            "trial": trial_id,
            "phase": phase,
            "object_pose": object_pose,
            "object_v": object_v,
            "object_w": object_w,
            "tcp": tcp,
            "arm_max_abs_err": self._arm_max_abs_err(),
            "gripper_left": left,
            "gripper_right": right,
            "gripper_gap_mm": self._gripper_gap_mm(left, right),
            "contact_left": contact_left,
            "contact_right": contact_right,
            "force_left": self._left_force,
            "force_right": self._right_force,
            "force_source": self._force_source,
            "force_valid_left": self._force_valid_left,
            "force_valid_right": self._force_valid_right,
            "attached": self._contact_status.startswith("attached "),
            "holding": self._contact_status.startswith("holding ")
            or self._contact_status.startswith("attached "),
            "sim_time": self.get_clock().now().nanoseconds * 1.0e-9,
        }

    def _log_snapshot(self, snapshot: dict) -> None:
        pose = snapshot["object_pose"] or {}
        tcp = snapshot["tcp"] or {}
        self.get_logger().info(
            "%s target=%s trial=%d object_pose=(%s,%s,%s yaw=%s) "
            "object_vel=(%.6f,%.6f) tcp=(%s,%s,%s) arm_home_err=%s "
            "gripper=(%s,%s gap=%s) contacts=%d/%d force=%.3f/%.3f "
            "attached=%s holding=%s sim_time=%.3f"
            % (
                snapshot["phase"],
                self.target,
                snapshot["trial"],
                self._fmt(pose.get("x")),
                self._fmt(pose.get("y")),
                self._fmt(pose.get("z")),
                self._fmt(pose.get("yaw")),
                snapshot["object_v"],
                snapshot["object_w"],
                self._fmt(tcp.get("x")),
                self._fmt(tcp.get("y")),
                self._fmt(tcp.get("z")),
                self._fmt(snapshot["arm_max_abs_err"]),
                self._fmt(snapshot["gripper_left"]),
                self._fmt(snapshot["gripper_right"]),
                self._fmt(snapshot["gripper_gap_mm"]),
                int(snapshot["contact_left"]),
                int(snapshot["contact_right"]),
                snapshot["force_left"],
                snapshot["force_right"],
                str(snapshot["attached"]).lower(),
                str(snapshot["holding"]).lower(),
                snapshot["sim_time"],
            )
        )

    def _flatten_snapshot(self, prefix: str, snapshot: dict) -> dict:
        pose = snapshot["object_pose"] or {}
        tcp = snapshot["tcp"] or {}
        return {
            f"{prefix}_object_x": self._csv_float(pose.get("x")),
            f"{prefix}_object_y": self._csv_float(pose.get("y")),
            f"{prefix}_object_z": self._csv_float(pose.get("z")),
            f"{prefix}_object_yaw": self._csv_float(pose.get("yaw")),
            f"{prefix}_object_v": self._csv_float(snapshot["object_v"]),
            f"{prefix}_object_w": self._csv_float(snapshot["object_w"]),
            f"{prefix}_tcp_x": self._csv_float(tcp.get("x")),
            f"{prefix}_tcp_y": self._csv_float(tcp.get("y")),
            f"{prefix}_tcp_z": self._csv_float(tcp.get("z")),
            f"{prefix}_arm_max_abs_err": self._csv_float(snapshot["arm_max_abs_err"]),
            f"{prefix}_gripper_left": self._csv_float(snapshot["gripper_left"]),
            f"{prefix}_gripper_right": self._csv_float(snapshot["gripper_right"]),
            f"{prefix}_gripper_gap_mm": self._csv_float(snapshot["gripper_gap_mm"]),
            f"{prefix}_contact_left": int(snapshot["contact_left"]),
            f"{prefix}_contact_right": int(snapshot["contact_right"]),
            f"{prefix}_force_left": self._csv_float(snapshot["force_left"]),
            f"{prefix}_force_right": self._csv_float(snapshot["force_right"]),
            f"{prefix}_force_source": snapshot["force_source"],
            f"{prefix}_force_valid_left": int(snapshot["force_valid_left"]),
            f"{prefix}_force_valid_right": int(snapshot["force_valid_right"]),
            f"{prefix}_attached": int(snapshot["attached"]),
            f"{prefix}_holding": int(snapshot["holding"]),
            f"{prefix}_sim_time": self._csv_float(snapshot["sim_time"]),
        }

    @staticmethod
    def _gripper_gap_mm(left: float | None, right: float | None) -> float:
        if left is None or right is None:
            return float("nan")
        return (0.092 - left - right) * 1000.0

    @staticmethod
    def _csv_float(value) -> str:
        if value is None:
            return ""
        try:
            value = float(value)
        except (TypeError, ValueError):
            return ""
        if not math.isfinite(value):
            return ""
        return f"{value:.6f}"

    @staticmethod
    def _fmt(value) -> str:
        text = GraspBenchmarkNode._csv_float(value)
        return text if text else "nan"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspBenchmarkNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
