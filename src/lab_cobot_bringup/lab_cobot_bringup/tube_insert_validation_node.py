#!/usr/bin/env python3
"""Independent test-tube insertion validation node."""
from __future__ import annotations

import copy
import math
import time
from threading import Lock, Thread

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import (
    GetCartesianPath,
    GetPositionFK,
    GetPositionIK,
    GetPlanningScene,
    GetStateValidity,
)
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lab_cobot_bringup.tube_insert_config import (
    BASE_LINK_WORLD_Z,
    GRIPPER_JOINT_MAX,
    GRIPPER_JOINT_MIN,
    GRIPPER_OPEN_INNER_GAP,
    RACK_INSERT_PRE_CLEARANCE,
    TEST_TUBE_DIAMETER,
    TEST_TUBE_GRASP_HEIGHT,
    TEST_TUBE_PRE_GRASP_CLEARANCE,
    TUBE_INSERT_CONFIG,
    TUBE_INSERT_VALIDATION_BASE_POSE,
    TUBE_POST_INSERT_RELEASE_GAP,
    TUBE_POST_INSERT_RELEASE_POSITION,
    TUBE_PRE_GRASP_POSITION,
    TUBE_PRE_OPEN_GAP,
    TUBE_TACTILE_MAX_POSITION,
    TUBE_TACTILE_START_POSITION,
    TUBE_TACTILE_STEP,
    TUBE_THEORETICAL_CONTACT_POSITION,
    tcp_pose_from_tube_bottom,
    world_to_base,
)
from lab_cobot_manipulation.pick_place_node import (
    DEFAULT_APPROACH_TOLERANCE_ORIENTATION,
    DEFAULT_APPROACH_TOLERANCE_POSITION,
    DEFAULT_GRASP_TOLERANCE_ORIENTATION,
    DEFAULT_GRASP_TOLERANCE_POSITION,
    DEFAULT_MOVE_TIMEOUT_SEC,
    GRIPPER_TCP_LINK,
    HOME_CONFIG,
    POST_GRASP_SETTLE_SEC,
    PRE_GRASP_SETTLE_SEC,
    PickPlace,
    UR_JOINTS,
    _ensure_trajectory_timing,
    _wait_for_moveit_result,
)
from lab_cobot_moveit.tube_insert_scene import (
    TEST_TUBE_ATTACHED_ID,
    TEST_TUBE_ATTACH_LINK,
    build_tube_insert_planning_scene,
    make_attach_test_tube_scene,
    make_detach_test_tube_scene,
)


CARTESIAN_MIN_FRACTION = 0.95
DEFAULT_CARTESIAN_MAX_STEP = 0.005
INSERT_FINAL_CARTESIAN_MAX_STEP = 0.002
INSERT_FINAL_CONTINUITY_SCAN_ZS = (
    0.6950,
    0.6945,
    0.6940,
    0.6935,
    0.6930,
    0.6925,
    0.6920,
    0.6915,
    0.6910,
    0.6905,
    0.6900,
)
STATION_B_TABLE_SURFACE_WORLD_Z = 0.75
TUBE_HIGH_APPROACH_BASE_Z = 0.86
TUBE_BASE_FRAME_X_CANDIDATES = (0.55, 0.575, 0.60, 0.625, 0.65, 0.675, 0.70)
TUBE_HIGH_APPROACH_Z_CANDIDATES = (0.82, 0.84, 0.86, 0.88, 0.90)
TUBE_READY_PRE_GRASP_TOLERANCE_POSITION = 0.025
TUBE_READY_PRE_GRASP_TOLERANCE_ORIENTATION = 0.25
ARM_CLEARANCE_SAMPLE_COUNT = 12
PROTECTED_ARM_LINK_GEOMETRY_HALF_Z = {
    "ur_upper_arm_link": 0.08,
    "ur_forearm_link": 0.07,
    "ur_wrist_1_link": 0.06,
    "ur_wrist_2_link": 0.06,
    "ur_wrist_3_link": 0.06,
}
PROTECTED_ARM_LINKS = tuple(PROTECTED_ARM_LINK_GEOMETRY_HALF_Z)
ARM_LINK_TABLE_SAFETY_MARGIN = {
    # The upper arm is mounted near the robot column and its conservative
    # Z-extent estimate can be below the tabletop plane while still being
    # collision-free.  MoveIt scene collision remains the primary safety check
    # for this mounting link; the Z guard only catches large downward excursions.
    "ur_upper_arm_link": -0.18,
    "ur_forearm_link": 0.04,
    "ur_wrist_1_link": 0.05,
    "ur_wrist_2_link": 0.05,
    "ur_wrist_3_link": 0.05,
}
ARM_HEIGHT_GUARD_LINKS = tuple(ARM_LINK_TABLE_SAFETY_MARGIN)
ARM_TABLE_CLEARANCE_NUMERIC_TOLERANCE = 0.002
INSERT_ACM_RACK2_OBJECT_PREFIX = "test_tube_rack_2_"
INSERT_ACM_STAGES = {"INSERT_STAGE1", "INSERT_STAGE2", "INSERT_FINAL"}
SHOULDER_PAN_JOINT = "ur_shoulder_pan_joint"
POST_INSERT_RETREAT_MIN_FRACTION = 0.99
OMPL_MULTI_CANDIDATE_ATTEMPTS = 6
OMPL_DIRECT_PATH_SAMPLE_COUNT = 12
TUBE_PRE_CLOSE_POSITION_TOLERANCE = 0.001
TUBE_PRE_CLOSE_TIMEOUT_SEC = 5.0
TUBE_PRE_CLOSE_CLOCK_STALL_SEC = 12.0
TUBE_PRE_CLOSE_WAIT_LOG_PERIOD_SEC = 1.0
TUBE_PRE_CLOSE_POLL_SEC = 0.05
OMPL_JOINT_COST_WEIGHTS = {
    "ur_shoulder_pan_joint": 1.0,
    "ur_shoulder_lift_joint": 1.0,
    "ur_elbow_joint": 1.0,
    "ur_wrist_1_joint": 1.0,
    "ur_wrist_2_joint": 1.0,
    "ur_wrist_3_joint": 1.0,
}


class TubeInsertValidation(PickPlace):
    """Run a fixed-coordinate test_tube_1 -> rack_2 middle-slot insertion."""

    def __init__(self) -> None:
        super().__init__(
            target_object=TUBE_INSERT_CONFIG["test_tube_1"]["entity_name"],
            use_tactile_grasp=True,
            use_planning_scene_obstacles=True,
            tactile_start_position=TUBE_TACTILE_START_POSITION,
            tactile_step_position=TUBE_TACTILE_STEP,
            tactile_max_position=TUBE_TACTILE_MAX_POSITION,
            tactile_log_prefix="TUBE_TACTILE",
            expected_object_width_mm=TEST_TUBE_DIAMETER * 1000.0,
            node_name="tube_insert_validation_node",
        )
        self._busy_lock = Lock()
        self._status_pub = self.create_publisher(
            String,
            TUBE_INSERT_CONFIG["status_topic"],
            10,
        )
        self._target_sub = self.create_subscription(
            String,
            TUBE_INSERT_CONFIG["target_topic"],
            self._on_target,
            10,
        )
        self._cartesian_path_client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
        )
        self._state_validity_client = self.create_client(
            GetStateValidity,
            "/check_state_validity",
        )
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self._planning_scene_client = self.create_client(
            GetPlanningScene,
            "/get_planning_scene",
        )
        self._active_stage_label = "UNKNOWN"
        self._arm_table_clearance_limits = {}
        self._insert_acm_active = False
        self._saved_allowed_collision_matrix = None
        self.declare_parameter("auto_prepare_tube_arm", True)
        self.auto_prepare_tube_arm = bool(
            self.get_parameter("auto_prepare_tube_arm").value
        )
        if self.auto_prepare_tube_arm:
            self._initial_pose_timer = self.create_timer(
                1.0,
                self._start_initial_pose_thread,
            )
        else:
            self._initial_pose_timer = self.create_timer(
                1.0,
                self._start_scene_only_thread,
            )
        self.get_logger().info(
            "tube insert validation ready on %s"
            % TUBE_INSERT_CONFIG["target_topic"]
        )

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = str(text)
        self._status_pub.publish(msg)
        self.get_logger().info(str(text))

    def _on_target(self, msg: String) -> None:
        command = str(msg.data).strip()
        if command != TUBE_INSERT_CONFIG["command"]:
            self._publish_status("IGNORED_UNKNOWN_COMMAND %s" % command)
            return
        if not self.auto_prepare_tube_arm:
            self._publish_status("IGNORED_FEASIBILITY_MODE_NO_ARM_MOTION")
            return
        if not self._busy_lock.acquire(blocking=False):
            self._publish_status("BUSY")
            return
        Thread(target=self._run_sequence, daemon=True).start()

    def _start_initial_pose_thread(self) -> None:
        self._initial_pose_timer.cancel()
        if not self._busy_lock.acquire(blocking=False):
            return
        Thread(target=self._prepare_initial_pose, daemon=True).start()

    def _start_scene_only_thread(self) -> None:
        self._initial_pose_timer.cancel()
        Thread(target=self._prepare_scene_only, daemon=True).start()

    def _prepare_scene_only(self) -> None:
        self._publish_status("TUBE_FEASIBILITY_MODE_WAITING")
        self._log_initial_validation_state("INITIAL")
        if not self._apply_tube_insert_scene():
            self._publish_status("FAILED_TUBE_SCENE")
            return
        self._publish_status("TUBE_SCENE_READY_NO_ARM_MOTION")

    def _prepare_initial_pose(self) -> None:
        try:
            tube = TUBE_INSERT_CONFIG["test_tube_1"]
            self._publish_status("PREPARING_ARM")
            if not self.gripper.open():
                self._publish_status("FAILED_INITIAL_GRIPPER_OPEN")
                return
            self._log_initial_validation_state("INITIAL")
            if not self._apply_tube_insert_scene():
                self._publish_status("FAILED_TUBE_SCENE")
                return
            if not self._initialize_arm_clearance_baseline():
                self._publish_status("FAILED_ARM_CLEARANCE")
                return
            if not self.go_home():
                self._publish_status("FAILED_INITIAL_HOME")
                return
            self._log_initial_validation_state("POST_INITIAL_HOME")
            self._log_tube_geometry()
            self._log_candidate_base_pose_ik_search()
            high_approach = self._high_approach_target(tube["pre_grasp_tcp"])
            self._log_ik_validity("HIGH_APPROACH", high_approach)
            self._log_ik_validity("PRE_GRASP", tube["pre_grasp_tcp"])
            self._log_ik_validity(
                "PRE_INSERT",
                TUBE_INSERT_CONFIG["test_tube_rack_2"]["pre_insert_tcp"],
            )
            if not self._move_ompl("HIGH_APPROACH", high_approach):
                self._publish_status("FAILED_INITIAL_HIGH_APPROACH")
                return
            if not self._move_ompl("READY_PRE_GRASP", tube["pre_grasp_tcp"]):
                self._publish_status("FAILED_INITIAL_PRE_GRASP")
                return
            self._publish_status("READY_PRE_GRASP")
        except Exception as exc:  # noqa: BLE001
            self._publish_status("FAILED_INITIAL_EXCEPTION %s" % exc)
        finally:
            self._release_busy()

    def _release_busy(self) -> None:
        try:
            self._busy_lock.release()
        except RuntimeError:
            pass

    def _apply_tube_insert_scene(self) -> bool:
        pose = TUBE_INSERT_VALIDATION_BASE_POSE
        scene = build_tube_insert_planning_scene(
            pose["x"],
            pose["y"],
            pose["yaw"],
        )
        expected_ids = self._tube_scene_expected_ids(scene)
        missing = self._tube_scene_missing_object_ids(expected_ids)
        if missing is None:
            self.get_logger().warn("TUBE_SCENE_QUERY_FAILED before apply")
        elif not missing:
            self.get_logger().info(
                "TUBE_SCENE_ALREADY_READY objects=%s"
                % ",".join(sorted(expected_ids))
            )
            return True
        else:
            self.get_logger().warn(
                "TUBE_SCENE_MISSING_OBJECTS ids=%s" % ",".join(sorted(missing))
            )

        self.get_logger().info(
            "TUBE_SCENE_APPLY_START objects=%s" % ",".join(sorted(expected_ids))
        )
        ok = self._apply_scene_diff(scene, "tube insert scene")
        if ok:
            missing = self._tube_scene_missing_object_ids(expected_ids)
            if missing is None:
                self.get_logger().warn("TUBE_SCENE_QUERY_FAILED after apply success")
                return False
            if not missing:
                object_ids = ",".join(obj.id for obj in scene.world.collision_objects)
                self.get_logger().info("TUBE_SCENE_APPLY_SUCCESS")
                self.get_logger().info(
                    "TUBE_INSERT_SCENE_APPLIED objects=%s" % object_ids
                )
                return True
            self.get_logger().warn(
                "TUBE_SCENE_MISSING_OBJECTS ids=%s" % ",".join(sorted(missing))
            )
            return False

        self.get_logger().warn("TUBE_SCENE_APPLY_TIMEOUT_RECHECK")
        missing = self._tube_scene_missing_object_ids(expected_ids)
        if missing is None:
            self.get_logger().warn("TUBE_SCENE_QUERY_FAILED after apply timeout")
            return False
        if not missing:
            self.get_logger().info(
                "TUBE_SCENE_APPLY_RESPONSE_TIMEOUT_BUT_READY objects=%s"
                % ",".join(sorted(expected_ids))
            )
            self.get_logger().info("TUBE_SCENE_READY_AFTER_RECHECK")
            return True
        self.get_logger().warn(
            "TUBE_SCENE_MISSING_OBJECTS ids=%s" % ",".join(sorted(missing))
        )
        return False

    def _tube_scene_expected_ids(self, scene: PlanningScene) -> set[str]:
        return {obj.id for obj in scene.world.collision_objects if obj.id}

    def _tube_scene_missing_object_ids(self, expected_ids: set[str]) -> set[str] | None:
        scene = self._get_current_planning_scene(
            PlanningSceneComponents.WORLD_OBJECT_NAMES,
            timeout_sec=1.0,
        )
        if scene is None:
            return None
        actual_ids = {obj.id for obj in scene.world.collision_objects if obj.id}
        return set(expected_ids) - actual_ids

    def _attach_tube_collision_object(self) -> bool:
        before = self._tube_scene_presence()
        self._log_tube_scene_presence("TUBE_SCENE_BEFORE_ATTACH", before)
        world_present, attached_present = before
        if attached_present and not world_present:
            self.get_logger().info("planning scene test_tube_1 already attached")
            self._log_tube_scene_presence(
                "TUBE_SCENE_AFTER_ATTACH",
                self._tube_scene_presence(),
            )
            return True
        if self._try_attach_tube_scene(remove_world_object=world_present):
            return True
        if world_present:
            self.get_logger().warn(
                "planning scene tube attach with world REMOVE failed; "
                "retrying attach without world REMOVE because MoveIt reported "
                "test_tube_1 is absent from its apply-scene target"
            )
            if self._try_attach_tube_scene(remove_world_object=False):
                return True
        return False

    def _try_attach_tube_scene(self, remove_world_object: bool) -> bool:
        scene = make_attach_test_tube_scene(
            remove_world_object=bool(remove_world_object)
        )
        label = (
            "tube attach remove-world"
            if remove_world_object
            else "tube attach no-world-remove"
        )
        if not self._apply_scene_diff(scene, label):
            return False
        after = self._tube_scene_presence()
        self._log_tube_scene_presence("TUBE_SCENE_AFTER_ATTACH", after)
        if after == (False, True):
            self.get_logger().info(
                "planning scene test_tube_1 attached remove_world_object=%s"
                % ("true" if remove_world_object else "false")
            )
            return True
        # Some MoveIt scene monitors report stale world names for one cycle after
        # a successful attach.  Treat attached=true as usable, but report the
        # stale world state explicitly so it can be verified in the next log.
        if after[1]:
            self.get_logger().warn(
                "planning scene test_tube_1 attached with stale world presence "
                "world_present=%s attached_present=true remove_world_object=%s"
                % (
                    "true" if after[0] else "false",
                    "true" if remove_world_object else "false",
                )
            )
            return True
        self.get_logger().warn(
            "planning scene test_tube_1 attach state unexpected "
            "world_present=%s attached_present=%s remove_world_object=%s"
            % (
                "true" if after[0] else "false",
                "true" if after[1] else "false",
                "true" if remove_world_object else "false",
            )
        )
        return False

    def _detach_tube_collision_object(self) -> None:
        world_present, attached_present = self._tube_scene_presence()
        if not attached_present:
            self.get_logger().info(
                "planning scene test_tube_1 already detached "
                "world_present=%s attached_present=false" % world_present
            )
            return
        scene = make_detach_test_tube_scene(remove_world_object=True)
        if self._apply_scene_diff(scene, "tube detach"):
            self.get_logger().info("planning scene test_tube_1 detached")

    def _tube_scene_presence(self) -> tuple[bool, bool]:
        scene = self._get_current_planning_scene(
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS,
            timeout_sec=1.0,
        )
        if scene is None:
            return (False, False)
        world_present = any(
            obj.id == TEST_TUBE_ATTACHED_ID
            for obj in scene.world.collision_objects
        )
        attached_present = any(
            attached.object.id == TEST_TUBE_ATTACHED_ID
            for attached in scene.robot_state.attached_collision_objects
        )
        return (world_present, attached_present)

    def _log_tube_scene_presence(
        self,
        label: str,
        presence: tuple[bool, bool],
    ) -> None:
        world_present, attached_present = presence
        self.get_logger().info(
            "%s world_present=%s attached_present=%s"
            % (
                label,
                "true" if world_present else "false",
                "true" if attached_present else "false",
            )
        )

    def _get_current_planning_scene(
        self,
        components: int,
        timeout_sec: float = 1.0,
    ) -> PlanningScene | None:
        if not self._planning_scene_client.wait_for_service(timeout_sec=0.3):
            self.get_logger().warn(
                "TUBE_PLANNING_SCENE_QUERY unavailable: /get_planning_scene not ready"
            )
            return None
        request = GetPlanningScene.Request()
        request.components.components = int(components)
        response = self._wait_for_cartesian_response(
            self._planning_scene_client.call_async(request),
            timeout_sec,
        )
        if response is None:
            self.get_logger().warn("TUBE_PLANNING_SCENE_QUERY unavailable: no response")
            return None
        return response.scene

    def _enable_insert_only_acm(self) -> bool:
        scene = self._get_current_planning_scene(
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS,
            timeout_sec=1.0,
        )
        if scene is None:
            return False
        self._saved_allowed_collision_matrix = copy.deepcopy(
            scene.allowed_collision_matrix
        )
        matrix = copy.deepcopy(scene.allowed_collision_matrix)
        rack2_names = sorted(
            obj.id
            for obj in scene.world.collision_objects
            if obj.id.startswith(INSERT_ACM_RACK2_OBJECT_PREFIX)
        )
        if not rack2_names:
            self.get_logger().warn("INSERT_ONLY_ACM unavailable: no rack2 objects")
            return False
        for rack_name in rack2_names:
            self._set_allowed_collision_pair(
                matrix,
                TEST_TUBE_ATTACHED_ID,
                rack_name,
                True,
            )
        diff = PlanningScene()
        diff.is_diff = True
        diff.allowed_collision_matrix = matrix
        if self._apply_scene_diff(diff, "tube insert-only ACM"):
            self._insert_acm_active = True
            self.get_logger().warn(
                "INSERT_ONLY_ACM_ENABLED attached=%s allowed_rack2_objects=%s"
                % (TEST_TUBE_ATTACHED_ID, ",".join(rack2_names))
            )
            return True
        return False

    def _restore_insert_only_acm(self) -> None:
        if not self._insert_acm_active:
            return
        if self._saved_allowed_collision_matrix is None:
            self._insert_acm_active = False
            return
        diff = PlanningScene()
        diff.is_diff = True
        diff.allowed_collision_matrix = self._saved_allowed_collision_matrix
        if self._apply_scene_diff(diff, "restore tube insert-only ACM"):
            self.get_logger().info("INSERT_ONLY_ACM_RESTORED")
        self._insert_acm_active = False
        self._saved_allowed_collision_matrix = None

    @staticmethod
    def _set_allowed_collision_pair(
        matrix: AllowedCollisionMatrix,
        name_a: str,
        name_b: str,
        allowed: bool,
    ) -> None:
        while len(matrix.entry_values) < len(matrix.entry_names):
            entry = AllowedCollisionEntry()
            entry.enabled = [False] * len(matrix.entry_names)
            matrix.entry_values.append(entry)
        for entry in matrix.entry_values:
            while len(entry.enabled) < len(matrix.entry_names):
                entry.enabled.append(False)
        for name in (name_a, name_b):
            if name in matrix.entry_names:
                continue
            matrix.entry_names.append(name)
            for entry in matrix.entry_values:
                entry.enabled.append(False)
            new_entry = AllowedCollisionEntry()
            new_entry.enabled = [False] * len(matrix.entry_names)
            matrix.entry_values.append(new_entry)
        index_a = matrix.entry_names.index(name_a)
        index_b = matrix.entry_names.index(name_b)
        matrix.entry_values[index_a].enabled[index_b] = bool(allowed)
        matrix.entry_values[index_b].enabled[index_a] = bool(allowed)

    def _abort_carried_tube(self, status: str) -> None:
        self._stop_hold_monitor()
        self._restore_insert_only_acm()
        self._detach_tube_collision_object()
        self.gripper.release_object()
        self.gripper.open()
        self._publish_status(status)

    def _handle_hold_lost(self) -> None:
        if not getattr(self, "_hold_monitor_fault", False):
            return
        self._stop_hold_monitor()
        self._detach_tube_collision_object()
        self.get_logger().error("tube insert failed: carried tube is no longer held")

    def _fixed_quat(self) -> list[float]:
        return list(TUBE_INSERT_CONFIG["fixed_tcp_quat_xyzw"])

    def _fixed_rpy(self) -> tuple[float, float, float]:
        return (3.141592653589793, 0.0, 0.0)

    def _high_approach_target(self, target: list[float]) -> list[float]:
        return [
            float(target[0]),
            float(target[1]),
            TUBE_HIGH_APPROACH_BASE_Z,
        ]

    def _log_tube_geometry(self) -> None:
        tube = TUBE_INSERT_CONFIG["test_tube_1"]
        rack = TUBE_INSERT_CONFIG["test_tube_rack_2"]
        pose = TUBE_INSERT_VALIDATION_BASE_POSE
        self.get_logger().info(
            "SELECTED_TUBE_BASE_WORLD_POSE x=%.4f y=%.4f yaw=%.6f"
            % (pose["x"], pose["y"], pose["yaw"])
        )
        self.get_logger().info(
            "TUBE_TABLE_SURFACE_WORLD_Z z=%.4f" % STATION_B_TABLE_SURFACE_WORLD_Z
        )
        self.get_logger().info(
            "TUBE_ARM_LINK_CLEARANCE mode=table_relative geometry=min_estimate"
        )
        self._log_point("TUBE_PICK_WORLD_POINT", tube["world_pose"])
        self._log_point("TUBE_PICK_BASE_POINT", tube["bottom_base"])
        self._log_point("SELECTED_PICK_BASE_POINT", tube["bottom_base"])
        self._log_point("TUBE_INSERT_WORLD_POINT", rack["slot_mouth_world"])
        self._log_point("TUBE_INSERT_BASE_POINT", rack["slot_mouth_base"])
        self._log_point("SELECTED_INSERT_BASE_POINT", rack["slot_mouth_base"])
        self._log_xyz("TUBE_PRE_GRASP_TARGET", tube["pre_grasp_tcp"])
        self._log_xyz("TUBE_PRE_INSERT_TARGET", rack["pre_insert_tcp"])
        self._log_xyz("TUBE_HIGH_APPROACH_TARGET", self._high_approach_target(tube["pre_grasp_tcp"]))
        roll, pitch, yaw = self._fixed_rpy()
        self.get_logger().info(
            "TUBE_GRASP_RPY roll=%.6f pitch=%.6f yaw=%.6f"
            % (roll, pitch, yaw)
        )
        quat = self._fixed_quat()
        self.get_logger().info(
            "TUBE_GRASP_QUAT x=%.6f y=%.6f z=%.6f w=%.6f"
            % (quat[0], quat[1], quat[2], quat[3])
        )
        self._log_tube_gripper_geometry()
        self._log_tube_pre_close_config()
        self._log_current_arm_link_heights()

    def _log_tube_gripper_geometry(self) -> None:
        max_gap = GRIPPER_OPEN_INNER_GAP - 2.0 * GRIPPER_JOINT_MIN
        min_gap = GRIPPER_OPEN_INNER_GAP - 2.0 * GRIPPER_JOINT_MAX
        self.get_logger().info(
            "TUBE_GRIPPER_GEOMETRY tube_diameter_mm=%.2f joint_min=%.6f "
            "joint_max=%.6f max_gap_mm=%.2f min_gap_mm=%.2f"
            % (
                TEST_TUBE_DIAMETER * 1000.0,
                GRIPPER_JOINT_MIN,
                GRIPPER_JOINT_MAX,
                max_gap * 1000.0,
                min_gap * 1000.0,
            )
        )

    def _log_tube_pre_close_config(self) -> None:
        single_side_clearance = (
            TUBE_PRE_OPEN_GAP - TEST_TUBE_DIAMETER
        ) * 0.5
        self.get_logger().info(
            "TUBE_PRE_CLOSE_CONFIG tube_diameter_mm=%.2f target_gap_mm=%.2f "
            "command=%.6f single_side_clearance_mm=%.2f "
            "theoretical_contact_q=%.6f"
            % (
                TEST_TUBE_DIAMETER * 1000.0,
                TUBE_PRE_OPEN_GAP * 1000.0,
                TUBE_PRE_GRASP_POSITION,
                single_side_clearance * 1000.0,
                TUBE_THEORETICAL_CONTACT_POSITION,
            )
        )
        self.get_logger().info(
            "TUBE_TACTILE_CONFIG start=%.6f step=%.6f max=%.6f urdf_upper=%.6f"
            % (
                TUBE_TACTILE_START_POSITION,
                TUBE_TACTILE_STEP,
                TUBE_TACTILE_MAX_POSITION,
                GRIPPER_JOINT_MAX,
            )
        )

    def _initialize_arm_clearance_baseline(self) -> bool:
        joint_state = getattr(self.moveit2, "joint_state", None)
        if joint_state is None:
            self.get_logger().error(
                "ARM_CLEARANCE_BASELINE unavailable: no joint_state"
            )
            return False
        heights = self._arm_link_heights_for_joint_state(joint_state)
        if heights is None:
            self.get_logger().error(
                "ARM_CLEARANCE_BASELINE unavailable: no arm link heights"
            )
            return False

        by_link = {
            link: (origin_z, min_geometry_z)
            for link, origin_z, min_geometry_z in heights
        }
        limits = {}
        for link in PROTECTED_ARM_LINKS:
            origin_z, min_geometry_z = by_link[link]
            self.get_logger().info(
                "HOME_ARM_LINK_HEIGHT link=%s origin_z=%.4f min_geometry_z=%.4f"
                % (link, origin_z, min_geometry_z)
            )
            if link not in ARM_HEIGHT_GUARD_LINKS:
                continue
            margin = float(ARM_LINK_TABLE_SAFETY_MARGIN[link])
            limit_z = STATION_B_TABLE_SURFACE_WORLD_Z + margin
            clearance = float(min_geometry_z) - STATION_B_TABLE_SURFACE_WORLD_Z
            limits[link] = {
                "home_min_z": float(min_geometry_z),
                "required_margin": margin,
                "limit_z": limit_z,
            }
            self.get_logger().info(
                "ARM_TABLE_CLEARANCE_LIMIT link=%s home_min_z=%.4f "
                "table_surface_z=%.4f home_clearance=%.4f "
                "required_margin=%.4f limit_z=%.4f"
                % (
                    link,
                    min_geometry_z,
                    STATION_B_TABLE_SURFACE_WORLD_Z,
                    clearance,
                    margin,
                    limit_z,
                )
            )

        self._arm_table_clearance_limits = limits
        return bool(limits)

    def _log_point(self, label: str, point: dict) -> None:
        self.get_logger().info(
            "%s x=%.4f y=%.4f z=%.4f"
            % (label, point["x"], point["y"], point["z"])
        )

    def _log_xyz(self, label: str, point: list[float]) -> None:
        self.get_logger().info(
            "%s x=%.4f y=%.4f z=%.4f"
            % (label, point[0], point[1], point[2])
        )

    def _log_candidate_base_pose_ik_search(self) -> None:
        tube_world = TUBE_INSERT_CONFIG["test_tube_1"]["world_pose"]
        rack_world = TUBE_INSERT_CONFIG["test_tube_rack_2"]["slot_mouth_world"]
        selected_pose = TUBE_INSERT_VALIDATION_BASE_POSE
        for base_x in TUBE_BASE_FRAME_X_CANDIDATES:
            candidate_pose = {
                "x": float(selected_pose["x"]),
                "y": float(tube_world["y"]) - float(base_x),
                "yaw": float(selected_pose["yaw"]),
            }
            pick_base = world_to_base(tube_world, candidate_pose)
            insert_base = world_to_base(rack_world, candidate_pose)
            pick_tcp = tcp_pose_from_tube_bottom(
                pick_base,
                TEST_TUBE_GRASP_HEIGHT,
            )
            pre_grasp = [
                pick_tcp[0],
                pick_tcp[1],
                pick_tcp[2] + TEST_TUBE_PRE_GRASP_CLEARANCE,
            ]
            pre_insert = self._pre_insert_target_for_base_point(insert_base)
            self.get_logger().info(
                "TUBE_BASE_POSE_CANDIDATE base_frame_x=%.3f "
                "base_world_x=%.4f base_world_y=%.4f yaw=%.6f "
                "pick_base_x=%.4f pick_base_y=%.4f "
                "insert_base_x=%.4f insert_base_y=%.4f"
                % (
                    base_x,
                    candidate_pose["x"],
                    candidate_pose["y"],
                    candidate_pose["yaw"],
                    pick_base["x"],
                    pick_base["y"],
                    insert_base["x"],
                    insert_base["y"],
                )
            )
            for high_z in TUBE_HIGH_APPROACH_Z_CANDIDATES:
                high_approach = [pre_grasp[0], pre_grasp[1], high_z]
                self._log_ik_validity(
                    "CANDIDATE_X%.3f_HIGH_Z%.3f" % (base_x, high_z),
                    high_approach,
                    timeout_sec=0.75,
                )
            self._log_ik_validity(
                "CANDIDATE_X%.3f_PRE_GRASP" % base_x,
                pre_grasp,
                timeout_sec=0.75,
            )
            self._log_ik_validity(
                "CANDIDATE_X%.3f_PRE_INSERT" % base_x,
                pre_insert,
                timeout_sec=0.75,
            )

    def _pre_insert_target_for_base_point(self, slot_mouth_base: dict) -> list[float]:
        bottom = {
            "x": slot_mouth_base["x"],
            "y": slot_mouth_base["y"],
            "z": slot_mouth_base["z"] + RACK_INSERT_PRE_CLEARANCE,
        }
        return tcp_pose_from_tube_bottom(bottom, TEST_TUBE_GRASP_HEIGHT)

    def _log_ik_validity(
        self,
        label: str,
        target: list[float],
        timeout_sec: float = 1.5,
    ) -> bool:
        if not self._ik_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("%s_IK_VALID unavailable: /compute_ik not ready" % label)
            return False
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator"
        request.ik_request.ik_link_name = GRIPPER_TCP_LINK
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.pose = self._target_pose(target)
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = int(timeout_sec)
        request.ik_request.timeout.nanosec = int(
            (float(timeout_sec) - int(timeout_sec)) * 1_000_000_000
        )
        joint_state = getattr(self.moveit2, "joint_state", None)
        if joint_state is not None:
            request.ik_request.robot_state = RobotState()
            request.ik_request.robot_state.joint_state = joint_state
        response = self._wait_for_cartesian_response(
            self._ik_client.call_async(request),
            float(timeout_sec) + 0.3,
        )
        if response is None:
            self.get_logger().warn("%s_IK_VALID unavailable: no response" % label)
            return False
        code = int(response.error_code.val)
        valid = code == 1
        self.get_logger().info(
            "%s_IK_VALID valid=%s error_code=%d"
            % (label, "true" if valid else "false", code)
        )
        if valid:
            self._log_ik_solution(label, response.solution.joint_state)
            self._log_solution_state_validity(label, response.solution.joint_state)
            self._log_arm_clearance_min(label, response.solution.joint_state)
        return valid

    def _log_ik_solution(self, label: str, joint_state) -> None:
        by_name = {
            name: float(position)
            for name, position in zip(joint_state.name, joint_state.position)
        }
        missing = [name for name in UR_JOINTS if name not in by_name]
        if missing:
            self.get_logger().warn(
                "%s_IK_JOINT_SOLUTION unavailable: missing %s"
                % (label, ",".join(missing))
            )
            return
        self.get_logger().info(
            "%s_IK_JOINT_SOLUTION %s"
            % (
                label,
                " ".join("%s=%.5f" % (name, by_name[name]) for name in UR_JOINTS),
            )
        )

    def _log_solution_state_validity(self, label: str, joint_state) -> None:
        if not self._state_validity_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(
                "%s_STATE_VALID unavailable: /check_state_validity not ready"
                % label
            )
            return
        request = GetStateValidity.Request()
        request.group_name = "ur_manipulator"
        request.robot_state = RobotState()
        request.robot_state.joint_state = joint_state
        response = self._wait_for_cartesian_response(
            self._state_validity_client.call_async(request),
            1.0,
        )
        if response is None:
            self.get_logger().warn("%s_STATE_VALID unavailable: no response" % label)
            return
        contacts = list(getattr(response, "contacts", []))
        self.get_logger().info(
            "%s_STATE_VALID valid=%s contacts=%d"
            % (label, "true" if response.valid else "false", len(contacts))
        )
        if contacts:
            contact = contacts[0]
            self.get_logger().warn(
                "%s_FIRST_INVALID_COLLISION_PAIR %s <-> %s depth=%.6f"
                % (
                    label,
                    contact.contact_body_1,
                    contact.contact_body_2,
                    float(contact.depth),
                )
            )

    def _log_arm_clearance_min(self, label: str, joint_state) -> None:
        heights = self._arm_link_heights_for_joint_state(joint_state)
        if heights is None:
            self.get_logger().warn("%s_ARM_CLEARANCE_MIN unavailable" % label)
            return
        for link, _origin_z, min_geometry_z in heights:
            limit = self._arm_table_clearance_limits.get(link)
            if limit is None:
                continue
            clearance = min_geometry_z - STATION_B_TABLE_SURFACE_WORLD_Z
            valid = self._arm_table_clearance_valid(clearance, limit)
            self.get_logger().info(
                "ARM_TABLE_CLEARANCE stage=%s link=%s link_min_z=%.4f "
                "table_surface_z=%.4f clearance=%.4f required_margin=%.4f "
                "tolerance=%.4f valid=%s"
                % (
                    label,
                    link,
                    min_geometry_z,
                    STATION_B_TABLE_SURFACE_WORLD_Z,
                    clearance,
                    limit["required_margin"],
                    ARM_TABLE_CLEARANCE_NUMERIC_TOLERANCE,
                    "true" if valid else "false",
                )
            )

    def _log_current_arm_link_heights(self) -> None:
        joint_state = getattr(self.moveit2, "joint_state", None)
        if joint_state is None:
            self.get_logger().warn("ARM_LINK_HEIGHT unavailable: no joint_state")
            return
        heights = self._arm_link_heights_for_joint_state(joint_state)
        if heights is None:
            return
        for link, origin_z, min_geometry_z in heights:
            self.get_logger().info(
                "ARM_LINK_HEIGHT link=%s origin_z=%.4f min_geometry_z=%.4f"
                % (link, origin_z, min_geometry_z)
            )

    def _log_initial_validation_state(self, label: str) -> None:
        pose = TUBE_INSERT_VALIDATION_BASE_POSE
        self.get_logger().info(
            "%s_TUBE_BASE_WORLD_POSE x=%.4f y=%.4f yaw=%.6f"
            % (label, pose["x"], pose["y"], pose["yaw"])
        )
        joint_state = getattr(self.moveit2, "joint_state", None)
        self.get_logger().info(
            "%s_ARM_JOINTS %s" % (label, self._joint_positions_text(joint_state))
        )
        self._log_current_tcp_pose(label)
        self._log_initial_state_validity(label, joint_state)

    def _joint_positions_text(self, joint_state) -> str:
        if joint_state is None:
            return "unavailable"
        by_name = {
            name: float(position)
            for name, position in zip(joint_state.name, joint_state.position)
        }
        names = [
            "ur_shoulder_pan_joint",
            "ur_shoulder_lift_joint",
            "ur_elbow_joint",
            "ur_wrist_1_joint",
            "ur_wrist_2_joint",
            "ur_wrist_3_joint",
        ]
        missing = [name for name in names if name not in by_name]
        if missing:
            return "missing %s" % ",".join(missing)
        return " ".join("%s=%.5f" % (name, by_name[name]) for name in names)

    def _log_current_tcp_pose(self, label: str) -> None:
        if not hasattr(self.moveit2, "compute_fk"):
            self.get_logger().warn("%s_TCP_POSE unavailable: no compute_fk" % label)
            return
        try:
            pose_stamped = self.moveit2.compute_fk(fk_link_names=[GRIPPER_TCP_LINK])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("%s_TCP_POSE unavailable: %s" % (label, exc))
            return
        if isinstance(pose_stamped, list):
            pose_stamped = pose_stamped[0] if pose_stamped else None
        if pose_stamped is None:
            self.get_logger().warn("%s_TCP_POSE unavailable: empty FK" % label)
            return
        pose = pose_stamped.pose
        self.get_logger().info(
            "%s_TCP_POSE frame=%s x=%.4f y=%.4f z=%.4f "
            "qx=%.6f qy=%.6f qz=%.6f qw=%.6f"
            % (
                label,
                pose_stamped.header.frame_id,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )

    def _log_initial_state_validity(self, label: str, joint_state) -> None:
        if joint_state is None:
            self.get_logger().warn("%s_STATE_VALID unavailable: no joint_state" % label)
            return
        if not self._state_validity_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(
                "%s_STATE_VALID unavailable: /check_state_validity not ready"
                % label
            )
            return
        request = GetStateValidity.Request()
        request.group_name = "ur_manipulator"
        request.robot_state = RobotState()
        request.robot_state.joint_state = joint_state
        future = self._state_validity_client.call_async(request)
        response = self._wait_for_cartesian_response(future, 1.0)
        if response is None:
            self.get_logger().warn("%s_STATE_VALID unavailable: no response" % label)
            return
        contacts = list(getattr(response, "contacts", []))
        self.get_logger().info(
            "%s_STATE_VALID valid=%s contacts=%d"
            % (label, "true" if response.valid else "false", len(contacts))
        )
        if contacts:
            contact = contacts[0]
            self.get_logger().warn(
                "%s_FIRST_INVALID_COLLISION_PAIR %s <-> %s depth=%.6f"
                % (
                    label,
                    contact.contact_body_1,
                    contact.contact_body_2,
                    float(contact.depth),
                )
            )

    def _move_ompl(self, label: str, target: list[float]) -> bool:
        self._publish_status(label)
        self._active_stage_label = label
        try:
            return self._move_pose_with_joint_optimization(
                label,
                target,
                timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
            )
        finally:
            self._active_stage_label = "UNKNOWN"

    def _current_joint_state(self) -> JointState | None:
        joint_state = getattr(self.moveit2, "joint_state", None)
        if joint_state is None:
            return None
        return copy.deepcopy(joint_state)

    def _joint_state_from_positions(self, positions_by_name: dict[str, float]) -> JointState:
        joint_state = JointState()
        joint_state.name = [name for name in UR_JOINTS if name in positions_by_name]
        joint_state.position = [float(positions_by_name[name]) for name in joint_state.name]
        return joint_state

    def _joint_positions_by_name(self, joint_state: JointState | None) -> dict[str, float]:
        if joint_state is None:
            return {}
        return {
            name: float(position)
            for name, position in zip(joint_state.name, joint_state.position)
        }

    def _normalize_goal_positions_to_start(
        self,
        start_state: JointState,
        goal_state: JointState,
    ) -> list[float]:
        start_by_name = self._joint_positions_by_name(start_state)
        goal_by_name = self._joint_positions_by_name(goal_state)
        ordered = []
        for name in UR_JOINTS:
            if name not in start_by_name or name not in goal_by_name:
                continue
            start = float(start_by_name[name])
            goal = float(goal_by_name[name])
            ordered.append(start + self._normalize_angle_delta(goal - start))
        return ordered

    def _joint_state_distance(
        self,
        current_state: JointState,
        goal_positions: list[float],
    ) -> float:
        current_by_name = self._joint_positions_by_name(current_state)
        if not current_by_name:
            return float("inf")
        total = 0.0
        for name, goal in zip(UR_JOINTS, goal_positions):
            if name not in current_by_name:
                return float("inf")
            weight = float(OMPL_JOINT_COST_WEIGHTS.get(name, 1.0))
            total += weight * abs(
                self._normalize_angle_delta(float(goal) - current_by_name[name])
            )
        return total

    def _trajectory_joint_cost(self, trajectory) -> float:
        points = list(getattr(trajectory, "points", []))
        names = list(getattr(trajectory, "joint_names", []))
        if len(points) < 2 or not names:
            return float("inf")
        by_index = {name: names.index(name) for name in UR_JOINTS if name in names}
        if len(by_index) != len(UR_JOINTS):
            return float("inf")
        total = 0.0
        for previous, current in zip(points, points[1:]):
            previous_positions = list(previous.positions)
            current_positions = list(current.positions)
            for name, index in by_index.items():
                if index >= len(previous_positions) or index >= len(current_positions):
                    return float("inf")
                delta = self._normalize_angle_delta(
                    float(current_positions[index]) - float(previous_positions[index])
                )
                total += float(OMPL_JOINT_COST_WEIGHTS.get(name, 1.0)) * abs(delta)
        return total

    def _build_joint_linear_trajectory(
        self,
        start_state: JointState,
        goal_positions: list[float],
    ) -> JointTrajectory | None:
        start_by_name = self._joint_positions_by_name(start_state)
        if any(name not in start_by_name for name in UR_JOINTS):
            return None
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = list(UR_JOINTS)
        start_positions = [start_by_name[name] for name in UR_JOINTS]
        deltas = [
            self._normalize_angle_delta(float(goal) - float(start))
            for start, goal in zip(start_positions, goal_positions)
        ]
        for index in range(OMPL_DIRECT_PATH_SAMPLE_COUNT + 1):
            ratio = index / float(OMPL_DIRECT_PATH_SAMPLE_COUNT)
            point = JointTrajectoryPoint()
            point.positions = [
                float(start) + delta * ratio
                for start, delta in zip(start_positions, deltas)
            ]
            joint_trajectory.points.append(point)
        return joint_trajectory

    def _direct_joint_path_safe(
        self,
        stage: str,
        start_state: JointState,
        goal_positions: list[float],
    ) -> bool:
        trajectory = self._build_joint_linear_trajectory(start_state, goal_positions)
        if trajectory is None:
            self.get_logger().info(
                "DIRECT_JOINT_PATH_CHECK stage=%s valid=false reason=missing_joint_state"
                % stage
            )
            return False
        _ensure_trajectory_timing(trajectory)
        valid = self._trajectory_arm_clearance_ok(trajectory, stage)
        self.get_logger().info(
            "DIRECT_JOINT_PATH_CHECK stage=%s valid=%s"
            % (stage, "true" if valid else "false")
        )
        if valid:
            self._log_trajectory_joint_summary(stage, trajectory)
        return valid

    def _build_joint_goal_trajectory(
        self,
        stage: str,
        start_state: JointState,
        goal_positions: list[float],
    ) -> JointTrajectory | None:
        if not hasattr(self.moveit2, "plan"):
            return None
        trajectory = self.moveit2.plan(
            joint_positions=list(goal_positions),
            joint_names=list(UR_JOINTS),
            start_joint_state=start_state,
        )
        if trajectory is None:
            return None
        self._normalize_wrist_trajectory_to_current(trajectory)
        return trajectory

    def _goal_joint_candidates_for_pose(
        self,
        stage: str,
        target_pose: list[float],
        current_state: JointState,
    ) -> list[dict]:
        seeds = [current_state]
        current_by_name = self._joint_positions_by_name(current_state)
        home_state = self._joint_state_from_positions(
            {name: float(value) for name, value in zip(UR_JOINTS, HOME_CONFIG)}
        )
        if home_state:
            seeds.append(home_state)
        for joint_name in (
            SHOULDER_PAN_JOINT,
            "ur_shoulder_lift_joint",
            "ur_elbow_joint",
            "ur_wrist_1_joint",
            "ur_wrist_2_joint",
            "ur_wrist_3_joint",
        ):
            if joint_name not in current_by_name:
                continue
            for offset in (math.tau, -math.tau):
                shifted = copy.deepcopy(current_state)
                shifted.position = [
                    (
                        float(position) + offset
                        if name == joint_name
                        else float(position)
                    )
                    for name, position in zip(shifted.name, shifted.position)
                ]
                seeds.append(shifted)

        candidates = []
        seen = set()
        for index, seed in enumerate(seeds):
            ik_solution = self._compute_ik_for_pose(
                "%s_IK_CANDIDATE_%d" % (stage, index),
                self._target_pose(target_pose),
                seed,
            )
            if ik_solution is None:
                continue
            if not self._state_validity_ok_for_joint_state(
                "%s_IK_CANDIDATE_%d" % (stage, index),
                ik_solution,
            ):
                continue
            normalized = self._normalize_goal_positions_to_start(current_state, ik_solution)
            if len(normalized) != len(UR_JOINTS):
                continue
            key = tuple(round(value, 6) for value in normalized)
            if key in seen:
                continue
            seen.add(key)
            candidates.append(
                {
                    "index": index,
                    "seed": seed,
                    "joint_state": ik_solution,
                    "positions": normalized,
                    "distance": self._joint_state_distance(current_state, normalized),
                }
            )
        candidates.sort(key=lambda item: float(item["distance"]))
        for candidate in candidates:
            self.get_logger().info(
                "JOINT_PATH_GOAL_CANDIDATE stage=%s candidate=%d %s distance=%.5f"
                % (
                    stage,
                    candidate["index"],
                    " ".join(
                        "%s=%.5f" % (name, value)
                        for name, value in zip(UR_JOINTS, candidate["positions"])
                    ),
                    float(candidate["distance"]),
                )
            )
        return candidates

    def _select_min_joint_path(
        self,
        stage: str,
        start_state: JointState,
        goal_positions: list[float],
    ) -> JointTrajectory | None:
        best_trajectory = None
        best_cost = float("inf")
        for attempt in range(OMPL_MULTI_CANDIDATE_ATTEMPTS):
            trajectory = self._build_joint_goal_trajectory(
                stage,
                start_state,
                goal_positions,
            )
            if trajectory is None or not getattr(trajectory, "points", None):
                continue
            _ensure_trajectory_timing(trajectory)
            if not self._trajectory_arm_clearance_ok(trajectory, stage):
                continue
            cost = self._trajectory_joint_cost(trajectory)
            self.get_logger().info(
                "OMPL_CANDIDATE stage=%s candidate=%d total_joint_travel=%.5f"
                % (stage, attempt, cost)
            )
            if cost < best_cost:
                best_cost = cost
                best_trajectory = trajectory
        if best_trajectory is None:
            return None
        self.get_logger().info(
            "SELECTED_MIN_JOINT_PATH stage=%s cost=%.5f"
            % (stage, best_cost)
        )
        return best_trajectory

    def _move_pose_with_joint_optimization(
        self,
        stage: str,
        target: list[float],
        timeout_sec: float = DEFAULT_MOVE_TIMEOUT_SEC,
    ) -> bool:
        current_state = self._current_joint_state()
        if current_state is None:
            self.get_logger().warn(
                "JOINT_PATH_START stage=%s unavailable=no_joint_state" % stage
            )
            return False
        self.get_logger().info(
            "JOINT_PATH_START stage=%s %s"
            % (
                stage,
                self._joint_positions_text(current_state),
            )
        )
        candidates = self._goal_joint_candidates_for_pose(stage, target, current_state)
        if not candidates:
            self.get_logger().warn(
                "SELECTED_NEAREST_IK stage=%s unavailable=no_candidates" % stage
            )
            return False
        selected = candidates[0]
        self.get_logger().info(
            "SELECTED_NEAREST_IK stage=%s candidate=%d distance=%.5f"
            % (stage, selected["index"], float(selected["distance"]))
        )
        if self._direct_joint_path_safe(stage, current_state, selected["positions"]):
            trajectory = self._build_joint_linear_trajectory(
                current_state,
                selected["positions"],
            )
            if trajectory is not None:
                _ensure_trajectory_timing(trajectory)
                self._log_trajectory_joint_summary(stage, trajectory)
                return super()._execute_trajectory_via_moveit(
                    trajectory,
                    timeout_sec,
                )
        self.get_logger().warn(
            "DIRECT_JOINT_PATH_CHECK stage=%s valid=false fallback=ompl"
            % stage
        )
        best_trajectory = self._select_min_joint_path(
            stage,
            current_state,
            selected["positions"],
        )
        if best_trajectory is None:
            return False
        self._log_trajectory_joint_summary(stage, best_trajectory)
        return super()._execute_trajectory_via_moveit(best_trajectory, timeout_sec)

    def _plan_execute_pose(
        self,
        pos,
        quat,
        frame_id,
        target_link,
        tolerance_position,
        tolerance_orientation,
        timeout_sec,
        cartesian=False,
        result_grace_sec=None,
    ) -> bool:
        if not hasattr(self.moveit2, "plan"):
            self.get_logger().error(
                "Tube arm clearance requires MoveIt2.plan before execution"
            )
            self._publish_status("FAILED_ARM_CLEARANCE")
            return False
        stage = getattr(self, "_active_stage_label", "UNKNOWN")
        if not cartesian:
            return self._move_pose_with_joint_optimization(
                stage,
                list(pos),
                timeout_sec=timeout_sec,
            )
        trajectory = self.moveit2.plan(
            position=list(pos),
            quat_xyzw=quat,
            frame_id=frame_id,
            target_link=target_link,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
            cartesian=cartesian,
        )
        if trajectory is None:
            return False
        self._normalize_wrist_trajectory_to_current(trajectory)
        _ensure_trajectory_timing(trajectory)
        if not self._trajectory_arm_clearance_ok(
            trajectory,
            stage,
        ):
            self._publish_status("FAILED_ARM_CLEARANCE")
            return False
        return super()._execute_trajectory_via_moveit(
            trajectory,
            timeout_sec,
        )

    def _execute_trajectory_via_moveit(self, trajectory, timeout_sec) -> bool:
        if not self._trajectory_arm_clearance_ok(
            trajectory,
            getattr(self, "_active_stage_label", "UNKNOWN"),
        ):
            self._publish_status("FAILED_ARM_CLEARANCE")
            return False
        return super()._execute_trajectory_via_moveit(trajectory, timeout_sec)

    def go_home(self, stage_label: str = "INITIAL_HOME") -> bool:
        self._active_stage_label = str(stage_label)
        try:
            if str(stage_label) == "RETURN_HOME":
                self._log_return_home_start_state()
                if self._move_joint_goal_with_optimization(
                    "RETURN_HOME",
                    HOME_CONFIG,
                    timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
                ):
                    self.get_logger().info("RETURN_HOME_SUCCESS")
                    return True
                return False
            if self._move_joint_goal_with_optimization(
                "INITIAL_HOME",
                HOME_CONFIG,
                timeout_sec=DEFAULT_MOVE_TIMEOUT_SEC,
            ):
                return True
            return False
        finally:
            self._active_stage_label = "UNKNOWN"

    def _move_joint_goal_with_optimization(
        self,
        stage: str,
        goal_positions: list[float],
        timeout_sec: float,
    ) -> bool:
        if not hasattr(self.moveit2, "plan"):
            self.get_logger().error("%s requires MoveIt2.plan" % stage)
            return False
        current_state = self._current_joint_state()
        if current_state is None:
            self.get_logger().warn(
                "JOINT_PATH_START stage=%s unavailable=no_joint_state" % stage
            )
            return False
        self.get_logger().info(
            "JOINT_PATH_START stage=%s %s"
            % (stage, self._joint_positions_text(current_state))
        )
        goal_state = self._joint_state_from_positions(
            {name: float(value) for name, value in zip(UR_JOINTS, goal_positions)}
        )
        goal_positions = self._normalize_goal_positions_to_start(
            current_state,
            goal_state,
        )
        if len(goal_positions) != len(UR_JOINTS):
            return False
        self.get_logger().info(
            "JOINT_PATH_GOAL_CANDIDATE stage=%s candidate=home %s"
            % (
                stage,
                " ".join(
                    "%s=%.5f" % (name, value)
                    for name, value in zip(UR_JOINTS, goal_positions)
                ),
            )
        )
        if self._direct_joint_path_safe(stage, current_state, goal_positions):
            trajectory = self._build_joint_linear_trajectory(
                current_state,
                goal_positions,
            )
            if trajectory is not None:
                _ensure_trajectory_timing(trajectory)
                self._log_trajectory_joint_summary(stage, trajectory)
                return super()._execute_trajectory_via_moveit(trajectory, timeout_sec)
        best_trajectory = self._select_min_joint_path(
            stage,
            current_state,
            goal_positions,
        )
        if best_trajectory is None:
            return False
        self._log_trajectory_joint_summary(stage, best_trajectory)
        return super()._execute_trajectory_via_moveit(best_trajectory, timeout_sec)

    def _log_return_home_start_state(self) -> None:
        joint_state = getattr(self.moveit2, "joint_state", None)
        self.get_logger().info(
            "RETURN_HOME_START_JOINTS %s"
            % self._joint_positions_text(joint_state)
        )
        pose = self._current_tcp_pose_for_joint_state(joint_state)
        if pose is not None:
            self._log_pose_xyz("RETURN_HOME_START_TCP", pose)

    @staticmethod
    def _normalize_angle_delta(delta: float) -> float:
        return math.atan2(math.sin(float(delta)), math.cos(float(delta)))

    def _trajectory_joint_values(self, trajectory, joint_name: str) -> list[float]:
        names = list(getattr(trajectory, "joint_names", []))
        if joint_name not in names:
            return []
        index = names.index(joint_name)
        values = []
        for point in getattr(trajectory, "points", []):
            if index < len(point.positions):
                values.append(float(point.positions[index]))
        return values

    def _trajectory_joint_motion_summary(
        self,
        trajectory,
        joint_name: str,
    ) -> dict | None:
        values = self._trajectory_joint_values(trajectory, joint_name)
        if not values:
            return None
        deltas = [
            self._normalize_angle_delta(values[index + 1] - values[index])
            for index in range(len(values) - 1)
        ]
        total_travel = sum(abs(delta) for delta in deltas)
        max_point_delta = max((abs(delta) for delta in deltas), default=0.0)
        start = values[0]
        end = values[-1]
        max_abs_delta = max(
            (abs(self._normalize_angle_delta(value - start)) for value in values),
            default=0.0,
        )
        return {
            "start": start,
            "end": end,
            "min": min(values),
            "max": max(values),
            "total_travel": total_travel,
            "max_point_delta": max_point_delta,
            "max_abs_delta": max_abs_delta,
        }

    def _log_trajectory_joint_summary(self, stage: str, trajectory) -> None:
        self.get_logger().info("JOINT_TRAJECTORY_SUMMARY stage=%s" % stage)
        total_joint_travel = 0.0
        for joint_name in UR_JOINTS:
            summary = self._trajectory_joint_motion_summary(
                trajectory,
                joint_name,
            )
            if summary is None:
                continue
            total_joint_travel += float(summary["total_travel"])
            self.get_logger().info(
                "JOINT_TRAJECTORY_JOINT stage=%s joint=%s start=%.5f end=%.5f "
                "min=%.5f max=%.5f total_travel=%.5f"
                % (
                    stage,
                    joint_name,
                    summary["start"],
                    summary["end"],
                    summary["min"],
                    summary["max"],
                    summary["total_travel"],
                )
            )
        self.get_logger().info(
            "TOTAL_JOINT_TRAVEL stage=%s value=%.5f" % (stage, total_joint_travel)
        )

    def _trajectory_arm_clearance_ok(self, trajectory, stage: str) -> bool:
        points = list(getattr(trajectory, "points", []))
        names = list(getattr(trajectory, "joint_names", []))
        if not points or not names:
            return True
        if not self._arm_table_clearance_limits:
            self.get_logger().warn(
                "ARM_TABLE_CLEARANCE_LIMITS missing before stage=%s; "
                "initializing from current state"
                % stage
            )
            if not self._initialize_arm_clearance_baseline():
                self.get_logger().error(
                    "ARM_TABLE_CLEARANCE_VIOLATION stage=%s trajectory_point=-1 "
                    "sample=-1 link=unknown link_min_z=nan table_surface_z=%.4f "
                    "clearance=nan required_margin=nan"
                    % (stage, STATION_B_TABLE_SURFACE_WORLD_Z)
                )
                return False
        sample_indices = self._trajectory_sample_indices(len(points))
        stage_minima = {
            link: float("inf")
            for link in self._arm_table_clearance_limits
        }
        stage_contacts_ok = True
        for sample_number, point_index in enumerate(sample_indices):
            point = points[point_index]
            joint_state = self._joint_state_for_trajectory_point(names, point)
            if not self._state_validity_ok_for_joint_state(
                "%s_POINT_%d" % (stage, point_index),
                joint_state,
            ):
                stage_contacts_ok = False
                return False
            heights = self._arm_link_heights_for_joint_state(joint_state)
            if heights is None:
                self.get_logger().error(
                    "ARM_TABLE_CLEARANCE_VIOLATION stage=%s trajectory_point=%d "
                    "sample=%d link=unknown link_min_z=nan table_surface_z=%.4f "
                    "clearance=nan required_margin=nan"
                    % (
                        stage,
                        point_index,
                        sample_number,
                        STATION_B_TABLE_SURFACE_WORLD_Z,
                    )
                )
                return False
            for link, _origin_z, min_geometry_z in heights:
                limit = self._arm_table_clearance_limits.get(link)
                if limit is None:
                    continue
                stage_minima[link] = min(stage_minima[link], min_geometry_z)
                clearance = min_geometry_z - STATION_B_TABLE_SURFACE_WORLD_Z
                valid = self._arm_table_clearance_valid(clearance, limit)
                self.get_logger().info(
                    "ARM_TABLE_CLEARANCE stage=%s link=%s link_min_z=%.4f "
                    "table_surface_z=%.4f clearance=%.4f required_margin=%.4f "
                    "tolerance=%.4f valid=%s"
                    % (
                        stage,
                        link,
                        min_geometry_z,
                        STATION_B_TABLE_SURFACE_WORLD_Z,
                        clearance,
                        limit["required_margin"],
                        ARM_TABLE_CLEARANCE_NUMERIC_TOLERANCE,
                        "true" if valid else "false",
                    )
                )
                if not valid:
                    self.get_logger().error(
                        "ARM_TABLE_CLEARANCE_VIOLATION stage=%s trajectory_point=%d "
                        "sample=%d link=%s link_min_z=%.4f table_surface_z=%.4f "
                        "clearance=%.4f required_margin=%.4f tolerance=%.4f"
                        % (
                            stage,
                            point_index,
                            sample_number,
                            link,
                            min_geometry_z,
                            STATION_B_TABLE_SURFACE_WORLD_Z,
                            clearance,
                            limit["required_margin"],
                            ARM_TABLE_CLEARANCE_NUMERIC_TOLERANCE,
                        )
                    )
                    return False
        for link, min_geometry_z in stage_minima.items():
            limit = self._arm_table_clearance_limits[link]
            clearance = min_geometry_z - STATION_B_TABLE_SURFACE_WORLD_Z
            self.get_logger().info(
                "ARM_TABLE_CLEARANCE_OK stage=%s link=%s min_z=%.4f "
                "table_surface_z=%.4f min_clearance=%.4f required_margin=%.4f "
                "tolerance=%.4f state_validity=%s"
                % (
                    stage,
                    link,
                    min_geometry_z,
                    STATION_B_TABLE_SURFACE_WORLD_Z,
                    clearance,
                    limit["required_margin"],
                    ARM_TABLE_CLEARANCE_NUMERIC_TOLERANCE,
                    "true" if stage_contacts_ok else "false",
                )
            )
        return True

    @staticmethod
    def _arm_table_clearance_valid(clearance: float, limit: dict) -> bool:
        return (
            float(clearance) + ARM_TABLE_CLEARANCE_NUMERIC_TOLERANCE
            >= float(limit["required_margin"])
        )

    def _state_validity_ok_for_joint_state(self, label: str, joint_state) -> bool:
        if not self._state_validity_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().error(
                "%s_STATE_VALID unavailable: /check_state_validity not ready"
                % label
            )
            return False
        request = GetStateValidity.Request()
        request.group_name = "ur_manipulator"
        request.robot_state = RobotState()
        request.robot_state.joint_state = joint_state
        response = self._wait_for_cartesian_response(
            self._state_validity_client.call_async(request),
            1.0,
        )
        if response is None:
            self.get_logger().error("%s_STATE_VALID unavailable: no response" % label)
            return False
        contacts = list(getattr(response, "contacts", []))
        self.get_logger().info(
            "%s_STATE_VALID valid=%s contacts=%d"
            % (label, "true" if response.valid else "false", len(contacts))
        )
        if response.valid and not contacts:
            return True
        if contacts:
            contact = contacts[0]
            self.get_logger().error(
                "%s_FIRST_INVALID_COLLISION_PAIR %s <-> %s depth=%.6f"
                % (
                    label,
                    contact.contact_body_1,
                    contact.contact_body_2,
                    float(contact.depth),
                )
            )
            if str(label).startswith("RETURN_HOME"):
                self.get_logger().error(
                    "RETURN_HOME_COLLISION link=%s object=%s depth=%.6f"
                    % (
                        contact.contact_body_1,
                        contact.contact_body_2,
                        float(contact.depth),
                    )
                )
        return False

    def _trajectory_sample_indices(self, count: int) -> list[int]:
        if count <= ARM_CLEARANCE_SAMPLE_COUNT:
            return list(range(count))
        indices = {
            round(index * (count - 1) / (ARM_CLEARANCE_SAMPLE_COUNT - 1))
            for index in range(ARM_CLEARANCE_SAMPLE_COUNT)
        }
        return sorted(indices)

    def _joint_state_for_trajectory_point(self, names: list[str], point) -> JointState:
        current = getattr(self.moveit2, "joint_state", None)
        by_name = {}
        if current is not None:
            by_name.update({
                name: float(position)
                for name, position in zip(current.name, current.position)
            })
        by_name.update({
            name: float(position)
            for name, position in zip(names, point.positions)
        })
        joint_state = JointState()
        joint_state.name = [
            name for name in UR_JOINTS
            if name in by_name
        ]
        joint_state.position = [by_name[name] for name in joint_state.name]
        return joint_state

    def _arm_link_heights_for_joint_state(self, joint_state):
        if not self._fk_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().error(
                "ARM_LINK_HEIGHT unavailable: /compute_fk not ready"
            )
            return None
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = list(PROTECTED_ARM_LINKS)
        request.robot_state = RobotState()
        request.robot_state.joint_state = joint_state
        response = self._wait_for_cartesian_response(
            self._fk_client.call_async(request),
            1.0,
        )
        if response is None:
            self.get_logger().error("ARM_LINK_HEIGHT unavailable: no FK response")
            return None
        poses_by_link = {
            name: pose
            for name, pose in zip(response.fk_link_names, response.pose_stamped)
        }
        heights = []
        for link in PROTECTED_ARM_LINKS:
            pose_stamped = poses_by_link.get(link)
            if pose_stamped is None:
                self.get_logger().error(
                    "ARM_LINK_HEIGHT unavailable: missing FK for %s" % link
                )
                return None
            origin_world_z = BASE_LINK_WORLD_Z + float(pose_stamped.pose.position.z)
            min_geometry_z = (
                origin_world_z - PROTECTED_ARM_LINK_GEOMETRY_HALF_Z[link]
            )
            heights.append((link, origin_world_z, min_geometry_z))
        return heights

    def _current_tcp_target_reached(self, target: list[float]) -> bool:
        joint_state = getattr(self.moveit2, "joint_state", None)
        if joint_state is None:
            return False
        if not self._fk_client.wait_for_service(timeout_sec=0.2):
            return False
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = [GRIPPER_TCP_LINK]
        request.robot_state = RobotState()
        request.robot_state.joint_state = joint_state
        response = self._wait_for_cartesian_response(
            self._fk_client.call_async(request),
            0.5,
        )
        if response is None or not response.pose_stamped:
            return False
        pose = response.pose_stamped[0].pose
        distance = (
            (float(pose.position.x) - float(target[0])) ** 2
            + (float(pose.position.y) - float(target[1])) ** 2
            + (float(pose.position.z) - float(target[2])) ** 2
        ) ** 0.5
        return distance <= TUBE_READY_PRE_GRASP_TOLERANCE_POSITION

    def _move_ompl_if_needed(self, label: str, target: list[float]) -> bool:
        if self._current_tcp_target_reached(target):
            self._publish_status("%s_ALREADY_REACHED" % label)
            return True
        return self._move_ompl(label, target)

    def _move_cartesian_fraction(self, label: str, target: list[float]) -> float:
        self._active_stage_label = label
        try:
            return self._move_cartesian_fraction_checked(label, target)
        finally:
            self._active_stage_label = "UNKNOWN"

    def _move_cartesian_fraction_checked(self, label: str, target: list[float]) -> float:
        self._publish_status(label)
        if not self._cartesian_path_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("MoveIt compute_cartesian_path unavailable")
            return 0.0
        joint_state = getattr(self.moveit2, "joint_state", None)
        start_pose = self._current_tcp_pose_for_joint_state(
            joint_state
        )
        if label.startswith("INSERT"):
            if start_pose is not None:
                self._log_pose_xyz("%s_START_TCP" % label, start_pose)
            self._log_xyz("%s_TARGET_TCP" % label, target)
            if label in ("INSERT_STAGE2", "INSERT_FINAL"):
                self._log_joint_state("%s_START_JOINTS" % label, joint_state)
        if label == "INSERT_FINAL":
            self._diagnose_insert_final_continuity_scan(target, joint_state)
        if label == "POST_INSERT_VERTICAL_RETREAT":
            if start_pose is not None:
                self._log_pose_xyz("POST_INSERT_RETREAT_START_TCP", start_pose)
            self._log_xyz("POST_INSERT_RETREAT_TARGET_TCP", target)

        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.group_name = "ur_manipulator"
        request.link_name = GRIPPER_TCP_LINK
        request.waypoints = [self._target_pose(target)]
        request.max_step = self._cartesian_max_step(label)
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        if joint_state is not None:
            request.start_state.joint_state = joint_state
            request.start_state.is_diff = True

        response = self._wait_for_cartesian_response(
            self._cartesian_path_client.call_async(request),
            DEFAULT_MOVE_TIMEOUT_SEC,
        )
        if response is None:
            return 0.0
        fraction = float(response.fraction)
        required_fraction = CARTESIAN_MIN_FRACTION
        if label == "POST_INSERT_VERTICAL_RETREAT":
            required_fraction = POST_INSERT_RETREAT_MIN_FRACTION
        if fraction < required_fraction:
            self._diagnose_cartesian_fraction_failure(
                label,
                target,
                start_pose,
                response,
            )
            return fraction
        trajectory = response.solution.joint_trajectory
        if not trajectory.points:
            return 0.0
        self._normalize_wrist_trajectory_to_current(trajectory)
        _ensure_trajectory_timing(trajectory)
        if not self._trajectory_arm_clearance_ok(trajectory, label):
            self._publish_status("FAILED_ARM_CLEARANCE")
            return 0.0
        if not super()._execute_trajectory_via_moveit(
            trajectory,
            DEFAULT_MOVE_TIMEOUT_SEC,
        ):
            return 0.0
        if (
            getattr(self, "_hold_monitor_active", False)
            and not self._holding_is_healthy()
        ):
            self._handle_hold_lost()
            return 0.0
        return fraction

    @staticmethod
    def _cartesian_max_step(label: str) -> float:
        if label == "INSERT_FINAL":
            return INSERT_FINAL_CARTESIAN_MAX_STEP
        return DEFAULT_CARTESIAN_MAX_STEP

    def _current_tcp_pose_for_joint_state(self, joint_state) -> Pose | None:
        if joint_state is None:
            return None
        if not self._fk_client.wait_for_service(timeout_sec=0.2):
            return None
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = [GRIPPER_TCP_LINK]
        request.robot_state = RobotState()
        request.robot_state.joint_state = joint_state
        response = self._wait_for_cartesian_response(
            self._fk_client.call_async(request),
            0.8,
        )
        if response is None or not response.pose_stamped:
            return None
        return response.pose_stamped[0].pose

    def _tcp_pose_for_trajectory_endpoint(self, trajectory) -> Pose | None:
        points = list(getattr(trajectory, "points", []))
        names = list(getattr(trajectory, "joint_names", []))
        if not points or not names:
            return None
        joint_state = self._joint_state_for_trajectory_point(names, points[-1])
        return self._current_tcp_pose_for_joint_state(joint_state)

    def _diagnose_cartesian_fraction_failure(
        self,
        label: str,
        target: list[float],
        start_pose: Pose | None,
        response,
    ) -> None:
        fraction = float(getattr(response, "fraction", 0.0))
        trajectory = response.solution.joint_trajectory
        last_valid = self._tcp_pose_for_trajectory_endpoint(trajectory)
        if last_valid is None:
            last_valid = start_pose
        if last_valid is not None:
            self._log_pose_xyz("%s_LAST_VALID_TCP" % label, last_valid)
        first_failed = self._first_failed_cartesian_pose(
            start_pose,
            target,
            fraction,
        )
        if first_failed is not None:
            self._log_pose_xyz("%s_FIRST_FAILED_TCP" % label, first_failed)
            self._diagnose_pose_validity("%s_FIRST_FAILED" % label, first_failed)
        else:
            self.get_logger().warn(
                "%s_FIRST_FAILED_TCP unavailable fraction=%.3f" % (label, fraction)
            )

    def _diagnose_insert_final_continuity_scan(
        self,
        target: list[float],
        seed_state: JointState | None,
    ) -> None:
        if seed_state is None:
            self.get_logger().warn(
                "INSERT_FINAL_CONTINUITY_SCAN skipped reason=no_seed_state"
            )
            return
        self.get_logger().info(
            "INSERT_FINAL_CONTINUITY_SCAN start_z=0.6950 target_z=0.6900"
        )
        previous_state = seed_state
        first_failure_z = None
        for z in INSERT_FINAL_CONTINUITY_SCAN_ZS:
            label = "INSERT_FINAL_CONTINUITY_Z%.4f" % z
            pose = self._target_pose([target[0], target[1], z])
            ik_solution = self._compute_ik_for_pose(label, pose, previous_state)
            if ik_solution is None:
                if first_failure_z is None:
                    first_failure_z = z
                self.get_logger().info(
                    "%s_STATE_VALID skipped reason=ik_failed" % label
                )
                continue
            self._log_joint_state("%s_IK_JOINT_SOLUTION" % label, ik_solution)
            max_delta = self._max_joint_delta(previous_state, ik_solution)
            self.get_logger().info(
                "%s_ADJACENT_MAX_JOINT_DELTA value=%.6f" % (label, max_delta)
            )
            response = self._state_validity_response_for_joint_state(
                ik_solution,
                label,
            )
            if response is None:
                if first_failure_z is None:
                    first_failure_z = z
                self.get_logger().info(
                    "%s_STATE_VALID unavailable contacts=0" % label
                )
                continue
            contacts = list(getattr(response, "contacts", []))
            self.get_logger().info(
                "%s_STATE_VALID valid=%s contacts=%d"
                % (label, "true" if response.valid else "false", len(contacts))
            )
            if (not response.valid or contacts) and first_failure_z is None:
                first_failure_z = z
            previous_state = ik_solution
        if first_failure_z is None:
            self.get_logger().info("INSERT_FINAL_FIRST_CONTINUITY_FAILURE_Z none")
        else:
            self.get_logger().info(
                "INSERT_FINAL_FIRST_CONTINUITY_FAILURE_Z %.4f" % first_failure_z
            )

    def _first_failed_cartesian_pose(
        self,
        start_pose: Pose | None,
        target: list[float],
        fraction: float,
    ) -> Pose | None:
        if start_pose is None:
            return None
        dx = float(target[0]) - float(start_pose.position.x)
        dy = float(target[1]) - float(start_pose.position.y)
        dz = float(target[2]) - float(start_pose.position.z)
        distance = (dx * dx + dy * dy + dz * dz) ** 0.5
        if distance <= 1e-9:
            alpha = min(max(float(fraction), 0.0) + 0.01, 1.0)
        else:
            alpha = min(max(float(fraction), 0.0) + 0.005 / distance, 1.0)
        pose = Pose()
        pose.position.x = float(start_pose.position.x) + dx * alpha
        pose.position.y = float(start_pose.position.y) + dy * alpha
        pose.position.z = float(start_pose.position.z) + dz * alpha
        pose.orientation = self._target_pose(target).orientation
        return pose

    def _diagnose_pose_validity(self, label: str, pose: Pose) -> None:
        ik_solution = self._compute_ik_for_pose(label, pose)
        if ik_solution is None:
            return
        response = self._state_validity_response_for_joint_state(
            ik_solution,
            label,
        )
        if response is None:
            return
        contacts = list(getattr(response, "contacts", []))
        self.get_logger().info(
            "%s_STATE_VALID valid=%s contacts=%d"
            % (label, "true" if response.valid else "false", len(contacts))
        )
        for contact in contacts[:5]:
            self.get_logger().error(
                "%s_COLLISION robot_or_attached_link=%s object=%s depth=%.6f"
                % (
                    label,
                    contact.contact_body_1,
                    contact.contact_body_2,
                    float(contact.depth),
                )
            )
        self._last_cartesian_failure_allows_insert_acm = any(
            self._is_insert_target_tube_rack_collision(
                contact.contact_body_1,
                contact.contact_body_2,
            )
            for contact in contacts
        ) and all(
            self._is_insert_target_tube_rack_collision(
                contact.contact_body_1,
                contact.contact_body_2,
            )
            for contact in contacts
        )

    def _compute_ik_for_pose(
        self,
        label: str,
        pose: Pose,
        seed_state: JointState | None = None,
    ) -> JointState | None:
        if not self._ik_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("%s_IK_VALID unavailable: /compute_ik not ready" % label)
            return None
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator"
        request.ik_request.ik_link_name = GRIPPER_TCP_LINK
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 1
        joint_state = seed_state
        if joint_state is None:
            joint_state = getattr(self.moveit2, "joint_state", None)
        if joint_state is not None:
            request.ik_request.robot_state = RobotState()
            request.ik_request.robot_state.joint_state = joint_state
        response = self._wait_for_cartesian_response(
            self._ik_client.call_async(request),
            1.3,
        )
        if response is None:
            self.get_logger().warn("%s_IK_VALID unavailable: no response" % label)
            return None
        code = int(response.error_code.val)
        valid = code == 1
        self.get_logger().info(
            "%s_IK_VALID valid=%s error_code=%d"
            % (label, "true" if valid else "false", code)
        )
        if not valid:
            return None
        return response.solution.joint_state

    def _max_joint_delta(
        self,
        previous_state: JointState | None,
        next_state: JointState | None,
    ) -> float:
        previous = self._joint_positions_by_name(previous_state)
        next_positions = self._joint_positions_by_name(next_state)
        deltas = [
            abs(next_positions[name] - previous[name])
            for name in UR_JOINTS
            if name in previous and name in next_positions
        ]
        if not deltas:
            return float("nan")
        return max(deltas)

    @staticmethod
    def _joint_positions_by_name(joint_state: JointState | None) -> dict[str, float]:
        if joint_state is None:
            return {}
        return {
            name: float(position)
            for name, position in zip(joint_state.name, joint_state.position)
        }

    def _state_validity_response_for_joint_state(self, joint_state, label: str):
        if not self._state_validity_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(
                "%s_STATE_VALID unavailable: /check_state_validity not ready"
                % label
            )
            return None
        request = GetStateValidity.Request()
        request.group_name = "ur_manipulator"
        request.robot_state = RobotState()
        request.robot_state.joint_state = joint_state
        return self._wait_for_cartesian_response(
            self._state_validity_client.call_async(request),
            1.0,
        )

    def _post_insert_vertical_retreat_target(self, rack: dict) -> list[float]:
        pose = self._current_tcp_pose_for_joint_state(
            getattr(self.moveit2, "joint_state", None)
        )
        if pose is not None:
            x = float(pose.position.x)
            y = float(pose.position.y)
        else:
            x = float(rack["insert_final_tcp"][0])
            y = float(rack["insert_final_tcp"][1])
        return [x, y, float(rack["pre_insert_tcp"][2])]

    def _post_insert_safe_state_ok(self) -> bool:
        joint_state = getattr(self.moveit2, "joint_state", None)
        if joint_state is None:
            self.get_logger().warn("POST_INSERT_SAFE_STATE unavailable: no joint_state")
            return False
        response = self._state_validity_response_for_joint_state(
            joint_state,
            "POST_INSERT_SAFE",
        )
        if response is None:
            self.get_logger().warn("POST_INSERT_SAFE_STATE unavailable: no response")
            return False
        contacts = list(getattr(response, "contacts", []))
        self.get_logger().info(
            "POST_INSERT_SAFE_STATE valid=%s contacts=%d"
            % ("true" if response.valid else "false", len(contacts))
        )
        for contact in contacts[:5]:
            self.get_logger().error(
                "POST_INSERT_SAFE_COLLISION link=%s object=%s depth=%.6f"
                % (
                    contact.contact_body_1,
                    contact.contact_body_2,
                    float(contact.depth),
                )
            )
        return bool(response.valid) and not contacts

    @staticmethod
    def _is_insert_target_tube_rack_collision(body_a: str, body_b: str) -> bool:
        text_a = str(body_a)
        text_b = str(body_b)
        tube_a = TEST_TUBE_ATTACHED_ID in text_a
        tube_b = TEST_TUBE_ATTACHED_ID in text_b
        rack_a = INSERT_ACM_RACK2_OBJECT_PREFIX in text_a
        rack_b = INSERT_ACM_RACK2_OBJECT_PREFIX in text_b
        return (tube_a and rack_b) or (tube_b and rack_a)

    def _log_pose_xyz(self, label: str, pose: Pose) -> None:
        self.get_logger().info(
            "%s x=%.4f y=%.4f z=%.4f"
            % (label, pose.position.x, pose.position.y, pose.position.z)
        )

    def _log_joint_state(self, label: str, joint_state: JointState | None) -> None:
        positions = self._joint_positions_by_name(joint_state)
        if not positions:
            self.get_logger().warn("%s unavailable" % label)
            return
        self.get_logger().info(
            "%s %s"
            % (
                label,
                " ".join(
                    "%s=%.5f" % (name, positions[name])
                    for name in UR_JOINTS
                    if name in positions
                ),
            )
        )

    def _target_pose(self, target: list[float]) -> Pose:
        quat = self._fixed_quat()
        pose = Pose()
        pose.position.x = float(target[0])
        pose.position.y = float(target[1])
        pose.position.z = float(target[2])
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def _wait_for_cartesian_response(self, future, timeout_sec: float):
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                return None
            time.sleep(0.02)
        return future.result()

    def _require_cartesian(self, label: str, target: list[float]) -> bool:
        self._last_cartesian_failure_allows_insert_acm = False
        fraction = self._move_cartesian_fraction(label, target)
        required_fraction = CARTESIAN_MIN_FRACTION
        if label == "POST_INSERT_VERTICAL_RETREAT":
            required_fraction = POST_INSERT_RETREAT_MIN_FRACTION
        if (
            fraction < required_fraction
            and label in INSERT_ACM_STAGES
            and self._last_cartesian_failure_allows_insert_acm
            and not self._insert_acm_active
        ):
            self.get_logger().warn(
                "%s enabling insert-only ACM for attached test_tube_1 <-> rack2 "
                "then retrying Cartesian path" % label
            )
            if self._enable_insert_only_acm():
                self._last_cartesian_failure_allows_insert_acm = False
                fraction = self._move_cartesian_fraction(label, target)
        if label == "POST_INSERT_VERTICAL_RETREAT":
            self._publish_status("POST_INSERT_RETREAT fraction %.3f" % fraction)
        else:
            self._publish_status("%s fraction %.3f" % (label, fraction))
        return fraction >= required_fraction

    def _tube_pre_close(self) -> bool:
        command = [TUBE_PRE_GRASP_POSITION, TUBE_PRE_GRASP_POSITION]
        command_ros_time = self._ros_now_sec()
        if not self.gripper.command_positions(command):
            return False
        if not self._wait_for_tube_pre_close_reached(
            command[0],
            command[1],
            min_feedback_ros_time=command_ros_time,
        ):
            return False
        _stamp, left_actual, right_actual = self._current_tube_finger_feedback(
            min_ros_time=command_ros_time,
        )
        if left_actual is None or right_actual is None:
            left_actual, right_actual = self.gripper.actual_finger_positions()
        gap_mm = self._gripper_gap_mm(left_actual, right_actual, command)
        self.get_logger().info(
            "TUBE_PRE_CLOSE_DONE left_command=%.6f right_command=%.6f "
            "left_actual=%s right_actual=%s estimated_gap_mm=%.2f"
            % (
                command[0],
                command[1],
                self._format_optional_float(left_actual),
                self._format_optional_float(right_actual),
                gap_mm,
            )
        )
        return True

    def _tube_post_insert_release_opening(self) -> bool:
        command = [
            TUBE_POST_INSERT_RELEASE_POSITION,
            TUBE_POST_INSERT_RELEASE_POSITION,
        ]
        if not self.gripper.command_positions(command):
            return False
        left_actual, right_actual = self.gripper.actual_finger_positions()
        gap_mm = self.gripper.estimated_gap_mm(command)
        self.get_logger().info(
            "TUBE_POST_INSERT_RELEASE_OPENING target_gap_mm=%.2f "
            "left_command=%.6f right_command=%.6f left_actual=%s "
            "right_actual=%s estimated_gap_mm=%.2f"
            % (
                TUBE_POST_INSERT_RELEASE_GAP * 1000.0,
                command[0],
                command[1],
                self._format_optional_float(left_actual),
                self._format_optional_float(right_actual),
                gap_mm,
            )
        )
        return True

    def _wait_for_tube_pre_close_reached(
        self,
        left_target: float,
        right_target: float,
        *,
        min_feedback_ros_time: float,
        timeout_sec: float = TUBE_PRE_CLOSE_TIMEOUT_SEC,
        tolerance: float = TUBE_PRE_CLOSE_POSITION_TOLERANCE,
    ) -> bool:
        start_ros = min_feedback_ros_time
        start_wall = time.monotonic()
        last_progress_ros = start_ros
        last_progress_wall = start_wall
        last_log_wall = start_wall - TUBE_PRE_CLOSE_WAIT_LOG_PERIOD_SEC

        while True:
            now_ros = self._ros_now_sec()
            now_wall = time.monotonic()
            if now_ros > last_progress_ros + 1.0e-6:
                last_progress_ros = now_ros
                last_progress_wall = now_wall

            reached, left_actual, right_actual, left_error, right_error = (
                self._tube_pre_close_reached(
                    left_target,
                    right_target,
                    min_feedback_ros_time=min_feedback_ros_time,
                    tolerance=tolerance,
                )
            )
            if reached:
                return True

            sim_elapsed = max(0.0, now_ros - start_ros)
            wall_elapsed = max(0.0, now_wall - start_wall)
            if now_wall - last_log_wall >= TUBE_PRE_CLOSE_WAIT_LOG_PERIOD_SEC:
                self.get_logger().info(
                    "TUBE_PRE_CLOSE_WAIT left_target=%.6f right_target=%.6f "
                    "left_actual=%s right_actual=%s left_error=%s right_error=%s "
                    "sim_elapsed=%.3f wall_elapsed=%.3f"
                    % (
                        left_target,
                        right_target,
                        self._format_optional_float(left_actual),
                        self._format_optional_float(right_actual),
                        self._format_optional_float(left_error),
                        self._format_optional_float(right_error),
                        sim_elapsed,
                        wall_elapsed,
                    )
                )
                last_log_wall = now_wall

            if sim_elapsed >= max(float(timeout_sec), 0.0):
                self.get_logger().warn(
                    "TUBE_PRE_CLOSE_FAILED reason=POSITION_NOT_REACHED "
                    "left_actual=%s right_actual=%s left_error=%s right_error=%s "
                    "tolerance=%.6f sim_elapsed=%.3f wall_elapsed=%.3f"
                    % (
                        self._format_optional_float(left_actual),
                        self._format_optional_float(right_actual),
                        self._format_optional_float(left_error),
                        self._format_optional_float(right_error),
                        tolerance,
                        sim_elapsed,
                        wall_elapsed,
                    )
                )
                return False

            if now_wall - last_progress_wall >= TUBE_PRE_CLOSE_CLOCK_STALL_SEC:
                self.get_logger().warn(
                    "TUBE_PRE_CLOSE_CLOCK_STALLED left_actual=%s right_actual=%s "
                    "left_error=%s right_error=%s wall_elapsed=%.3f"
                    % (
                        self._format_optional_float(left_actual),
                        self._format_optional_float(right_actual),
                        self._format_optional_float(left_error),
                        self._format_optional_float(right_error),
                        wall_elapsed,
                    )
                )
                return False

            time.sleep(TUBE_PRE_CLOSE_POLL_SEC)

    def _tube_pre_close_reached(
        self,
        left_target: float,
        right_target: float,
        *,
        min_feedback_ros_time: float,
        tolerance: float = TUBE_PRE_CLOSE_POSITION_TOLERANCE,
    ) -> tuple[bool, float | None, float | None, float | None, float | None]:
        _stamp, left_actual, right_actual = self._current_tube_finger_feedback(
            min_ros_time=min_feedback_ros_time,
        )
        left_error = self._position_error(left_actual, left_target)
        right_error = self._position_error(right_actual, right_target)
        reached = (
            left_error is not None
            and right_error is not None
            and left_error <= tolerance
            and right_error <= tolerance
        )
        return reached, left_actual, right_actual, left_error, right_error

    def _current_tube_finger_feedback(
        self,
        *,
        min_ros_time: float,
    ) -> tuple[float | None, float | None, float | None]:
        joint_state = getattr(getattr(self, "moveit2", None), "joint_state", None)
        if joint_state is None:
            return None, None, None
        stamp = self._joint_state_stamp_sec(joint_state)
        if stamp is None or stamp <= min_ros_time + 1.0e-6:
            return stamp, None, None
        positions_by_name = {
            str(name): float(position)
            for name, position in zip(joint_state.name, joint_state.position)
        }
        return (
            stamp,
            positions_by_name.get("gripper_left_finger_joint"),
            positions_by_name.get("gripper_right_finger_joint"),
        )

    @staticmethod
    def _joint_state_stamp_sec(joint_state: JointState) -> float | None:
        stamp = getattr(getattr(joint_state, "header", None), "stamp", None)
        if stamp is None:
            return None
        sec = getattr(stamp, "sec", None)
        nanosec = getattr(stamp, "nanosec", None)
        if sec is None or nanosec is None:
            return None
        return float(sec) + float(nanosec) * 1.0e-9

    @staticmethod
    def _gripper_gap_mm(
        left_actual: float | None,
        right_actual: float | None,
        command: list[float],
    ) -> float:
        if left_actual is not None and right_actual is not None:
            return (
                GRIPPER_OPEN_INNER_GAP
                - float(left_actual)
                - float(right_actual)
            ) * 1000.0
        return (
            GRIPPER_OPEN_INNER_GAP
            - float(command[0])
            - float(command[1])
        ) * 1000.0

    @staticmethod
    def _position_error(actual: float | None, target: float) -> float | None:
        if actual is None:
            return None
        return abs(float(actual) - float(target))

    @staticmethod
    def _format_optional_float(value: float | None) -> str:
        if value is None:
            return "nan"
        return "%.6f" % float(value)

    def _ros_now_sec(self) -> float:
        try:
            now = self.get_clock().now()
        except Exception:  # noqa: BLE001
            return time.monotonic()
        nanoseconds = getattr(now, "nanoseconds", None)
        if nanoseconds is not None:
            return float(nanoseconds) * 1.0e-9
        seconds_nanoseconds = getattr(now, "seconds_nanoseconds", None)
        if callable(seconds_nanoseconds):
            sec, nanosec = seconds_nanoseconds()
            return float(sec) + float(nanosec) * 1.0e-9
        try:
            return float(now)
        except (TypeError, ValueError):
            return time.monotonic()

    def _run_sequence(self) -> None:
        tube = TUBE_INSERT_CONFIG["test_tube_1"]
        rack = TUBE_INSERT_CONFIG["test_tube_rack_2"]
        try:
            self._publish_status("START")
            if not self.gripper.open():
                self._publish_status("FAILED_GRIPPER_OPEN")
                return
            if not self._move_ompl_if_needed("PRE_GRASP", tube["pre_grasp_tcp"]):
                self._publish_status("FAILED_PRE_GRASP")
                return
            self._publish_status("TUBE_PRE_CLOSE")
            if not self._tube_pre_close():
                self._publish_status("FAILED_TUBE_PRE_CLOSE")
                return
            self._log_xyz("DESCEND_GRASP_START_TCP", tube["pre_grasp_tcp"])
            self._log_xyz("DESCEND_GRASP_TARGET_TCP", tube["grasp_tcp"])
            if not self._require_cartesian("DESCEND_GRASP", tube["grasp_tcp"]):
                self._publish_status("FAILED_DESCEND_GRASP")
                return
            if PRE_GRASP_SETTLE_SEC > 0.0:
                time.sleep(PRE_GRASP_SETTLE_SEC)
            self._publish_status("TUBE_FINAL_CLOSE")
            if not self.gripper.acquire_object():
                self.gripper.open()
                self._publish_status("FAILED_ATTACH")
                return
            if not self._wait_until_gripper_holding():
                self.gripper.release_object()
                self.gripper.open()
                self._publish_status("FAILED_ATTACH_CONFIRM")
                return
            self._publish_status("ATTACHED")
            if not self._attach_tube_collision_object():
                self.gripper.release_object()
                self.gripper.open()
                self._publish_status("FAILED_PLANNING_SCENE_ATTACH")
                return
            self._start_hold_monitor()
            if POST_GRASP_SETTLE_SEC > 0.0:
                time.sleep(POST_GRASP_SETTLE_SEC)
            if not self._require_cartesian("LIFT", tube["lift_tcp"]):
                self._abort_carried_tube("FAILED_LIFT")
                return
            if not self._move_ompl("PRE_INSERT", rack["pre_insert_tcp"]):
                self._handle_hold_lost()
                self._abort_carried_tube("FAILED_PRE_INSERT")
                return
            if not self._require_cartesian(
                "INSERT_STAGE1",
                rack["insert_stage1_tcp"],
            ):
                self._handle_hold_lost()
                self._abort_carried_tube("FAILED_INSERT_STAGE1")
                return
            if not self._require_cartesian(
                "INSERT_STAGE2",
                rack["insert_stage2_tcp"],
            ):
                self._handle_hold_lost()
                self._abort_carried_tube("FAILED_INSERT_STAGE2")
                return
            if not self._require_cartesian(
                "INSERT_FINAL",
                rack["insert_final_tcp"],
            ):
                self._handle_hold_lost()
                self._abort_carried_tube("FAILED_INSERT_FINAL")
                return
            self._stop_hold_monitor()
            self._restore_insert_only_acm()
            if not self.gripper.release_object():
                self._detach_tube_collision_object()
                self._publish_status("FAILED_RELEASE")
                return
            self._detach_tube_collision_object()
            self._publish_status("TUBE_POST_INSERT_RELEASE_OPENING")
            if not self._tube_post_insert_release_opening():
                self._publish_status("FAILED_POST_INSERT_RELEASE_OPENING")
                return
            retreat_target = self._post_insert_vertical_retreat_target(rack)
            if not self._require_cartesian(
                "POST_INSERT_VERTICAL_RETREAT",
                retreat_target,
            ):
                self._publish_status("FAILED_POST_INSERT_VERTICAL_RETREAT")
                return
            self.get_logger().info("POST_INSERT_RETREAT_SUCCESS")
            if not self._post_insert_safe_state_ok():
                self._publish_status("FAILED_POST_INSERT_SAFE_STATE")
                return
            if not self.go_home(stage_label="RETURN_HOME"):
                self._publish_status("FAILED_RETURN_HOME")
                return
            if not self.gripper.open():
                self._publish_status("FAILED_GRIPPER_REOPEN")
                return
            self._publish_status("SUCCESS")
        except Exception as exc:  # noqa: BLE001
            self._restore_insert_only_acm()
            self._publish_status("FAILED_EXCEPTION %s" % exc)
        finally:
            self._release_busy()


def main() -> None:
    rclpy.init()
    node = TubeInsertValidation()
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
