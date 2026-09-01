#!/usr/bin/env python3
"""Independent tooling-zone grasp validation entry point."""
from __future__ import annotations

import copy
import math
import time
from threading import Lock, Thread

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState, GetModelProperties
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetStateValidity
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from std_msgs.msg import String

from lab_cobot_bringup.grasp_target_config import (
    GRASP_TARGETS,
    get_target_config,
)
from lab_cobot_manipulation.pick_place_node import (
    DEFAULT_APPROACH_TOLERANCE_ORIENTATION,
    DEFAULT_APPROACH_TOLERANCE_POSITION,
    DEFAULT_GRASP_TOLERANCE_ORIENTATION,
    DEFAULT_GRASP_TOLERANCE_POSITION,
    DEFAULT_MOVE_TIMEOUT_SEC,
    GRIPPER_TCP_LINK,
    POST_GRASP_SETTLE_SEC,
    PRE_GRASP_SETTLE_SEC,
    PickPlace,
    _ensure_trajectory_timing,
)
from lab_cobot_manipulation.gripper_driver import (
    FORCE_CONTROL_BALANCE_FRAMES,
    FORCE_CONTROL_BALANCE_LIMIT_N,
    FORCE_CONTROL_DEADBAND_N,
    FORCE_CONTROL_FILTER_WINDOW,
    FORCE_CONTROL_KP,
    FORCE_CONTROL_MAX_CLOSE_STEP,
    FORCE_CONTROL_MAX_OPEN_STEP,
    FORCE_CONTROL_SAFETY_FRAMES,
    FORCE_CONTROL_SAFETY_LIMIT_N,
    FORCE_CONTROL_SETTLE_FRAMES,
    FORCE_CONTROL_TARGET_N,
)


TARGET_TOPIC = "/grasp_validation/target"
STATUS_TOPIC = "/grasp_validation/status"
DEFAULT_HOLD_SEC = 2.5
DEFAULT_VALIDATION_TARGET = "material_spare_igbt"
MATERIAL_SPARE_INSERT_SAFE_Z = 0.615
MATERIAL_SPARE_INSERT_DISTANCE = 0.015
CARTESIAN_SEGMENT_MIN_FRACTION = 0.95
GAZEBO_MODEL_STATES_TOPIC = "/gazebo/model_states"
GAZEBO_ENTITY_STATE_SERVICE = "/gazebo/get_entity_state"
GAZEBO_MODEL_PROPERTIES_SERVICE = "/gazebo/get_model_properties"
MOVEIT_STATE_VALIDITY_SERVICE = "/check_state_validity"
ROBOT_ENTITY_NAME = "lab_cobot"
BASE_FOOTPRINT_FRAME = "base_footprint"
BASE_FOOTPRINT_ENTITY = f"{ROBOT_ENTITY_NAME}::{BASE_FOOTPRINT_FRAME}"
BASE_LINK_FRAME = "base_link"
GRIPPER_LEFT_FINGER_LINK = "gripper_left_finger"
GRIPPER_RIGHT_FINGER_LINK = "gripper_right_finger"
CONSISTENCY_DIAG_FRAMES = (
    "base_footprint",
    "base_link",
    "ur_base_link",
    "ur_tool0",
    GRIPPER_TCP_LINK,
    GRIPPER_LEFT_FINGER_LINK,
    GRIPPER_RIGHT_FINGER_LINK,
)
UR_ARM_JOINTS = (
    "ur_shoulder_pan_joint",
    "ur_shoulder_lift_joint",
    "ur_elbow_joint",
    "ur_wrist_1_joint",
    "ur_wrist_2_joint",
    "ur_wrist_3_joint",
)
DESCEND_CARTESIAN_JUMP_THRESHOLD = 1.5
DESCEND_MAX_JOINT_DELTA_PER_STEP = 0.20
DESCEND_MAX_TOTAL_JOINT_DELTA = {
    "ur_shoulder_pan_joint": 0.50,
    "ur_shoulder_lift_joint": 0.90,
    "ur_elbow_joint": 1.20,
    "ur_wrist_1_joint": 1.20,
    "ur_wrist_2_joint": 0.50,
    "ur_wrist_3_joint": 0.50,
}
VALIDATION_PRE_GRASP_BRANCH_CONSTRAINT_JOINTS = (
    "ur_wrist_1_joint",
    "ur_wrist_2_joint",
    "ur_wrist_3_joint",
)
VALIDATION_PRE_GRASP_BRANCH_CONSTRAINT_TOLERANCES = [2.40, 2.40, 1.60]
LIFT_STATE_VERIFIED_MAX_POSITION_ERROR_M = 0.003
LIFT_STATE_VERIFIED_MAX_Z_ERROR_M = 0.0015

SUCCESS_STATUSES = (
    "READY",
    "TARGET_CONFIGURED",
    "PLANNING",
    "PRE_GRASP_REACHED",
    "DESCENDING",
    "GRIPPING",
    "CONTACT_OK",
    "ATTACHED",
    "LIFTING",
    "HOLDING",
    "SUCCESS",
)
FAILURE_STATUSES = (
    "FAILED_UNKNOWN_TARGET",
    "FAILED_TARGET_CONFIG",
    "FAILED_TARGET_POSE",
    "FAILED_TRANSFORM",
    "FAILED_PRE_GRASP_PLAN",
    "FAILED_PRE_GRASP_EXEC",
    "FAILED_DESCEND",
    "FAILED_CONTACT",
    "FAILED_ATTACH",
    "FAILED_LIFT",
    "FAILED_HOLD_LOST",
    "FAILED_EXCEPTION",
)


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * x - sin_yaw * y,
        sin_yaw * x + cos_yaw * y,
    )


def _rotate_xyz_rpy(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return (
        (cy * cp) * x
        + (cy * sp * sr - sy * cr) * y
        + (cy * sp * cr + sy * sr) * z,
        (sy * cp) * x
        + (sy * sp * sr + cy * cr) * y
        + (sy * sp * cr - cy * sr) * z,
        (-sp) * x + (cp * sr) * y + (cp * cr) * z,
    )


def world_to_base_xy(
    world_x: float,
    world_y: float,
    base_x: float,
    base_y: float,
    base_yaw: float,
) -> tuple[float, float]:
    """Transform a world/map XY point into the validation base_link frame."""
    dx = float(world_x) - float(base_x)
    dy = float(world_y) - float(base_y)
    cos_yaw = math.cos(base_yaw)
    sin_yaw = math.sin(base_yaw)
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
    )


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def rpy_to_quaternion(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    half_roll = 0.5 * float(roll)
    half_pitch = 0.5 * float(pitch)
    half_yaw = 0.5 * float(yaw)
    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_multiply(
    q1: tuple[float, float, float, float],
    q2: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def pose_msg_to_world_pose(pose) -> dict:
    roll, pitch, yaw = quaternion_to_rpy(
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    return {
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "z": float(pose.position.z),
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }


def configured_grasp_world_pose(config: dict, object_world_pose: dict | None = None) -> dict:
    world_pose = object_world_pose or config["world_pose"]
    offset = config["grasp_offset"]
    z_adjust = float(config.get("grasp_z_adjust", 0.0))
    offset_x, offset_y, offset_z = _rotate_xyz_rpy(
        float(offset["x"]),
        float(offset["y"]),
        float(offset["z"]),
        float(world_pose.get("roll", 0.0)),
        float(world_pose.get("pitch", 0.0)),
        float(world_pose["yaw"]),
    )
    return {
        "x": float(world_pose["x"]) + offset_x,
        "y": float(world_pose["y"]) + offset_y,
        "z": float(world_pose["z"]) + offset_z + z_adjust,
        "yaw": float(world_pose["yaw"]) + float(config.get("grasp_yaw", 0.0)),
    }


def configured_handle_local_point(config: dict) -> dict:
    point = config.get(
        "handle_grasp_local_point",
        config.get("grasp_point_local", config["grasp_offset"]),
    )
    return {
        "x": float(point.get("x", 0.0)),
        "y": float(point.get("y", 0.0)),
        "z": float(point.get("z", 0.0)),
    }


def handle_long_axis_world_yaw(
    config: dict,
    object_world_pose: dict | None = None,
) -> float:
    world_pose = object_world_pose or config["world_pose"]
    local_axis_yaw = float(
        config.get(
            "handle_long_axis_yaw",
            config.get("grasp_long_axis_yaw", 0.0),
        )
    )
    yaw = float(world_pose["yaw"]) + local_axis_yaw
    return math.atan2(math.sin(yaw), math.cos(yaw))


def board_base_block_local_center(config: dict) -> dict | None:
    center = config.get("base_block_local_center")
    if center is None:
        return None
    return {
        "x": float(center.get("x", 0.0)),
        "y": float(center.get("y", 0.0)),
        "z": float(center.get("z", 0.0)),
    }


def board_base_block_world_center(
    config: dict,
    object_world_pose: dict | None = None,
) -> dict | None:
    center = board_base_block_local_center(config)
    if center is None:
        return None
    world_pose = object_world_pose or config["world_pose"]
    x, y, z = _rotate_xyz_rpy(
        center["x"],
        center["y"],
        center["z"],
        float(world_pose.get("roll", 0.0)),
        float(world_pose.get("pitch", 0.0)),
        float(world_pose["yaw"]),
    )
    return {
        "x": float(world_pose["x"]) + x,
        "y": float(world_pose["y"]) + y,
        "z": float(world_pose["z"]) + z,
    }


def board_base_block_long_axis_world_yaw(
    config: dict,
    object_world_pose: dict | None = None,
) -> float | None:
    if "base_block_long_axis_yaw" not in config:
        return None
    world_pose = object_world_pose or config["world_pose"]
    yaw = float(world_pose["yaw"]) + float(config["base_block_long_axis_yaw"])
    return math.atan2(math.sin(yaw), math.cos(yaw))


def validation_base_pose_for_target_base_pose(
    grasp_world_pose: dict,
    target_base_x: float,
    target_base_y: float,
    base_yaw: float,
    base_z: float = 0.155,
) -> dict:
    dx_world, dy_world = _rotate_xy(
        float(target_base_x),
        float(target_base_y),
        float(base_yaw),
    )
    return {
        "x": float(grasp_world_pose["x"]) - dx_world,
        "y": float(grasp_world_pose["y"]) - dy_world,
        "z": float(base_z),
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": float(base_yaw),
    }


def _configured_local_point_world(
    point: dict,
    config: dict,
    object_world_pose: dict | None = None,
) -> dict:
    world_pose = object_world_pose or config["world_pose"]
    x, y, z = _rotate_xyz_rpy(
        float(point.get("x", 0.0)),
        float(point.get("y", 0.0)),
        float(point.get("z", 0.0)),
        float(world_pose.get("roll", 0.0)),
        float(world_pose.get("pitch", 0.0)),
        float(world_pose["yaw"]),
    )
    return {
        "x": float(world_pose["x"]) + x,
        "y": float(world_pose["y"]) + y,
        "z": float(world_pose["z"]) + z,
    }


def configured_candidate_world_point(
    candidate: dict,
    config: dict,
    object_world_pose: dict | None = None,
    point_key: str = "local_center",
) -> dict:
    return _configured_local_point_world(
        candidate[point_key],
        config,
        object_world_pose,
    )


def configured_candidate_world_yaw(
    candidate: dict,
    config: dict,
    object_world_pose: dict | None = None,
    yaw_key: str = "grasp_yaw",
) -> float:
    world_pose = object_world_pose or config["world_pose"]
    yaw = float(world_pose["yaw"]) + float(candidate.get(yaw_key, 0.0))
    return math.atan2(math.sin(yaw), math.cos(yaw))


def world_pose_to_base_pose(world_pose: dict, base_world_pose: dict) -> dict:
    x_base, y_base = world_to_base_xy(
        float(world_pose["x"]),
        float(world_pose["y"]),
        float(base_world_pose["x"]),
        float(base_world_pose["y"]),
        float(base_world_pose["yaw"]),
    )
    yaw = float(world_pose.get("yaw", 0.0)) - float(base_world_pose["yaw"])
    return {
        "x": x_base,
        "y": y_base,
        "z": float(world_pose["z"]) - float(base_world_pose["z"]),
        "yaw": math.atan2(math.sin(yaw), math.cos(yaw)),
    }


def compose_child_world_pose(parent_world_pose: dict, child_in_parent_pose: dict) -> dict:
    """Compose a child pose under a world planar pose.

    Validation only needs the robot ground pose plus the fixed
    base_footprint->base_link transform.  Roll/pitch are preserved for logs;
    x/y/yaw use the full planar rigid transform instead of direct subtraction.
    """
    parent_yaw = float(parent_world_pose.get("yaw", 0.0))
    child_x, child_y = _rotate_xy(
        float(child_in_parent_pose.get("x", 0.0)),
        float(child_in_parent_pose.get("y", 0.0)),
        parent_yaw,
    )
    yaw = parent_yaw + float(child_in_parent_pose.get("yaw", 0.0))
    return {
        "x": float(parent_world_pose["x"]) + child_x,
        "y": float(parent_world_pose["y"]) + child_y,
        "z": float(parent_world_pose["z"])
        + float(child_in_parent_pose.get("z", 0.0)),
        "roll": float(parent_world_pose.get("roll", 0.0))
        + float(child_in_parent_pose.get("roll", 0.0)),
        "pitch": float(parent_world_pose.get("pitch", 0.0))
        + float(child_in_parent_pose.get("pitch", 0.0)),
        "yaw": math.atan2(math.sin(yaw), math.cos(yaw)),
    }


def configured_grasp_base_pose(
    config: dict,
    base_world_pose: dict | None = None,
    object_world_pose: dict | None = None,
) -> list[float]:
    grasp_world = configured_grasp_world_pose(config, object_world_pose)
    if base_world_pose is not None:
        grasp_base = world_pose_to_base_pose(grasp_world, base_world_pose)
        return [grasp_base["x"], grasp_base["y"], grasp_base["z"]]

    base_pose = config["validation_base_pose"]
    x_base, y_base = world_to_base_xy(
        grasp_world["x"],
        grasp_world["y"],
        float(base_pose["x"]),
        float(base_pose["y"]),
        float(base_pose["yaw"]),
    )
    return [x_base, y_base, float(grasp_world["z"])]


def grasp_z_debug_values(config: dict, tcp_target_z: float) -> dict[str, float]:
    world_pose = config["world_pose"]
    collision_local = config.get("collision_local_pose", {})
    collision_size = config["collision_size"]
    model_origin_z = float(world_pose["z"])
    collision_local_z = float(
        collision_local.get("z", config["grasp_offset"]["z"])
    )
    object_center_z = model_origin_z + collision_local_z
    half_height = 0.5 * float(collision_size[2])
    return {
        "MODEL_ORIGIN_Z": model_origin_z,
        "COLLISION_LOCAL_Z": collision_local_z,
        "OBJECT_CENTER_Z": object_center_z,
        "OBJECT_TOP_Z": object_center_z + half_height,
        "OBJECT_BOTTOM_Z": object_center_z - half_height,
        "OBJECT_COLLISION_CENTER_WORLD_Z": object_center_z,
        "OBJECT_COLLISION_TOP_WORLD_Z": object_center_z + half_height,
        "OBJECT_COLLISION_BOTTOM_WORLD_Z": object_center_z - half_height,
        "EXPECTED_GRASP_Z": object_center_z,
        "TCP_CLEARANCE_Z": float(config["tcp_clearance"]),
        "TCP_TARGET_Z": float(tcp_target_z),
    }


def configured_grasp_base_yaw(
    config: dict,
    base_world_pose: dict | None = None,
    object_world_pose: dict | None = None,
) -> float:
    grasp_world = configured_grasp_world_pose(config, object_world_pose)
    base_yaw = (
        float(base_world_pose["yaw"])
        if base_world_pose is not None
        else float(config["validation_base_pose"]["yaw"])
    )
    yaw = float(grasp_world["yaw"]) - base_yaw
    return math.atan2(math.sin(yaw), math.cos(yaw))


def down_quat_for_yaw(yaw: float) -> list[float]:
    half_yaw = 0.5 * float(yaw)
    return [math.cos(half_yaw), math.sin(half_yaw), 0.0, 0.0]


def configured_grasp_rpy(config: dict, grasp_base_yaw: float) -> tuple[float, float, float]:
    fixed = config.get("fixed_grasp_rpy")
    if fixed:
        return (
            float(fixed.get("roll", math.pi)),
            float(fixed.get("pitch", 0.0)),
            float(fixed.get("yaw", grasp_base_yaw)),
        )
    return (math.pi, 0.0, float(grasp_base_yaw))


def configured_grasp_quat_and_yaw(
    config: dict,
    grasp_base_yaw: float,
) -> tuple[list[float], float, tuple[float, float, float]]:
    roll, pitch, yaw = configured_grasp_rpy(config, grasp_base_yaw)
    if config.get("fixed_grasp_rpy"):
        quat = list(rpy_to_quaternion(roll, pitch, yaw))
        return quat, yaw, (roll, pitch, yaw)
    return down_quat_for_yaw(grasp_base_yaw), grasp_base_yaw, (roll, pitch, yaw)


def validate_target_config(target: str, config: dict) -> None:
    required = {
        "station",
        "entity_name",
        "model_uri",
        "link_name",
        "world_pose",
        "validation_base_pose",
        "grasp_offset",
        "grasp_yaw",
        "pre_grasp_height",
        "tcp_clearance",
        "descend_distance",
        "lift_distance",
        "collision_size",
        "carried_collision_size",
        "attach_envelope",
    }
    missing = sorted(required - set(config))
    if missing:
        raise ValueError(f"{target}: missing fields {missing}")
    if config["entity_name"] != target:
        raise ValueError(f"{target}: entity_name must match target")
    for key in ("world_pose", "validation_base_pose"):
        for field in ("x", "y", "yaw"):
            if field not in config[key]:
                raise ValueError(f"{target}: {key}.{field} missing")
    if "z" not in config["world_pose"]:
        raise ValueError(f"{target}: world_pose.z missing")


class GraspValidationNode(Node):
    """Runs a single-target grasp/lift/hold validation outside mission_node."""

    def __init__(self, pick_place: PickPlace | None = None):
        super().__init__("grasp_validation_node")
        self.declare_parameter("validation_target", DEFAULT_VALIDATION_TARGET)
        self.declare_parameter("hold_duration_sec", DEFAULT_HOLD_SEC)
        self.declare_parameter("use_tactile_grasp", True)
        self.declare_parameter("use_planning_scene_obstacles", True)
        self.declare_parameter("enable_force_gate", False)
        self.declare_parameter("enable_force_control", False)
        self.declare_parameter("force_target_n", FORCE_CONTROL_TARGET_N)
        self.declare_parameter("force_deadband_n", FORCE_CONTROL_DEADBAND_N)
        self.declare_parameter("force_kp", FORCE_CONTROL_KP)
        self.declare_parameter("force_max_close_step", FORCE_CONTROL_MAX_CLOSE_STEP)
        self.declare_parameter("force_max_open_step", FORCE_CONTROL_MAX_OPEN_STEP)
        self.declare_parameter("force_safety_limit_n", FORCE_CONTROL_SAFETY_LIMIT_N)
        self.declare_parameter("force_safety_frames", FORCE_CONTROL_SAFETY_FRAMES)
        self.declare_parameter("force_balance_limit_n", FORCE_CONTROL_BALANCE_LIMIT_N)
        self.declare_parameter("force_balance_frames", FORCE_CONTROL_BALANCE_FRAMES)
        self.declare_parameter("force_settle_frames", FORCE_CONTROL_SETTLE_FRAMES)
        self.declare_parameter("force_filter_window", FORCE_CONTROL_FILTER_WINDOW)
        self.declare_parameter("material_spare_descend_mode", "horizontal_insert")
        self.declare_parameter("fixture_grasp_point_y_override", float("nan"))
        self.validation_target = str(
            self.get_parameter("validation_target").value
        ).strip()
        self.hold_duration_sec = float(
            self.get_parameter("hold_duration_sec").value
        )
        use_tactile = bool(self.get_parameter("use_tactile_grasp").value)
        use_scene = bool(
            self.get_parameter("use_planning_scene_obstacles").value
        )
        self.enable_force_gate = bool(self.get_parameter("enable_force_gate").value)
        self.enable_force_control = bool(
            self.get_parameter("enable_force_control").value
        )
        self.material_spare_descend_mode = str(
            self.get_parameter("material_spare_descend_mode").value
        ).strip()
        fixture_y_override = float(
            self.get_parameter("fixture_grasp_point_y_override").value
        )
        self.fixture_grasp_point_y_override = (
            fixture_y_override if math.isfinite(fixture_y_override) else None
        )
        target_config = get_target_config(self.validation_target) or {}
        pre_grasp_position = target_config.get("pre_grasp_position")
        gripper_open_positions = (
            None
            if pre_grasp_position is None
            else [float(pre_grasp_position), float(pre_grasp_position)]
        )
        target_pick_tcp_z_clearance = (
            float(target_config["tcp_clearance"])
            if self.validation_target == "material_spare_igbt"
            else None
        )
        self.pp = pick_place or PickPlace(
            target_object=self.validation_target,
            use_tactile_grasp=use_tactile,
            use_planning_scene_obstacles=use_scene,
            tactile_start_position=target_config.get("tactile_start_position"),
            tactile_max_position=target_config.get("tactile_max_position"),
            gripper_open_positions=gripper_open_positions,
            expected_object_width_mm=target_config.get("expected_grasp_width_mm"),
            pick_tcp_z_clearance=target_pick_tcp_z_clearance,
            tactile_target_force_n=target_config.get("tactile_target_force_n"),
            tactile_max_force_n=target_config.get("tactile_max_force_n"),
            enable_force_gate=self.enable_force_gate,
            enable_force_control=self.enable_force_control,
            force_target_n=float(self.get_parameter("force_target_n").value),
            force_deadband_n=float(self.get_parameter("force_deadband_n").value),
            force_kp=float(self.get_parameter("force_kp").value),
            force_max_close_step=float(
                self.get_parameter("force_max_close_step").value
            ),
            force_max_open_step=float(
                self.get_parameter("force_max_open_step").value
            ),
            force_safety_limit_n=float(
                self.get_parameter("force_safety_limit_n").value
            ),
            force_safety_frames=int(
                self.get_parameter("force_safety_frames").value
            ),
            force_balance_limit_n=float(
                self.get_parameter("force_balance_limit_n").value
            ),
            force_balance_frames=int(
                self.get_parameter("force_balance_frames").value
            ),
            force_settle_frames=int(
                self.get_parameter("force_settle_frames").value
            ),
            force_filter_window=int(
                self.get_parameter("force_filter_window").value
            ),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._entity_state_client = self.create_client(
            GetEntityState,
            GAZEBO_ENTITY_STATE_SERVICE,
        )
        self._model_properties_client = self.create_client(
            GetModelProperties,
            GAZEBO_MODEL_PROPERTIES_SERVICE,
        )
        self._state_validity_client = self.create_client(
            GetStateValidity,
            MOVEIT_STATE_VALIDITY_SERVICE,
        )
        self._latest_model_states: ModelStates | None = None
        self._busy = False
        self._lock = Lock()
        self.status_pub = self.create_publisher(
            String,
            STATUS_TOPIC,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            ModelStates,
            GAZEBO_MODEL_STATES_TOPIC,
            self._on_model_states,
            10,
        )
        self.create_subscription(String, TARGET_TOPIC, self._on_target, 10)
        self.get_logger().info(
            "grasp_validation_node ready on %s" % TARGET_TOPIC
        )

    def _on_model_states(self, msg: ModelStates) -> None:
        self._latest_model_states = msg

    def _publish_status(self, status: str, target: str) -> None:
        msg = String()
        msg.data = f"{status} {target}".strip()
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _on_target(self, msg: String) -> None:
        target = str(msg.data).strip()
        with self._lock:
            if self._busy:
                self._publish_status("BUSY", target)
                return
            self._busy = True
        Thread(target=self._run_validation, args=(target,), daemon=True).start()

    def _run_validation(self, target: str) -> None:
        try:
            if not self._validate_target_matches_launch(target):
                return
            config = get_target_config(target)
            if config is None:
                self._publish_status("FAILED_UNKNOWN_TARGET", target)
                return
            config = self._validation_config_for_trial(target, config)
            try:
                validate_target_config(target, config)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(str(exc))
                self._publish_status("FAILED_TARGET_CONFIG", target)
                return

            self._publish_status("READY", target)
            self._publish_status("TARGET_CONFIGURED", target)
            if not self._execute_configured_grasp(target, config):
                return
            self._publish_status("SUCCESS", target)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"grasp validation exception: {exc}")
            self._publish_status("FAILED_EXCEPTION", target)
        finally:
            with self._lock:
                self._busy = False

    def _validate_target_matches_launch(self, target: str) -> bool:
        if target != self.validation_target:
            self.get_logger().error(
                "requested target %r does not match validation_target %r"
                % (target, self.validation_target)
            )
            self._publish_status("FAILED_TARGET_CONFIG", target)
            return False
        return True

    def _validation_config_for_trial(self, target: str, config: dict) -> dict:
        trial_config = copy.deepcopy(config)
        if (
            target == "tooling_fixture_box"
            and self.fixture_grasp_point_y_override is not None
        ):
            for key in ("grasp_point_local", "grasp_offset"):
                trial_config[key]["y"] = self.fixture_grasp_point_y_override
            self.get_logger().info(
                "FIXTURE_GRASP_POINT_Y_OVERRIDE %s y=%.6f"
                % (target, self.fixture_grasp_point_y_override)
            )
        return trial_config

    def _execute_configured_grasp(self, target: str, config: dict) -> bool:
        try:
            object_world_pose = self._get_object_world_pose(target, config)
            base_world_pose = self._get_base_link_world_pose(config)
            grasp_world = configured_grasp_world_pose(config, object_world_pose)
            grasp_base = configured_grasp_base_pose(
                config,
                base_world_pose=base_world_pose,
                object_world_pose=object_world_pose,
            )
            grasp_base_yaw = configured_grasp_base_yaw(
                config,
                base_world_pose=base_world_pose,
                object_world_pose=object_world_pose,
            )
            if target == "material_spare_igbt":
                # Material spare validation must use its own object-derived
                # yaw.  Ignore any fixed-RPY compatibility override in target
                # config so this path cannot inherit another object's yaw.
                grasp_quat = down_quat_for_yaw(grasp_base_yaw)
                final_grasp_yaw = grasp_base_yaw
                final_grasp_rpy = (math.pi, 0.0, grasp_base_yaw)
            else:
                grasp_quat, final_grasp_yaw, final_grasp_rpy = (
                    configured_grasp_quat_and_yaw(config, grasp_base_yaw)
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"target pose transform failed: {exc}")
            self._publish_status("FAILED_TRANSFORM", target)
            return False
        if len(grasp_base) != 3 or any(not math.isfinite(v) for v in grasp_base):
            self._publish_status("FAILED_TARGET_POSE", target)
            return False

        # Match the production A->B pick semantic:
        # grasp_base is an object grasp point in base_link, not a TCP pose.
        # PickPlace converts it to a gripper_tcp target; MoveIt/TF handles the
        # fixed base_link->ur_base_link and gripper_tcp->ur_tool0 transforms.
        tcp_target = self.pp._pick_tcp_target(grasp_base)  # noqa: SLF001
        base_tcp_target = list(tcp_target)
        tcp_z_adjust = float(config.get("validation_grasp_tcp_z_adjust", 0.0))
        if abs(tcp_z_adjust) > 1e-12:
            tcp_target[2] += tcp_z_adjust
            if target == "high_voltage_probe_kit":
                self.get_logger().info(
                    "HIGH_VOLTAGE_GRASP_Z_OFFSET "
                    "old_z=%.4f new_z=%.4f delta_z=%.4f"
                    % (base_tcp_target[2], tcp_target[2], tcp_z_adjust)
                )
        self._log_grasp_z_geometry(target, config, tcp_target[2])
        descend_distance = max(
            float(config["descend_distance"]),
            float(config["pre_grasp_height"]),
        )
        if (
            abs(tcp_z_adjust) > 1e-12
            and bool(config.get("preserve_pre_grasp_z_on_tcp_z_adjust", False))
        ):
            descend_distance -= tcp_z_adjust
        self.pp.approach_height = descend_distance
        pre_grasp = self.pp._pick_approach_target(tcp_target)  # noqa: SLF001
        self._log_validation_pick_targets(target, grasp_base, tcp_target, pre_grasp)
        self._log_world_base_target_geometry(
            target,
            config,
            object_world_pose,
            base_world_pose,
            grasp_world,
            grasp_base,
            tcp_target,
            pre_grasp,
            grasp_quat,
            final_grasp_yaw,
            final_grasp_rpy,
        )
        lift = [
            tcp_target[0],
            tcp_target[1],
            tcp_target[2] + float(config["lift_distance"]),
        ]

        if not self.pp.gripper.open():
            self._publish_status("FAILED_CONTACT", target)
            return False

        self._publish_status("PLANNING", target)
        branch_constraints_set = self._apply_validation_pre_grasp_branch_constraints(target)
        try:
            pre_grasp_ok = self.pp._move_approach(  # noqa: SLF001
                pre_grasp,
                quat=grasp_quat,
                cartesian=False,
                local_speed=False,
                fallback_to_ompl=True,
                tolerance_orientation=DEFAULT_APPROACH_TOLERANCE_ORIENTATION,
                stabilize_wrist=False if branch_constraints_set else True,
            )
        finally:
            if branch_constraints_set:
                self._clear_validation_branch_constraints(target)
        if not pre_grasp_ok:
            self._publish_status("FAILED_PRE_GRASP_PLAN", target)
            return False
        self._publish_status("PRE_GRASP_REACHED", target)
        self._log_current_arm_joints("PRE_GRASP_JOINTS", target)
        if target == "board_test_fixture":
            self._log_coordinate_consistency_snapshot(
                "PRE_GRASP_COORDINATE_CONSISTENCY",
                target,
                config,
                base_world_pose,
                joint_state=getattr(self.pp.moveit2, "joint_state", None),
            )

        self.pp._inject_station_surface(grasp_base)  # noqa: SLF001
        self._log_descend_tcp_poses(target, pre_grasp, tcp_target, grasp_quat)
        self._publish_status("DESCENDING", target)
        if (
            target == "material_spare_igbt"
            and self.material_spare_descend_mode == "horizontal_insert"
        ):
            if not self._validation_descend_with_horizontal_insert(
                target,
                pre_grasp,
                tcp_target,
                grasp_quat,
                final_grasp_yaw,
            ):
                self._log_current_arm_joints("DESCEND_ABORT_JOINTS", target)
                self._publish_status("FAILED_DESCEND", target)
                return False
        else:
            if target == "material_spare_igbt":
                self.get_logger().info(
                    "MATERIAL_SPARE_DESCEND_MODE %s mode=cartesian" % target
                )
            descend_trajectory = self._plan_validation_cartesian_descend(
                target,
                tcp_target,
                grasp_quat,
                config,
                base_world_pose,
            )
            if descend_trajectory is None:
                self._publish_status("FAILED_DESCEND", target)
                return False
            if not self._execute_validation_descend(descend_trajectory):
                self._log_current_arm_joints("DESCEND_ABORT_JOINTS", target)
                self._publish_status("FAILED_DESCEND", target)
                return False
        if PRE_GRASP_SETTLE_SEC > 0.0:
            time.sleep(PRE_GRASP_SETTLE_SEC)
        self._log_current_arm_joints("GRASP_REACHED_JOINTS", target)
        self._log_gripper_world_poses_at_grasp(target, base_world_pose)

        self._publish_status("GRIPPING", target)
        if not self.pp.gripper.acquire_object():
            left, right = self.pp.gripper.last_tactile_contact_sides()
            failure = "FAILED_ATTACH" if left and right else "FAILED_CONTACT"
            self._publish_status(failure, target)
            return False
        self._publish_status("CONTACT_OK", target)

        if not self.pp._wait_until_gripper_holding():  # noqa: SLF001
            self._publish_status("FAILED_ATTACH", target)
            return False
        self._publish_status("ATTACHED", target)

        self.pp._attach_carried_sample()  # noqa: SLF001
        self.pp._start_hold_monitor()  # noqa: SLF001
        if not self.pp._holding_is_healthy():  # noqa: SLF001
            self._publish_status("FAILED_HOLD_LOST", target)
            return False
        if POST_GRASP_SETTLE_SEC > 0.0:
            time.sleep(POST_GRASP_SETTLE_SEC)

        self._publish_status("LIFTING", target)
        if not self._execute_validation_lift(
            target,
            lift,
            self.pp._current_tcp_quat(),  # noqa: SLF001
        ):
            return False

        self._publish_status("HOLDING", target)
        deadline = time.monotonic() + max(self.hold_duration_sec, 0.0)
        while time.monotonic() < deadline:
            if not self.pp._holding_is_healthy():  # noqa: SLF001
                self._publish_status("FAILED_HOLD_LOST", target)
                return False
            time.sleep(0.1)
        return True

    def _log_descend_tcp_poses(
        self,
        target: str,
        pre_grasp: list[float],
        tcp_target: list[float],
        grasp_quat: list[float],
    ) -> None:
        self.get_logger().info(
            "DESCEND_START_TCP_POSE %s x=%.4f y=%.4f z=%.4f "
            "qx=%.6f qy=%.6f qz=%.6f qw=%.6f"
            % (
                target,
                pre_grasp[0],
                pre_grasp[1],
                pre_grasp[2],
                grasp_quat[0],
                grasp_quat[1],
                grasp_quat[2],
                grasp_quat[3],
            )
        )
        self.get_logger().info(
            "DESCEND_END_TCP_POSE %s x=%.4f y=%.4f z=%.4f "
            "qx=%.6f qy=%.6f qz=%.6f qw=%.6f"
            % (
                target,
                tcp_target[0],
                tcp_target[1],
                tcp_target[2],
                grasp_quat[0],
                grasp_quat[1],
                grasp_quat[2],
                grasp_quat[3],
            )
        )

    def _get_object_world_pose(self, target: str, config: dict) -> dict:
        link_entity = f"{config['entity_name']}::{config['link_name']}"
        pose = self._get_entity_world_pose(link_entity)
        if pose is not None:
            return pose
        pose = self._get_entity_world_pose(config["entity_name"])
        if pose is not None:
            self.get_logger().warn(
                "using model pose fallback for %s because %s was unavailable"
                % (target, link_entity)
            )
            return pose
        cached = self._model_state_pose(config["entity_name"])
        if cached is not None:
            self.get_logger().warn(
                "using /gazebo/model_states fallback for %s" % target
            )
            return cached
        self.get_logger().warn(
            "using configured world pose fallback for %s; Gazebo state unavailable"
            % target
        )
        return {
            "x": float(config["world_pose"]["x"]),
            "y": float(config["world_pose"]["y"]),
            "z": float(config["world_pose"]["z"]),
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": float(config["world_pose"]["yaw"]),
        }

    def _get_base_link_world_pose(self, config: dict) -> dict:
        base_footprint_pose = self._get_base_footprint_world_pose(config)
        base_link_in_footprint = self._get_base_link_pose_in_base_footprint()
        return compose_child_world_pose(
            base_footprint_pose,
            base_link_in_footprint,
        )

    def _get_base_footprint_world_pose(self, config: dict) -> dict:
        pose = self._get_entity_world_pose(BASE_FOOTPRINT_ENTITY)
        if pose is not None:
            return pose
        cached = self._model_state_pose(ROBOT_ENTITY_NAME)
        if cached is not None:
            self.get_logger().warn(
                "using /gazebo/model_states pose for %s as %s pose; "
                "base_link will be derived through TF"
                % (ROBOT_ENTITY_NAME, BASE_FOOTPRINT_FRAME)
            )
            return cached
        self.get_logger().warn(
            "using validation_base_pose fallback as base_footprint pose; "
            "Gazebo base_footprint/model state unavailable"
        )
        validation_base = config["validation_base_pose"]
        return {
            "x": float(validation_base["x"]),
            "y": float(validation_base["y"]),
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": float(validation_base["yaw"]),
        }

    def _get_base_link_pose_in_base_footprint(self) -> dict:
        try:
            transform = self._tf_buffer.lookup_transform(
                BASE_FOOTPRINT_FRAME,
                BASE_LINK_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                "TF lookup failed %s <- %s: %s; using URDF nominal "
                "base_footprint->base_link z=0.155 fallback"
                % (BASE_FOOTPRINT_FRAME, BASE_LINK_FRAME, exc)
            )
            return {
                "x": 0.0,
                "y": 0.0,
                "z": 0.155,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        roll, pitch, yaw = quaternion_to_rpy(
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )
        return {
            "x": float(translation.x),
            "y": float(translation.y),
            "z": float(translation.z),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }

    def _get_entity_world_pose(self, entity_name: str) -> dict | None:
        if not self._entity_state_client.wait_for_service(timeout_sec=0.2):
            return None
        request = GetEntityState.Request()
        request.name = entity_name
        request.reference_frame = "world"
        future = self._entity_state_client.call_async(request)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if future.done():
                result = future.result()
                if result is not None and result.success:
                    return pose_msg_to_world_pose(result.state.pose)
                return None
            time.sleep(0.01)
        self.get_logger().warn(
            "timeout waiting for %s entity state" % entity_name
        )
        return None

    def _model_state_pose(self, entity_name: str) -> dict | None:
        msg = self._latest_model_states
        if msg is None or entity_name not in msg.name:
            return None
        index = msg.name.index(entity_name)
        return pose_msg_to_world_pose(msg.pose[index])

    def _log_world_base_target_geometry(
        self,
        target: str,
        config: dict,
        object_world_pose: dict,
        base_world_pose: dict,
        grasp_world: dict,
        grasp_base: list[float],
        tcp_target: list[float],
        pre_grasp: list[float],
        grasp_quat: list[float],
        grasp_base_yaw: float,
        grasp_rpy: tuple[float, float, float],
    ) -> None:
        self.get_logger().info(
            "OBJECT_WORLD_POSE %s %s"
            % (target, self._format_pose_dict(object_world_pose))
        )
        handle_local = configured_handle_local_point(config)
        self.get_logger().info(
            "HANDLE_GRASP_LOCAL_POINT %s x=%.4f y=%.4f z=%.4f"
            % (
                target,
                handle_local["x"],
                handle_local["y"],
                handle_local["z"],
            )
        )
        self.get_logger().info(
            "HANDLE_GRASP_WORLD_POINT %s x=%.4f y=%.4f z=%.4f"
            % (
                target,
                grasp_world["x"],
                grasp_world["y"],
                grasp_world["z"],
            )
        )
        self.get_logger().info(
            "HANDLE_LONG_AXIS_WORLD_YAW %s yaw=%.4f"
            % (target, handle_long_axis_world_yaw(config, object_world_pose))
        )
        if "grasp_region_label" in config or "grasp_point_local" in config:
            region_point = config.get("grasp_point_local", config["grasp_offset"])
            region_size = config.get("grasp_region_size", (0.0, 0.0, 0.0))
            region_axis_yaw = handle_long_axis_world_yaw(config, object_world_pose)
            self.get_logger().info(
                "GRASP_REGION_LABEL %s %s"
                % (target, config.get("grasp_region_label", "configured_region"))
            )
            self.get_logger().info(
                "GRASP_LOCAL_POINT %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    float(region_point.get("x", 0.0)),
                    float(region_point.get("y", 0.0)),
                    float(region_point.get("z", 0.0)),
                )
            )
            self.get_logger().info(
                "GRASP_REGION_SIZE %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    float(region_size[0]),
                    float(region_size[1]),
                    float(region_size[2]),
                )
            )
            self.get_logger().info(
                "GRASP_LONG_AXIS_WORLD_YAW %s yaw=%.4f"
                % (target, region_axis_yaw)
            )
            self.get_logger().info(
                "GRASP_WORLD_POINT %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    grasp_world["x"],
                    grasp_world["y"],
                    grasp_world["z"],
                )
            )
            self.get_logger().info(
                "GRASP_LOCAL_POINT_FINAL %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    float(region_point.get("x", 0.0)),
                    float(region_point.get("y", 0.0)),
                    float(region_point.get("z", 0.0)),
                )
            )
            self.get_logger().info(
                "GRASP_WORLD_POINT_FINAL %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    grasp_world["x"],
                    grasp_world["y"],
                    grasp_world["z"],
                )
            )
            self.get_logger().info(
                "GRASP_WORLD_POINT %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    grasp_world["x"],
                    grasp_world["y"],
                    grasp_world["z"],
                )
            )
            self.get_logger().info(
                "GRASP_BASE_POINT %s x=%.4f y=%.4f z=%.4f"
                % (target, grasp_base[0], grasp_base[1], grasp_base[2])
            )
            self.get_logger().info(
                "TCP_TARGET_Z %s z=%.4f" % (target, tcp_target[2])
            )
            if target == "material_spare_igbt":
                before_point = {
                    "x": 0.00194070,
                    "y": 0.05511090,
                    "z": -0.01482813,
                }
                before_world = _configured_local_point_world(
                    before_point,
                    config,
                    object_world_pose,
                )
                before_world["z"] += float(config.get("grasp_z_adjust", 0.0))
                before_base = world_pose_to_base_pose(before_world, base_world_pose)
                self.get_logger().info(
                    "GRASP_WORLD_POINT_BEFORE %s x=%.4f y=%.4f z=%.4f"
                    % (
                        target,
                        before_world["x"],
                        before_world["y"],
                        before_world["z"],
                    )
                )
                self.get_logger().info(
                    "GRASP_WORLD_POINT_AFTER %s x=%.4f y=%.4f z=%.4f"
                    % (
                        target,
                        grasp_world["x"],
                        grasp_world["y"],
                        grasp_world["z"],
                    )
                )
                self.get_logger().info(
                    "GRASP_BASE_POINT_BEFORE %s x=%.4f y=%.4f z=%.4f"
                    % (
                        target,
                        before_base["x"],
                        before_base["y"],
                        before_base["z"],
                    )
                )
                self.get_logger().info(
                    "GRASP_BASE_POINT_AFTER %s x=%.4f y=%.4f z=%.4f"
                    % (target, grasp_base[0], grasp_base[1], grasp_base[2])
                )
        self._log_board_grasp_candidates(target, config, object_world_pose)
        block_center = board_base_block_local_center(config)
        block_world = board_base_block_world_center(config, object_world_pose)
        block_axis_yaw = board_base_block_long_axis_world_yaw(
            config,
            object_world_pose,
        )
        if block_center is not None and block_world is not None:
            block_size = config.get("base_block_size", (0.0, 0.0, 0.0))
            block_rpy = config.get("base_block_local_rpy", (0.0, 0.0, 0.0))
            self.get_logger().info(
                "BOARD_BASE_BLOCK_LOCAL_CENTER %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    block_center["x"],
                    block_center["y"],
                    block_center["z"],
                )
            )
            self.get_logger().info(
                "BOARD_BASE_BLOCK_SIZE %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    float(block_size[0]),
                    float(block_size[1]),
                    float(block_size[2]),
                )
            )
            self.get_logger().info(
                "TARGET_COARSE_BLOCK_LOCAL_CENTER %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    block_center["x"],
                    block_center["y"],
                    block_center["z"],
                )
            )
            self.get_logger().info(
                "TARGET_COARSE_BLOCK_SIZE %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    float(block_size[0]),
                    float(block_size[1]),
                    float(block_size[2]),
                )
            )
            self.get_logger().info(
                "TARGET_COARSE_BLOCK_LOCAL_RPY %s roll=%.4f pitch=%.4f yaw=%.4f"
                % (
                    target,
                    float(block_rpy[0]),
                    float(block_rpy[1]),
                    float(block_rpy[2]),
                )
            )
            self.get_logger().info(
                "BOARD_BASE_BLOCK_WORLD_CENTER %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    block_world["x"],
                    block_world["y"],
                    block_world["z"],
                )
            )
            self.get_logger().info(
                "TARGET_COARSE_BLOCK_WORLD_CENTER %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    block_world["x"],
                    block_world["y"],
                    block_world["z"],
                )
            )
            self.get_logger().info(
                "TARGET_COARSE_BLOCK_BASE_POINT %s x=%.4f y=%.4f z=%.4f"
                % (target, grasp_base[0], grasp_base[1], grasp_base[2])
            )
            self.get_logger().info(
                "BOARD_BASE_BLOCK_GRASP_POINT %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    grasp_world["x"],
                    grasp_world["y"],
                    grasp_world["z"],
                )
            )
        if block_axis_yaw is not None:
            self.get_logger().info(
                "BOARD_BASE_BLOCK_LONG_AXIS_YAW %s yaw=%.4f"
                % (target, block_axis_yaw)
            )
            self.get_logger().info(
                "COARSE_BLOCK_GRASP_YAW %s yaw=%.4f"
                % (target, grasp_world["yaw"])
            )
        if target == "board_test_fixture":
            old_point = {"x": 0.10275911, "y": -0.00783486, "z": 0.006}
            old_world = _configured_local_point_world(
                old_point,
                config,
                object_world_pose,
            )
            self.get_logger().info(
                "OLD_GRASP_POINT %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    old_world["x"],
                    old_world["y"],
                    old_world["z"],
                )
            )
            self.get_logger().info(
                "NEW_GRASP_POINT %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    grasp_world["x"],
                    grasp_world["y"],
                    grasp_world["z"],
                )
            )
        self.get_logger().info(
            "BASE_LINK_WORLD_POSE %s" % self._format_pose_dict(base_world_pose)
        )
        self.get_logger().info(
            "TARGET_WORLD_POSE %s x=%.4f y=%.4f z=%.4f yaw=%.4f"
            % (
                target,
                grasp_world["x"],
                grasp_world["y"],
                grasp_world["z"],
                grasp_world["yaw"],
            )
        )
        self.get_logger().info(
            "TARGET_BASE_POSE %s x=%.4f y=%.4f z=%.4f yaw=%.4f "
            "tcp_x=%.4f tcp_y=%.4f tcp_z=%.4f"
            % (
                target,
                grasp_base[0],
                grasp_base[1],
                grasp_base[2],
                grasp_base_yaw,
                tcp_target[0],
                tcp_target[1],
                tcp_target[2],
            )
        )
        self.get_logger().info(
            "VALIDATION_GRASP_YAW %s yaw=%.4f" % (target, grasp_base_yaw)
        )
        if target == "board_test_fixture":
            z_adjust = float(config.get("grasp_z_adjust", 0.0))
            self.get_logger().info(
                "GRASP_Z_ADJUST %s %+0.4f" % (target, z_adjust)
            )
            self.get_logger().info(
                "OLD_BOARD_GRASP_Z %s z=%.4f"
                % (target, grasp_base[2] - z_adjust)
            )
            self.get_logger().info(
                "NEW_BOARD_GRASP_Z %s z=%.4f" % (target, grasp_base[2])
            )
            self.get_logger().info(
                "OLD_TCP_TARGET_Z %s z=%.4f"
                % (target, tcp_target[2] - z_adjust)
            )
            self.get_logger().info(
                "NEW_TCP_TARGET_Z %s z=%.4f" % (target, tcp_target[2])
            )
            old_base = {
                "x": -4.70,
                "y": -3.35,
                "z": float(base_world_pose["z"]),
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": math.pi / 2.0,
            }
            old_target_base = world_pose_to_base_pose(grasp_world, old_base)
            new_target_base = world_pose_to_base_pose(grasp_world, base_world_pose)
            self.get_logger().info(
                "OLD_VALIDATION_BASE_POSE %s x=%.4f y=%.4f yaw=%.4f"
                % (target, old_base["x"], old_base["y"], old_base["yaw"])
            )
            self.get_logger().info(
                "NEW_VALIDATION_BASE_POSE %s x=%.4f y=%.4f yaw=%.4f"
                % (
                    target,
                    base_world_pose["x"],
                    base_world_pose["y"],
                    base_world_pose["yaw"],
                )
            )
            self.get_logger().info(
                "OLD_TARGET_BASE_POSE %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    old_target_base["x"],
                    old_target_base["y"],
                    old_target_base["z"],
                )
            )
            self.get_logger().info(
                "NEW_TARGET_BASE_POSE %s x=%.4f y=%.4f z=%.4f"
                % (
                    target,
                    new_target_base["x"],
                    new_target_base["y"],
                    new_target_base["z"],
                )
            )
        self.get_logger().info(
            "FINAL_GRASP_YAW %s yaw=%.4f" % (target, grasp_base_yaw)
        )
        self.get_logger().info(
            "FINAL_GRASP_RPY %s roll=%.4f pitch=%.4f yaw=%.4f"
            % (target, grasp_rpy[0], grasp_rpy[1], grasp_rpy[2])
        )
        self.get_logger().info(
            "FINAL_GRASP_QUAT %s x=%.6f y=%.6f z=%.6f w=%.6f"
            % (
                target,
                grasp_quat[0],
                grasp_quat[1],
                grasp_quat[2],
                grasp_quat[3],
            )
        )
        self.get_logger().info(
            "FINAL_PRE_GRASP_QUAT %s x=%.6f y=%.6f z=%.6f w=%.6f "
            "pre_x=%.4f pre_y=%.4f pre_z=%.4f"
            % (
                target,
                grasp_quat[0],
                grasp_quat[1],
                grasp_quat[2],
                grasp_quat[3],
                pre_grasp[0],
                pre_grasp[1],
                pre_grasp[2],
            )
        )
        if target == "material_spare_igbt":
            self._log_caliper_grasp_config(
                config,
                object_world_pose,
                base_world_pose,
                grasp_base,
                tcp_target,
                grasp_base_yaw,
                grasp_quat,
                grasp_rpy,
            )

    def _log_caliper_grasp_config(
        self,
        config: dict,
        object_world_pose: dict,
        base_world_pose: dict,
        grasp_base: list[float],
        tcp_target: list[float],
        grasp_base_yaw: float,
        grasp_quat: list[float],
        grasp_rpy: tuple[float, float, float],
    ) -> None:
        point = config.get("grasp_point_local", config["grasp_offset"])
        expected_width_mm = float(config.get("expected_grasp_width_mm", 0.0))
        preopen_command = float(config.get("pre_grasp_position", 0.0))
        preopen_gap_mm = (0.092 - 2.0 * preopen_command) * 1000.0
        expected_contact_q = float(
            config.get(
                "expected_contact_q",
                (0.092 - expected_width_mm / 1000.0) / 2.0,
            )
        )
        tactile_start = float(config.get("tactile_start_position", 0.006))
        tactile_max = float(config.get("tactile_max_position", 0.0185))
        self.get_logger().info(
            "CALIPER_GRASP_CONFIG "
            "grasp_local_point=(%.5f,%.5f,%.5f) "
            "closing_axis=%s expected_width_mm=%.2f "
            "preopen_gap_mm=%.2f preopen_command=%.5f "
            "expected_contact_q=%.5f tactile_start=%.5f tactile_max=%.5f "
            "tcp_target=(%.4f,%.4f,%.4f) "
            "orientation=(roll=%.4f,pitch=%.4f,yaw=%.4f,"
            "qx=%.6f,qy=%.6f,qz=%.6f,qw=%.6f)"
            % (
                float(point.get("x", 0.0)),
                float(point.get("y", 0.0)),
                float(point.get("z", 0.0)),
                config.get("gripper_closing_axis_local", "unknown"),
                expected_width_mm,
                preopen_gap_mm,
                preopen_command,
                expected_contact_q,
                tactile_start,
                tactile_max,
                tcp_target[0],
                tcp_target[1],
                tcp_target[2],
                grasp_rpy[0],
                grasp_rpy[1],
                grasp_rpy[2],
                grasp_quat[0],
                grasp_quat[1],
                grasp_quat[2],
                grasp_quat[3],
            )
        )
        grasp_z_adjust = float(config.get("grasp_z_adjust", 0.0))
        self.get_logger().info(
            "CALIPER_GRASP_Z "
            "model_world_z=%.4f local_z=%.5f base_link_world_z=%.4f "
            "grasp_base_z=%.4f tcp_clearance=%.4f grasp_z_adjust=%.4f "
            "final_tcp_z=%.4f"
            % (
                float(object_world_pose["z"]),
                float(point.get("z", 0.0)),
                float(base_world_pose["z"]),
                grasp_base[2],
                float(config["tcp_clearance"]),
                grasp_z_adjust,
                tcp_target[2],
            )
        )

    def _log_board_grasp_candidates(
        self,
        target: str,
        config: dict,
        object_world_pose: dict,
    ) -> None:
        for index in (1, 2):
            candidate = config.get(f"grasp_candidate_{index}")
            if not candidate:
                continue
            center = candidate["local_center"]
            size = candidate["size"]
            world_center = configured_candidate_world_point(
                candidate,
                config,
                object_world_pose,
                point_key="local_center",
            )
            world_grasp = configured_candidate_world_point(
                candidate,
                config,
                object_world_pose,
                point_key="grasp_local_point",
            )
            grasp_yaw = configured_candidate_world_yaw(
                candidate,
                config,
                object_world_pose,
                yaw_key="grasp_yaw",
            )
            long_axis_yaw = configured_candidate_world_yaw(
                candidate,
                config,
                object_world_pose,
                yaw_key="long_axis_yaw",
            )
            prefix = f"CANDIDATE_{index}"
            self.get_logger().info(
                "%s_LABEL %s %s" % (prefix, target, candidate.get("label", ""))
            )
            self.get_logger().info(
                "%s_LOCAL_CENTER %s x=%.4f y=%.4f z=%.4f"
                % (
                    prefix,
                    target,
                    float(center["x"]),
                    float(center["y"]),
                    float(center["z"]),
                )
            )
            self.get_logger().info(
                "%s_SIZE %s x=%.4f y=%.4f z=%.4f"
                % (
                    prefix,
                    target,
                    float(size[0]),
                    float(size[1]),
                    float(size[2]),
                )
            )
            self.get_logger().info(
                "%s_WORLD_CENTER %s x=%.4f y=%.4f z=%.4f"
                % (
                    prefix,
                    target,
                    world_center["x"],
                    world_center["y"],
                    world_center["z"],
                )
            )
            self.get_logger().info(
                "%s_GRASP_POINT %s x=%.4f y=%.4f z=%.4f"
                % (
                    prefix,
                    target,
                    world_grasp["x"],
                    world_grasp["y"],
                    world_grasp["z"],
                )
            )
            self.get_logger().info(
                "%s_LONG_AXIS_YAW %s yaw=%.4f"
                % (prefix, target, long_axis_yaw)
            )
            self.get_logger().info(
                "%s_GRASP_YAW %s yaw=%.4f" % (prefix, target, grasp_yaw)
            )

    def _log_validation_pick_targets(
        self,
        target: str,
        object_grasp_point_base: list[float],
        tcp_target: list[float],
        pre_grasp: list[float],
    ) -> None:
        self.get_logger().info(
            "VALIDATION_OBJECT_GRASP_POINT_BASE %s x=%.4f y=%.4f z=%.4f"
            % (
                target,
                object_grasp_point_base[0],
                object_grasp_point_base[1],
                object_grasp_point_base[2],
            )
        )
        self.get_logger().info(
            "VALIDATION_PICK_TCP_TARGET %s x=%.4f y=%.4f z=%.4f"
            % (target, tcp_target[0], tcp_target[1], tcp_target[2])
        )
        self.get_logger().info(
            "VALIDATION_PRE_GRASP_TARGET %s x=%.4f y=%.4f z=%.4f"
            % (target, pre_grasp[0], pre_grasp[1], pre_grasp[2])
        )

    def _log_gripper_world_poses_at_grasp(
        self, target: str, base_world_pose: dict
    ) -> None:
        base_pose = base_world_pose
        self.get_logger().info(
            "BASE_LINK_WORLD_POSE_AT_GRASP %s"
            % self._format_pose_dict(base_pose)
        )
        for label, link_name in (
            ("TCP_WORLD_POSE_AT_GRASP", GRIPPER_TCP_LINK),
            ("LEFT_FINGER_WORLD_POSE_AT_GRASP", GRIPPER_LEFT_FINGER_LINK),
            ("RIGHT_FINGER_WORLD_POSE_AT_GRASP", GRIPPER_RIGHT_FINGER_LINK),
        ):
            pose = self._link_world_pose_from_tf(link_name, base_pose)
            if pose is None:
                self.get_logger().warn(
                    "%s %s unavailable from TF" % (label, target)
                )
                continue
            self.get_logger().info(
                "%s %s %s" % (label, target, self._format_pose_dict(pose))
            )
        self._log_gripper_world_z_summary(target, base_pose)

    def _link_world_pose_from_tf(
        self, link_name: str, base_world_pose: dict
    ) -> dict | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                BASE_LINK_FRAME,
                link_name,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                "TF lookup failed base_link <- %s: %s" % (link_name, exc)
            )
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = float(base_world_pose["yaw"])
        x_world = (
            float(base_world_pose["x"])
            + math.cos(yaw) * float(translation.x)
            - math.sin(yaw) * float(translation.y)
        )
        y_world = (
            float(base_world_pose["y"])
            + math.sin(yaw) * float(translation.x)
            + math.cos(yaw) * float(translation.y)
        )
        z_world = float(base_world_pose["z"]) + float(translation.z)
        world_quat = quaternion_multiply(
            yaw_to_quaternion(yaw),
            (
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        )
        roll, pitch, world_yaw = quaternion_to_rpy(*world_quat)
        return {
            "x": x_world,
            "y": y_world,
            "z": z_world,
            "roll": roll,
            "pitch": pitch,
            "yaw": world_yaw,
        }

    def _log_gripper_world_z_summary(
        self, target: str, base_world_pose: dict
    ) -> None:
        tcp_pose = self._link_world_pose_from_tf(GRIPPER_TCP_LINK, base_world_pose)
        left_pose = self._link_world_pose_from_tf(
            GRIPPER_LEFT_FINGER_LINK,
            base_world_pose,
        )
        right_pose = self._link_world_pose_from_tf(
            GRIPPER_RIGHT_FINGER_LINK,
            base_world_pose,
        )
        if tcp_pose and left_pose and right_pose:
            self.get_logger().info(
                "GRIPPER_WORLD_Z_AT_GRASP %s TCP_WORLD_Z=%.4f "
                "LEFT_FINGER_WORLD_Z=%.4f RIGHT_FINGER_WORLD_Z=%.4f"
                % (
                    target,
                    tcp_pose["z"],
                    left_pose["z"],
                    right_pose["z"],
                )
            )

    def _set_cartesian_avoid_collisions(self, enabled: bool) -> None:
        request = getattr(
            self.pp.moveit2,
            "_MoveIt2__cartesian_path_request",
            None,
        )
        if request is None or not hasattr(request, "avoid_collisions"):
            self.get_logger().warn("CARTESIAN_AVOID_COLLISIONS unavailable")
            return
        request.avoid_collisions = bool(enabled)
        self.get_logger().info(
            "CARTESIAN_AVOID_COLLISIONS %s"
            % ("true" if request.avoid_collisions else "false")
        )

    def _set_cartesian_jump_threshold(self, threshold: float) -> None:
        request = getattr(
            self.pp.moveit2,
            "_MoveIt2__cartesian_path_request",
            None,
        )
        if request is None or not hasattr(request, "jump_threshold"):
            self.get_logger().warn("CARTESIAN_JUMP_THRESHOLD unavailable")
            return
        request.jump_threshold = float(threshold)
        self.get_logger().info(
            "CARTESIAN_JUMP_THRESHOLD %.3f" % request.jump_threshold
        )

    def _plan_validation_cartesian_descend(
        self,
        target: str,
        tcp_target: list[float],
        grasp_quat: list[float],
        config: dict | None = None,
        base_world_pose: dict | None = None,
    ):
        if not hasattr(self.pp.moveit2, "plan"):
            self.get_logger().warn(
                "validation Cartesian descend unavailable: no plan API"
            )
            return None
        self._set_cartesian_avoid_collisions(True)
        self._set_cartesian_jump_threshold(DESCEND_CARTESIAN_JUMP_THRESHOLD)
        try:
            trajectory = self.pp._with_local_arm_scaling(  # noqa: SLF001
                True,
                lambda: self.pp.moveit2.plan(
                    position=list(tcp_target),
                    quat_xyzw=grasp_quat,
                    frame_id=BASE_LINK_FRAME,
                    target_link=GRIPPER_TCP_LINK,
                    tolerance_position=DEFAULT_GRASP_TOLERANCE_POSITION,
                    tolerance_orientation=DEFAULT_GRASP_TOLERANCE_ORIENTATION,
                    cartesian=True,
                ),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                "validation Cartesian descend planning failed for %s: %s"
                % (target, exc)
            )
            return None
        if trajectory is None:
            self.get_logger().warn(
                "validation Cartesian descend planning returned no trajectory"
            )
            return None
        self.pp._normalize_wrist_trajectory_to_current(trajectory)  # noqa: SLF001
        _ensure_trajectory_timing(trajectory)
        self._log_descend_trajectory_extent(target, trajectory)
        self._log_state_validity_for_current_joints(
            "PRE_GRASP_STATE_VALIDITY",
            target,
        )
        self._log_state_validity_for_trajectory_endpoint(
            "PARTIAL_DESCEND_END_STATE_VALIDITY",
            target,
            trajectory,
        )
        if (
            target == "board_test_fixture"
            and config is not None
            and base_world_pose is not None
        ):
            self._log_coordinate_consistency_snapshot(
                "PARTIAL_DESCEND_END_COORDINATE_CONSISTENCY",
                target,
                config,
                base_world_pose,
                joint_state=self._trajectory_endpoint_joint_state(trajectory),
            )
        if not self._validate_descend_joint_continuity(target, trajectory):
            return None
        return trajectory

    def _validation_descend_with_horizontal_insert(
        self,
        target: str,
        pre_grasp: list[float],
        tcp_target: list[float],
        grasp_quat: list[float],
        grasp_yaw: float,
    ) -> bool:
        safe_z = MATERIAL_SPARE_INSERT_SAFE_Z
        stage1 = [tcp_target[0], tcp_target[1], safe_z]
        self.get_logger().info(
            "DESCEND_STAGE1_START %s x=%.4f y=%.4f z=%.4f"
            % (target, pre_grasp[0], pre_grasp[1], pre_grasp[2])
        )
        self.get_logger().info(
            "DESCEND_STAGE1_END %s x=%.4f y=%.4f z=%.4f"
            % (target, stage1[0], stage1[1], stage1[2])
        )
        self.get_logger().info(
            "DESCEND_STAGE1_TARGET %s x=%.4f y=%.4f z=%.4f"
            % (target, stage1[0], stage1[1], stage1[2])
        )
        trajectory, fraction = self._plan_validation_cartesian_pose(
            target,
            "DESCEND_STAGE1",
            stage1,
            grasp_quat,
        )
        self.get_logger().info(
            "DESCEND_STAGE1_FRACTION %s %.4f" % (target, fraction)
        )
        if trajectory is None or fraction < CARTESIAN_SEGMENT_MIN_FRACTION:
            return False
        if not self._execute_validation_descend(trajectory):
            return False

        insert_x = math.cos(grasp_yaw) * MATERIAL_SPARE_INSERT_DISTANCE
        insert_y = math.sin(grasp_yaw) * MATERIAL_SPARE_INSERT_DISTANCE
        stage2_candidates = (
            [stage1[0] + insert_x, stage1[1] + insert_y, stage1[2]],
            [stage1[0] - insert_x, stage1[1] - insert_y, stage1[2]],
        )
        stage2_success = None
        for index, stage2 in enumerate(stage2_candidates, start=1):
            self.get_logger().info(
                "DESCEND_STAGE2_TARGET %s attempt=%d x=%.4f y=%.4f z=%.4f"
                % (target, index, stage2[0], stage2[1], stage2[2])
            )
            trajectory, fraction = self._plan_validation_cartesian_pose(
                target,
                "DESCEND_STAGE2",
                stage2,
                grasp_quat,
            )
            self.get_logger().info(
                "DESCEND_STAGE2_FRACTION %s attempt=%d %.4f"
                % (target, index, fraction)
            )
            if trajectory is None or fraction < CARTESIAN_SEGMENT_MIN_FRACTION:
                continue
            if not self._execute_validation_descend(trajectory):
                continue
            stage2_success = stage2
            break
        if stage2_success is None:
            return False

        self.get_logger().info(
            "DESCEND_STAGE3_TARGET %s x=%.4f y=%.4f z=%.4f"
            % (target, tcp_target[0], tcp_target[1], tcp_target[2])
        )
        trajectory, fraction = self._plan_validation_cartesian_pose(
            target,
            "DESCEND_STAGE3",
            tcp_target,
            grasp_quat,
        )
        self.get_logger().info(
            "DESCEND_STAGE3_FRACTION %s %.4f" % (target, fraction)
        )
        if trajectory is None or fraction < CARTESIAN_SEGMENT_MIN_FRACTION:
            return False
        return self._execute_validation_descend(trajectory)

    def _plan_validation_cartesian_pose(
        self,
        target: str,
        label: str,
        tcp_pose: list[float],
        grasp_quat: list[float],
    ):
        if not hasattr(self.pp.moveit2, "set_pose_goal"):
            self.get_logger().warn("%s %s unavailable: no pose goal API" % (label, target))
            return None, 0.0
        self._set_cartesian_avoid_collisions(True)
        self._set_cartesian_jump_threshold(DESCEND_CARTESIAN_JUMP_THRESHOLD)
        try:
            trajectory, fraction = self.pp._with_local_arm_scaling(  # noqa: SLF001
                True,
                lambda: self._call_validation_cartesian_service(
                    tcp_pose,
                    grasp_quat,
                ),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                "%s Cartesian planning failed for %s: %s" % (label, target, exc)
            )
            return None, 0.0
        if trajectory is None:
            return None, fraction
        self.pp._normalize_wrist_trajectory_to_current(trajectory)  # noqa: SLF001
        _ensure_trajectory_timing(trajectory)
        self._log_descend_trajectory_extent("%s_%s" % (label, target), trajectory)
        if not self._validate_descend_joint_continuity(
            "%s_%s" % (label, target),
            trajectory,
        ):
            return None, fraction
        return trajectory, fraction

    def _call_validation_cartesian_service(
        self,
        tcp_pose: list[float],
        grasp_quat: list[float],
    ):
        moveit2 = self.pp.moveit2
        moveit2.clear_goal_constraints()
        moveit2.set_pose_goal(
            position=list(tcp_pose),
            quat_xyzw=list(grasp_quat),
            frame_id=BASE_LINK_FRAME,
            target_link=GRIPPER_TCP_LINK,
            tolerance_position=DEFAULT_GRASP_TOLERANCE_POSITION,
            tolerance_orientation=DEFAULT_GRASP_TOLERANCE_ORIENTATION,
        )
        goal = getattr(moveit2, "_MoveIt2__move_action_goal")
        if getattr(moveit2, "joint_state", None) is not None:
            goal.request.start_state.joint_state = moveit2.joint_state
        request = getattr(moveit2, "_MoveIt2__cartesian_path_request")
        request.start_state = goal.request.start_state
        request.group_name = goal.request.group_name
        request.link_name = GRIPPER_TCP_LINK
        request.max_step = 0.0025
        request.header.frame_id = BASE_LINK_FRAME
        stamp = self.get_clock().now().to_msg()
        request.header.stamp = stamp
        request.path_constraints = goal.request.path_constraints
        for constraint in request.path_constraints.position_constraints:
            constraint.header.stamp = stamp
        for constraint in request.path_constraints.orientation_constraints:
            constraint.header.stamp = stamp
        target_pose = Pose()
        target_pose.position.x = float(tcp_pose[0])
        target_pose.position.y = float(tcp_pose[1])
        target_pose.position.z = float(tcp_pose[2])
        target_pose.orientation.x = float(grasp_quat[0])
        target_pose.orientation.y = float(grasp_quat[1])
        target_pose.orientation.z = float(grasp_quat[2])
        target_pose.orientation.w = float(grasp_quat[3])
        request.waypoints = [target_pose]
        service = moveit2._plan_cartesian_path_service
        if not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("compute_cartesian_path service unavailable")
            moveit2.clear_goal_constraints()
            return None, 0.0
        response = service.call(request)
        moveit2.clear_goal_constraints()
        fraction = float(getattr(response, "fraction", 0.0))
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                "Cartesian planning failed! Error code: %d"
                % response.error_code.val
            )
            return None, fraction
        return response.solution.joint_trajectory, fraction

    def _execute_validation_descend(self, trajectory) -> bool:
        return bool(
            self.pp._with_local_arm_scaling(  # noqa: SLF001
                True,
                lambda: self.pp._execute_trajectory_via_moveit(  # noqa: SLF001
                    trajectory,
                    DEFAULT_MOVE_TIMEOUT_SEC,
                ),
            )
        )

    def _apply_validation_pre_grasp_branch_constraints(self, target: str) -> bool:
        if target != "high_voltage_probe_kit":
            return False
        moveit2 = getattr(self.pp, "moveit2", None)
        if moveit2 is None:
            return False
        current = self._current_arm_joint_map()
        if not all(name in current for name in VALIDATION_PRE_GRASP_BRANCH_CONSTRAINT_JOINTS):
            self.get_logger().warn(
                "VALIDATION_BRANCH_CONSTRAINT_SKIPPED %s incomplete_joint_state"
                % target
            )
            return False
        positions = [
                current[name]
                for name in VALIDATION_PRE_GRASP_BRANCH_CONSTRAINT_JOINTS
            ]
        if hasattr(moveit2, "set_joint_path_constraints"):
            moveit2.set_joint_path_constraints(
                positions,
                joint_names=list(VALIDATION_PRE_GRASP_BRANCH_CONSTRAINT_JOINTS),
                tolerance=list(VALIDATION_PRE_GRASP_BRANCH_CONSTRAINT_TOLERANCES),
            )
            backend = "pymoveit2_api"
        else:
            goal = getattr(moveit2, "_MoveIt2__move_action_goal", None)
            if goal is None:
                self.get_logger().warn(
                    "VALIDATION_BRANCH_CONSTRAINT_UNAVAILABLE %s" % target
                )
                return False
            constraints = Constraints()
            constraints.name = "validation_pre_grasp_branch"
            for name, position, tolerance in zip(
                VALIDATION_PRE_GRASP_BRANCH_CONSTRAINT_JOINTS,
                positions,
                VALIDATION_PRE_GRASP_BRANCH_CONSTRAINT_TOLERANCES,
            ):
                constraint = JointConstraint()
                constraint.joint_name = name
                constraint.position = float(position)
                constraint.tolerance_above = float(tolerance)
                constraint.tolerance_below = float(tolerance)
                constraint.weight = 1.0
                constraints.joint_constraints.append(constraint)
            goal.request.path_constraints = constraints
            backend = "move_action_goal_request"
        self.get_logger().info(
            "VALIDATION_PRE_GRASP_BRANCH_CONSTRAINT %s backend=%s %s tolerance=%s"
            % (
                target,
                backend,
                self._format_arm_joints(current),
                ",".join(
                    "%.3f" % value
                    for value in VALIDATION_PRE_GRASP_BRANCH_CONSTRAINT_TOLERANCES
                ),
            )
        )
        return True

    def _clear_validation_branch_constraints(self, target: str) -> None:
        moveit2 = getattr(self.pp, "moveit2", None)
        cleared = False
        if moveit2 is not None and hasattr(moveit2, "clear_path_constraints"):
            moveit2.clear_path_constraints()
            cleared = True
        goal = (
            getattr(moveit2, "_MoveIt2__move_action_goal", None)
            if moveit2 is not None
            else None
        )
        if goal is not None:
            goal.request.path_constraints = Constraints()
            cleared = True
        if cleared:
            self.get_logger().info(
                "VALIDATION_PRE_GRASP_BRANCH_CONSTRAINT_CLEARED %s" % target
            )

    def _execute_validation_lift(
        self,
        target: str,
        lift_target: list[float],
        lift_quat: list[float],
    ) -> bool:
        ok, stage = self._try_validation_lift(
            target,
            lift_target,
            lift_quat,
            "primary",
        )
        if ok:
            return True

        if stage != "LIFT_EXECUTION":
            terminal = {
                "LIFT_PATH": "FAILED_LIFT_PATH",
                "LIFT_FRACTION": "FAILED_LIFT_FRACTION",
            }.get(stage, "FAILED_LIFT_EXECUTION")
            self._publish_status(terminal, target)
            return False
        if not self.pp._holding_is_healthy():  # noqa: SLF001
            self._publish_status(
                "LIFT_RECOVERY_BLOCKED %s reason=HOLD_LOST" % target,
                "",
            )
            self._publish_status("FAILED_HOLD_LOST", target)
            return False

        self._publish_status(
            "LIFT_RECOVERY_ATTEMPT %s reason=%s scene=detach_carried_sample"
            % (target, stage),
            "",
        )
        self.pp._detach_carried_sample()  # noqa: SLF001
        time.sleep(0.2)
        ok, retry_stage = self._try_validation_lift(
            target,
            lift_target,
            lift_quat,
            "scene_detached",
        )
        self.pp._attach_carried_sample()  # noqa: SLF001
        if ok:
            self._publish_status(
                "LIFT_RECOVERY_SUCCESS %s reason=%s" % (target, stage),
                "",
            )
            return True
        terminal = {
            "LIFT_PATH": "FAILED_LIFT_PATH",
            "LIFT_FRACTION": "FAILED_LIFT_FRACTION",
            "LIFT_EXECUTION": "FAILED_LIFT_EXECUTION",
        }.get(retry_stage, "FAILED_LIFT_EXECUTION")
        self._publish_status(terminal, target)
        return False

    def _try_validation_lift(
        self,
        target: str,
        lift_target: list[float],
        lift_quat: list[float],
        attempt_label: str,
    ) -> tuple[bool, str]:
        start_tcp = self._current_tcp_position()
        plan_start_joints = self._current_arm_joint_map()
        self._log_current_arm_joints(
            "LIFT_PLAN_START_JOINTS_%s" % attempt_label.upper(),
            target,
        )
        self.get_logger().info(
            "LIFT_START_TCP %s attempt=%s %s"
            % (target, attempt_label, self._format_xyz(start_tcp))
        )
        self.get_logger().info(
            "LIFT_TARGET_TCP %s attempt=%s x=%.4f y=%.4f z=%.4f"
            % (
                target,
                attempt_label,
                lift_target[0],
                lift_target[1],
                lift_target[2],
            )
        )
        trajectory, fraction = self._plan_validation_cartesian_pose(
            target,
            "LIFT_%s" % attempt_label.upper(),
            lift_target,
            lift_quat,
        )
        lift_distance = self._distance_xyz(start_tcp, lift_target)
        if trajectory is None:
            self._publish_lift_result(
                target,
                ok=False,
                stage="LIFT_PATH",
                fraction=fraction,
                lift_distance=lift_distance,
                start_tcp=start_tcp,
                target_tcp=lift_target,
                final_tcp=self._current_tcp_position(),
                attempt_label=attempt_label,
            )
            return False, "LIFT_PATH"
        if fraction < CARTESIAN_SEGMENT_MIN_FRACTION:
            self._publish_lift_result(
                target,
                ok=False,
                stage="LIFT_FRACTION",
                fraction=fraction,
                lift_distance=lift_distance,
                start_tcp=start_tcp,
                target_tcp=lift_target,
                final_tcp=self._current_tcp_position(),
                attempt_label=attempt_label,
            )
            return False, "LIFT_FRACTION"

        self._log_state_validity_for_current_joints(
            "LIFT_START_STATE_VALIDITY_%s" % attempt_label.upper(),
            target,
        )
        self._log_state_validity_for_trajectory_endpoint(
            "LIFT_END_STATE_VALIDITY_%s" % attempt_label.upper(),
            target,
            trajectory,
        )
        start_error = self._max_start_joint_error(plan_start_joints, trajectory)
        self._log_lift_trajectory_diagnostic(
            target,
            trajectory,
            fraction,
            start_error,
            attempt_label,
        )
        self._publish_lift_diagnostic(
            target,
            fraction,
            lift_distance,
            start_error,
            start_tcp,
            lift_target,
            trajectory,
            attempt_label,
        )
        execute_start = time.monotonic()
        self._log_current_arm_joints(
            "LIFT_EXECUTE_START_JOINTS_%s" % attempt_label.upper(),
            target,
        )
        ok = bool(
            self.pp._with_local_arm_scaling(  # noqa: SLF001
                True,
                lambda: self.pp._execute_trajectory_via_moveit(  # noqa: SLF001
                    trajectory,
                    DEFAULT_MOVE_TIMEOUT_SEC,
                ),
            )
        )
        duration = time.monotonic() - execute_start
        final_tcp = self._current_tcp_position()
        self._log_current_arm_joints(
            "LIFT_FINAL_JOINTS_%s" % attempt_label.upper(),
            target,
        )
        self.get_logger().info(
            "LIFT_FINAL_TCP %s attempt=%s %s"
            % (target, attempt_label, self._format_xyz(final_tcp))
        )
        self._publish_lift_result(
            target,
            ok=ok,
            stage="LIFT_OK" if ok else "LIFT_EXECUTION",
            fraction=fraction,
            lift_distance=lift_distance,
            start_tcp=start_tcp,
            target_tcp=lift_target,
            final_tcp=final_tcp,
            duration=duration,
            error_code=getattr(self.pp, "last_execute_error_code", ""),
            error_text=getattr(self.pp, "last_execute_error_text", ""),
            attempt_label=attempt_label,
        )
        if not ok:
            if self._lift_state_verified_after_abort(target, final_tcp, lift_target):
                return True, "LIFT_STATE_VERIFIED"
            return False, "LIFT_EXECUTION"
        return True, "LIFT_OK"

    def _lift_state_verified_after_abort(
        self,
        target: str,
        final_tcp: list[float] | None,
        target_tcp: list[float],
    ) -> bool:
        if final_tcp is None:
            return False
        final_error = self._distance_xyz(final_tcp, target_tcp)
        z_error = abs(float(final_tcp[2]) - float(target_tcp[2]))
        holding_ok = self.pp._holding_is_healthy()  # noqa: SLF001
        verified = (
            final_error <= LIFT_STATE_VERIFIED_MAX_POSITION_ERROR_M
            and z_error <= LIFT_STATE_VERIFIED_MAX_Z_ERROR_M
            and holding_ok
        )
        self._publish_status(
            "LIFT_STATE_VERIFIED %s ok=%d final_target_error=%.6f z_error=%.6f holding=%d"
            % (target, int(verified), final_error, z_error, int(holding_ok)),
            "",
        )
        return verified

    def _publish_lift_diagnostic(
        self,
        target: str,
        fraction: float,
        lift_distance: float,
        start_error: float,
        start_tcp: list[float] | None,
        target_tcp: list[float],
        trajectory,
        attempt_label: str,
    ) -> None:
        points = list(getattr(trajectory, "points", []))
        fields = {
            "lift_fraction": fraction,
            "lift_distance": lift_distance,
            "max_start_joint_error": start_error,
            "lift_point_count": len(points),
            "lift_attempt": attempt_label,
        }
        fields.update(self._tcp_status_fields("lift_start", start_tcp))
        fields.update(self._tcp_status_fields("lift_target", target_tcp))
        self._publish_status(
            "LIFT_DIAGNOSTIC %s %s" % (target, self._format_status_fields(fields)),
            ""
        )

    def _publish_lift_result(
        self,
        target: str,
        ok: bool,
        stage: str,
        fraction: float,
        lift_distance: float,
        start_tcp: list[float] | None,
        target_tcp: list[float],
        final_tcp: list[float] | None,
        duration: float | None = None,
        error_code: str | int | None = "",
        error_text: str | None = "",
        attempt_label: str = "",
    ) -> None:
        fields = {
            "ok": int(ok),
            "lift_stage": stage,
            "lift_attempt": attempt_label,
            "lift_fraction": fraction,
            "lift_distance": lift_distance,
            "lift_duration": "" if duration is None else duration,
            "lift_error_code": "" if error_code is None else error_code,
            "lift_error_text": error_text or "",
        }
        fields.update(self._tcp_status_fields("lift_start", start_tcp))
        fields.update(self._tcp_status_fields("lift_target", target_tcp))
        fields.update(self._tcp_status_fields("lift_final", final_tcp))
        self._publish_status(
            "LIFT_EXECUTION_RESULT %s %s"
            % (target, self._format_status_fields(fields)),
            "",
        )

    def _log_lift_trajectory_diagnostic(
        self,
        target: str,
        trajectory,
        fraction: float,
        start_error: float,
        attempt_label: str,
    ) -> None:
        points = list(getattr(trajectory, "points", []))
        self.get_logger().info(
            "LIFT_FRACTION %s attempt=%s %.4f"
            % (target, attempt_label, fraction)
        )
        self.get_logger().info(
            "LIFT_START_STATE_ERROR %s attempt=%s max_joint_error=%.6f"
            % (target, attempt_label, start_error)
        )
        self.get_logger().info(
            "LIFT_TRAJECTORY_POINTS %s attempt=%s count=%d"
            % (target, attempt_label, len(points))
        )
        if not points:
            return
        first = self._trajectory_point_joints_text(trajectory, 0)
        second = self._trajectory_point_joints_text(
            trajectory,
            1 if len(points) > 1 else 0,
        )
        last = self._trajectory_point_joints_text(trajectory, len(points) - 1)
        self.get_logger().info(
            "LIFT_TRAJECTORY_FIRST %s attempt=%s %s"
            % (target, attempt_label, first)
        )
        self.get_logger().info(
            "LIFT_TRAJECTORY_SECOND %s attempt=%s %s"
            % (target, attempt_label, second)
        )
        self.get_logger().info(
            "LIFT_TRAJECTORY_LAST %s attempt=%s %s"
            % (target, attempt_label, last)
        )
        self.get_logger().info(
            "LIFT_TRAJECTORY_TIME %s attempt=%s first=%.6f second=%.6f last=%.6f"
            % (
                target,
                attempt_label,
                self._point_time_sec(points[0]),
                self._point_time_sec(points[1] if len(points) > 1 else points[0]),
                self._point_time_sec(points[-1]),
            )
        )

    def _current_tcp_position(self) -> list[float] | None:
        pose = self._lookup_transform_pose(BASE_LINK_FRAME, GRIPPER_TCP_LINK)
        if pose is None:
            return None
        return [pose["x"], pose["y"], pose["z"]]

    def _current_arm_joint_map(self) -> dict[str, float]:
        joint_state = getattr(self.pp.moveit2, "joint_state", None)
        if joint_state is None:
            return {}
        return {
            name: float(position)
            for name, position in zip(joint_state.name, joint_state.position)
            if name in UR_ARM_JOINTS
        }

    def _max_start_joint_error(
        self,
        start_joints: dict[str, float],
        trajectory,
    ) -> float:
        first = self._trajectory_point_arm_joint_map(trajectory, 0)
        errors = [
            abs(first[name] - start_joints[name])
            for name in UR_ARM_JOINTS
            if name in first and name in start_joints
        ]
        return max(errors) if errors else float("nan")

    def _trajectory_point_arm_joint_map(self, trajectory, point_index: int) -> dict[str, float]:
        points = list(getattr(trajectory, "points", []))
        names = list(getattr(trajectory, "joint_names", []))
        if not points or point_index >= len(points):
            return {}
        positions = list(points[point_index].positions)
        return {
            name: float(positions[index])
            for index, name in enumerate(names)
            if index < len(positions) and name in UR_ARM_JOINTS
        }

    def _trajectory_point_joints_text(self, trajectory, point_index: int) -> str:
        return self._format_arm_joints(
            self._trajectory_point_arm_joint_map(trajectory, point_index)
        )

    @staticmethod
    def _point_time_sec(point) -> float:
        stamp = getattr(point, "time_from_start", None)
        if stamp is None:
            return float("nan")
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _distance_xyz(start: list[float] | None, end: list[float]) -> float:
        if start is None:
            return float("nan")
        return math.sqrt(
            (float(end[0]) - float(start[0])) ** 2
            + (float(end[1]) - float(start[1])) ** 2
            + (float(end[2]) - float(start[2])) ** 2
        )

    @staticmethod
    def _tcp_status_fields(prefix: str, tcp: list[float] | None) -> dict[str, float | str]:
        if tcp is None:
            return {
                f"{prefix}_tcp_x": "",
                f"{prefix}_tcp_y": "",
                f"{prefix}_tcp_z": "",
            }
        return {
            f"{prefix}_tcp_x": float(tcp[0]),
            f"{prefix}_tcp_y": float(tcp[1]),
            f"{prefix}_tcp_z": float(tcp[2]),
        }

    @staticmethod
    def _format_status_fields(fields: dict[str, object]) -> str:
        parts = []
        for key, value in fields.items():
            if isinstance(value, float):
                text = "" if not math.isfinite(value) else "%.6f" % value
            else:
                text = str(value)
            parts.append("%s=%s" % (key, text))
        return " ".join(parts)

    @staticmethod
    def _format_xyz(xyz: list[float] | None) -> str:
        if xyz is None:
            return "x=nan y=nan z=nan"
        return "x=%.4f y=%.4f z=%.4f" % (xyz[0], xyz[1], xyz[2])

    def _log_descend_trajectory_extent(self, target: str, trajectory) -> None:
        points = list(getattr(trajectory, "points", []))
        self.get_logger().info(
            "DESCEND_TRAJECTORY_POINTS %s count=%d" % (target, len(points))
        )
        endpoint = self._trajectory_endpoint_joints_text(trajectory)
        self.get_logger().info(
            "DESCEND_TRAJECTORY_ENDPOINT %s %s" % (target, endpoint)
        )

    def _log_state_validity_for_current_joints(
        self,
        label: str,
        target: str,
    ) -> None:
        joint_state = getattr(self.pp.moveit2, "joint_state", None)
        if joint_state is None:
            self.get_logger().warn("%s %s unavailable: no joint_state" % (label, target))
            return
        self._log_state_validity(label, target, joint_state)

    def _log_state_validity_for_trajectory_endpoint(
        self,
        label: str,
        target: str,
        trajectory,
    ) -> None:
        points = list(getattr(trajectory, "points", []))
        names = list(getattr(trajectory, "joint_names", []))
        if not points or not names:
            self.get_logger().warn(
                "%s %s unavailable: empty trajectory" % (label, target)
            )
            return
        joint_state = JointState()
        joint_state.name = names
        joint_state.position = list(points[-1].positions)
        self._log_state_validity(label, target, joint_state)

    def _log_state_validity(self, label: str, target: str, joint_state) -> None:
        if not self._state_validity_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(
                "%s %s unavailable: %s service not ready"
                % (label, target, MOVEIT_STATE_VALIDITY_SERVICE)
            )
            return
        request = GetStateValidity.Request()
        request.group_name = "ur_manipulator"
        request.robot_state = RobotState()
        request.robot_state.joint_state = joint_state
        future = self._state_validity_client.call_async(request)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if future.done():
                result = future.result()
                if result is None:
                    self.get_logger().warn("%s %s no response" % (label, target))
                    return
                contacts = list(getattr(result, "contacts", []))
                self.get_logger().info(
                    "%s %s valid=%s contacts=%d"
                    % (
                        label,
                        target,
                        "true" if result.valid else "false",
                        len(contacts),
                    )
                )
                if contacts:
                    contact = contacts[0]
                    self.get_logger().warn(
                        "FIRST_INVALID_COLLISION_PAIR %s %s <-> %s depth=%.6f"
                        % (
                            target,
                            contact.contact_body_1,
                            contact.contact_body_2,
                            float(contact.depth),
                        )
                    )
                elif not result.valid:
                    self.get_logger().warn(
                        "FIRST_INVALID_COLLISION_PAIR %s unavailable"
                        % target
                    )
                return
            time.sleep(0.01)
        self.get_logger().warn("%s %s timed out" % (label, target))

    def _trajectory_endpoint_joint_state(self, trajectory) -> JointState | None:
        points = list(getattr(trajectory, "points", []))
        names = list(getattr(trajectory, "joint_names", []))
        if not points or not names:
            return None
        joint_state = JointState()
        joint_state.name = names
        joint_state.position = list(points[-1].positions)
        return joint_state

    def _log_coordinate_consistency_snapshot(
        self,
        label: str,
        target: str,
        config: dict,
        base_world_pose: dict,
        joint_state=None,
    ) -> None:
        self.get_logger().info(
            "%s %s timestamp=%.6f"
            % (label, target, time.monotonic())
        )
        self._log_gazebo_robot_link_names(label, target)
        object_pose = self._get_entity_world_pose(
            f"{config['entity_name']}::{config['link_name']}"
        )
        if object_pose is not None:
            self.get_logger().info(
                "%s OBJECT_LINK_WORLD_POSE %s %s"
                % (label, target, self._format_pose_dict(object_pose))
            )

        gazebo_z = self._gazebo_frame_world_z_no_fallback()
        tf_world_poses = self._tf_world_poses_from_base_footprint()
        tf_z = {
            frame: pose["z"]
            for frame, pose in tf_world_poses.items()
            if pose is not None
        }
        fk_z = self._moveit_fk_world_z(tf_world_poses, joint_state)
        self.get_logger().info(
            "%s FRAME_Z_TABLE %s FRAME GAZEBO_Z TF_Z MOVEIT_FK_Z"
            % (label, target)
        )
        for frame in CONSISTENCY_DIAG_FRAMES:
            self.get_logger().info(
                "%s FRAME_Z %s %s gazebo=%s tf=%s moveit_fk=%s"
                % (
                    label,
                    target,
                    frame,
                    self._format_optional_float(gazebo_z.get(frame)),
                    self._format_optional_float(tf_z.get(frame)),
                    self._format_optional_float(fk_z.get(frame)),
                )
            )

        grasp_world = configured_grasp_world_pose(config, object_pose)
        target_from_actual_base = world_pose_to_base_pose(
            grasp_world,
            base_world_pose,
        )
        self.get_logger().info(
            "%s TARGET_BASE_FROM_REAL_BASE_LINK %s x=%.4f y=%.4f z=%.4f"
            % (
                label,
                target,
                target_from_actual_base["x"],
                target_from_actual_base["y"],
                target_from_actual_base["z"],
            )
        )

    def _log_gazebo_robot_link_names(self, label: str, target: str) -> None:
        if not self._model_properties_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(
                "%s GAZEBO_MODEL_LINKS %s unavailable: service not ready"
                % (label, target)
            )
            return
        request = GetModelProperties.Request()
        request.model_name = ROBOT_ENTITY_NAME
        future = self._model_properties_client.call_async(request)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if future.done():
                result = future.result()
                if result is None or not result.success:
                    self.get_logger().warn(
                        "%s GAZEBO_MODEL_LINKS %s unavailable"
                        % (label, target)
                    )
                    return
                self.get_logger().info(
                    "%s GAZEBO_CANONICAL_BODY %s %s"
                    % (label, target, result.canonical_body_name)
                )
                self.get_logger().info(
                    "%s GAZEBO_BODY_NAMES %s %s"
                    % (label, target, ",".join(result.body_names))
                )
                return
            time.sleep(0.01)
        self.get_logger().warn(
            "%s GAZEBO_MODEL_LINKS %s timed out" % (label, target)
        )

    def _gazebo_frame_world_z_no_fallback(self) -> dict[str, float | None]:
        values: dict[str, float | None] = {}
        for frame in CONSISTENCY_DIAG_FRAMES:
            pose = self._get_entity_world_pose(f"{ROBOT_ENTITY_NAME}::{frame}")
            values[frame] = None if pose is None else float(pose["z"])
        return values

    def _tf_world_poses_from_base_footprint(self) -> dict[str, dict | None]:
        base_footprint_pose = self._get_entity_world_pose(BASE_FOOTPRINT_ENTITY)
        if base_footprint_pose is None:
            cached = self._model_state_pose(ROBOT_ENTITY_NAME)
            if cached is not None:
                self.get_logger().warn(
                    "using /gazebo/model_states pose for %s as %s pose in "
                    "coordinate consistency diagnostics"
                    % (ROBOT_ENTITY_NAME, BASE_FOOTPRINT_FRAME)
                )
            base_footprint_pose = cached
        poses: dict[str, dict | None] = {
            frame: None for frame in CONSISTENCY_DIAG_FRAMES
        }
        if base_footprint_pose is None:
            return poses
        poses[BASE_FOOTPRINT_FRAME] = base_footprint_pose
        for frame in CONSISTENCY_DIAG_FRAMES:
            if frame == BASE_FOOTPRINT_FRAME:
                continue
            child = self._lookup_transform_pose(BASE_FOOTPRINT_FRAME, frame)
            if child is not None:
                poses[frame] = compose_child_world_pose(base_footprint_pose, child)
        return poses

    def _lookup_transform_pose(self, parent: str, child: str) -> dict | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                parent,
                child,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                "TF lookup failed %s <- %s: %s" % (parent, child, exc)
            )
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        roll, pitch, yaw = quaternion_to_rpy(
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        )
        return {
            "x": float(translation.x),
            "y": float(translation.y),
            "z": float(translation.z),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }

    def _moveit_fk_world_z(
        self,
        tf_world_poses: dict[str, dict | None],
        joint_state=None,
    ) -> dict[str, float | None]:
        values: dict[str, float | None] = {
            frame: None for frame in CONSISTENCY_DIAG_FRAMES
        }
        ur_base_world = tf_world_poses.get("ur_base_link")
        if ur_base_world is None:
            return values
        fk_links = [
            frame
            for frame in CONSISTENCY_DIAG_FRAMES
            if frame != BASE_FOOTPRINT_FRAME
        ]
        try:
            poses = self.pp.moveit2.compute_fk(
                joint_state=joint_state,
                fk_link_names=fk_links,
                wait_for_server_timeout_sec=0.2,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("MoveIt FK diagnostic failed: %s" % exc)
            return values
        if poses is None:
            return values
        if not isinstance(poses, list):
            poses = [poses]
        for link_name, pose_stamped in zip(fk_links, poses):
            pose = pose_stamped.pose
            rel = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
            world_pose = compose_child_world_pose(ur_base_world, rel)
            values[link_name] = float(world_pose["z"])
        return values

    def _format_optional_float(self, value: float | None) -> str:
        return "missing" if value is None else "%.6f" % float(value)

    def _validate_descend_joint_continuity(self, target: str, trajectory) -> bool:
        samples = self._trajectory_arm_joint_samples(trajectory)
        if not samples:
            self.get_logger().warn(
                "DESCEND_JOINT_TRAJECTORY %s unavailable" % target
            )
            return False
        self._log_descend_joint_samples(target, samples)
        max_step = self._max_joint_delta_per_step(samples)
        total = self._total_joint_delta(samples)
        self.get_logger().info(
            "MAX_JOINT_DELTA_PER_STEP %s %s"
            % (target, self._format_arm_joints(max_step))
        )
        self.get_logger().info(
            "TOTAL_JOINT_DELTA %s %s"
            % (target, self._format_arm_joints(total))
        )
        for name, value in max_step.items():
            if value > DESCEND_MAX_JOINT_DELTA_PER_STEP:
                self.get_logger().warn(
                    "Cartesian descend rejected for %s: %s step jump %.4f > %.4f"
                    % (
                        target,
                        name,
                        value,
                        DESCEND_MAX_JOINT_DELTA_PER_STEP,
                    )
                )
                return False
        for name, limit in DESCEND_MAX_TOTAL_JOINT_DELTA.items():
            if total.get(name, 0.0) > limit:
                self.get_logger().warn(
                    "Cartesian descend rejected for %s: %s total delta %.4f > %.4f"
                    % (target, name, total.get(name, 0.0), limit)
                )
                return False
        return True

    def _trajectory_arm_joint_samples(self, trajectory) -> list[dict[str, float]]:
        names = list(getattr(trajectory, "joint_names", []))
        samples = []
        for point in getattr(trajectory, "points", []):
            positions = list(point.positions)
            by_name = {
                name: float(positions[index])
                for index, name in enumerate(names)
                if index < len(positions) and name in UR_ARM_JOINTS
            }
            if by_name:
                samples.append(by_name)
        return samples

    def _log_descend_joint_samples(
        self, target: str, samples: list[dict[str, float]]
    ) -> None:
        start = samples[0]
        mid = samples[len(samples) // 2]
        end = samples[-1]
        self.get_logger().info(
            "DESCEND_JOINT_START %s %s"
            % (target, self._format_arm_joints(start))
        )
        self.get_logger().info(
            "DESCEND_JOINT_MID %s %s"
            % (target, self._format_arm_joints(mid))
        )
        self.get_logger().info(
            "DESCEND_JOINT_END %s %s"
            % (target, self._format_arm_joints(end))
        )
        self.get_logger().info(
            "GRASP_JOINTS %s %s"
            % (target, self._format_arm_joints(end))
        )

    def _max_joint_delta_per_step(
        self, samples: list[dict[str, float]]
    ) -> dict[str, float]:
        result = {name: 0.0 for name in UR_ARM_JOINTS}
        for prev, curr in zip(samples, samples[1:]):
            for name in UR_ARM_JOINTS:
                if name in prev and name in curr:
                    result[name] = max(result[name], abs(curr[name] - prev[name]))
        return result

    def _total_joint_delta(
        self, samples: list[dict[str, float]]
    ) -> dict[str, float]:
        start = samples[0]
        end = samples[-1]
        return {
            name: abs(end[name] - start[name])
            for name in UR_ARM_JOINTS
            if name in start and name in end
        }

    def _log_current_arm_joints(self, label: str, target: str) -> None:
        text = self._joint_positions_text(
            getattr(self.pp.moveit2, "joint_state", None)
        )
        if not text:
            self.get_logger().warn("%s %s unavailable" % (label, target))
            return
        self.get_logger().info("%s %s %s" % (label, target, text))

    def _trajectory_endpoint_joints_text(self, trajectory) -> str:
        if trajectory is None or not getattr(trajectory, "points", None):
            return ""
        names = list(getattr(trajectory, "joint_names", []))
        positions = list(trajectory.points[-1].positions)
        by_name = {
            name: float(positions[index])
            for index, name in enumerate(names)
            if index < len(positions)
        }
        return self._format_arm_joints(by_name)

    def _joint_positions_text(self, joint_state) -> str:
        if joint_state is None:
            return ""
        by_name = {
            name: float(position)
            for name, position in zip(joint_state.name, joint_state.position)
        }
        return self._format_arm_joints(by_name)

    def _format_arm_joints(self, by_name: dict[str, float]) -> str:
        if not by_name:
            return ""
        parts = []
        for name in UR_ARM_JOINTS:
            if name in by_name:
                parts.append("%s=%.4f" % (name, by_name[name]))
        return " ".join(parts)

    def _format_pose_dict(self, pose: dict) -> str:
        return (
            "x=%.4f y=%.4f z=%.4f roll=%.4f pitch=%.4f yaw=%.4f"
            % (
                pose["x"],
                pose["y"],
                pose["z"],
                pose.get("roll", 0.0),
                pose.get("pitch", 0.0),
                pose["yaw"],
            )
        )

    def _log_grasp_z_geometry(
        self, target: str, config: dict, tcp_target_z: float
    ) -> None:
        values = grasp_z_debug_values(config, tcp_target_z)
        ordered_keys = (
            "MODEL_ORIGIN_Z",
            "COLLISION_LOCAL_Z",
            "OBJECT_CENTER_Z",
            "OBJECT_TOP_Z",
            "OBJECT_BOTTOM_Z",
            "EXPECTED_GRASP_Z",
            "TCP_CLEARANCE_Z",
            "TCP_TARGET_Z",
        )
        self.get_logger().info(
            "grasp z geometry %s: %s"
            % (
                target,
                " ".join(
                    f"{key}={values[key]:.4f}" for key in ordered_keys
                ),
            )
        )


def main(args=None):
    rclpy.init(args=args)
    validation_node = GraspValidationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(validation_node)
    executor.add_node(validation_node.pp)
    try:
        executor.spin()
    finally:
        executor.remove_node(validation_node.pp)
        executor.remove_node(validation_node)
        validation_node.pp.destroy_node()
        validation_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
