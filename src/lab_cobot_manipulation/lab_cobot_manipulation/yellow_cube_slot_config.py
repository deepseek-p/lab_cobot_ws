"""Geometry and task constants for yellow cube slot validation.

All poses are derived from lab.world and the referenced SDF models.  This module
is intentionally independent from the aging-rack validation task.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Pose3D:
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Box:
    center: tuple[float, float, float]
    size: tuple[float, float, float]


TARGET_OBJECT = "material_cube_yellow"
COMMAND = "insert_yellow_cube"
TARGET_TOPIC = "/yellow_cube_slot_validation/target"
STATUS_TOPIC = "/yellow_cube_slot_validation/status"

STATUS_SEQUENCE = (
    "YELLOW_SLOT_TASK_READY",
    "START",
    "SKIP_INITIAL_NAV_STATION_A",
    "STATION_A_FINE_DOCK_START",
    "STATION_A_FINE_DOCK_DONE",
    "PRE_GRASP_HIGH",
    "PRE_GRASP",
    "DESCEND_GRASP",
    "GRIPPER_CLOSE",
    "ATTACHED",
    "HOLDING",
    "LIFT",
    "ARM_TRANSPORT_SAFE",
    "STATION_A_RETREAT_START",
    "STATION_A_RETREAT_SAFE",
    "NAV_AGING_ZONE",
    "NAV_AGING_ZONE_SUCCESS",
    "PRE_SLOT_HIGH",
    "SLOT_ALIGNMENT_ROTATE_START",
    "SLOT_ALIGNMENT_ROTATE_DONE",
    "PRE_SLOT",
    "INSERT_STAGE1",
    "INSERT_STAGE2",
    "INSERT_FINAL",
    "RELEASE",
    "DETACHED",
    "SETTLE",
    "YELLOW_CUBE_SLOT_PLACE_VALID",
    "VERTICAL_RETREAT",
    "YELLOW_CUBE_SLOT_SUCCESS",
)

FAILURE_STATES = (
    "FAILED_PRE_GRASP",
    "FAILED_STATION_A_FINE_DOCK",
    "FAILED_DESCEND_GRASP",
    "FAILED_GRASP",
    "FAILED_HOLDING",
    "FAILED_LIFT",
    "FAILED_TRANSPORT_SAFE",
    "FAILED_STATION_A_RETREAT_NAV_VALIDATION",
    "FAILED_NAV_AGING_ZONE",
    "FAILED_PRE_SLOT",
    "FAILED_SLOT_ALIGNMENT_ROTATE",
    "FAILED_INSERT_STAGE1",
    "FAILED_INSERT_STAGE2",
    "FAILED_INSERT_FINAL",
    "FAILED_RELEASE",
    "FAILED_PLACE_VALIDATION",
    "FAILED_VERTICAL_RETREAT",
)

# The yellow validation starts at the aging-rack core debug safe spawn.
# It fine-docks to Station A after target receipt, before arm grasp.
YELLOW_CUBE_SLOT_VALIDATION_BASE_POSE = {
    "x": -4.30,
    "y": 2.745,
    "yaw": math.pi / 2.0,
}

BASE_LINK_WORLD_Z = 0.155
DOWN_QUAT_XYZW = [1.0, 0.0, 0.0, 0.0]
SLOT_QUAT_XYZW = DOWN_QUAT_XYZW

STATION_A_TABLE_WORLD = Box(center=(-4.30, 3.80, 0.375), size=(1.60, 1.20, 0.75))
AGING_ZONE_TABLE_WORLD = Box(center=(0.20, 4.20, 0.375), size=(1.60, 1.20, 0.75))

MATERIAL_CUBE_YELLOW_WORLD_POSE = Pose3D(x=-4.30, y=3.52, z=0.785)
MATERIAL_CUBE_YELLOW_COLLISION_POSE = Pose3D(x=0.0, y=0.0, z=0.0)
MATERIAL_CUBE_YELLOW_COLLISION_SIZE = (0.070, 0.070, 0.070)

AGING_RACK_WORLD_POSE = Pose3D(x=0.20, y=3.88, z=0.80)
AGING_RACK_COLLISIONS = {
    "bottom": Box(center=(0.0, 0.0, -0.04), size=(0.34, 0.22, 0.02)),
    "front_wall": Box(center=(0.0, 0.10, 0.01), size=(0.34, 0.02, 0.08)),
    "back_wall": Box(center=(0.0, -0.10, 0.01), size=(0.34, 0.02, 0.08)),
    "left_wall": Box(center=(-0.16, 0.0, 0.01), size=(0.02, 0.18, 0.08)),
    "right_wall": Box(center=(0.16, 0.0, 0.01), size=(0.02, 0.18, 0.08)),
    "divider_left": Box(center=(-0.0525, 0.0, 0.01), size=(0.015, 0.18, 0.08)),
    "divider_right": Box(center=(0.0525, 0.0, 0.01), size=(0.015, 0.18, 0.08)),
}

MIDDLE_SLOT_INDEX = 1
MIDDLE_SLOT_LEFT_BOUNDARY_LOCAL_X = -0.045
MIDDLE_SLOT_RIGHT_BOUNDARY_LOCAL_X = 0.045
MIDDLE_SLOT_BACK_BOUNDARY_LOCAL_Y = -0.090
MIDDLE_SLOT_FRONT_BOUNDARY_LOCAL_Y = 0.090
MIDDLE_SLOT_BOTTOM_LOCAL_Z = -0.030
MIDDLE_SLOT_TOP_LOCAL_Z = 0.050
MIDDLE_SLOT_WIDTH = (
    MIDDLE_SLOT_RIGHT_BOUNDARY_LOCAL_X - MIDDLE_SLOT_LEFT_BOUNDARY_LOCAL_X
)
MIDDLE_SLOT_DEPTH = (
    MIDDLE_SLOT_FRONT_BOUNDARY_LOCAL_Y - MIDDLE_SLOT_BACK_BOUNDARY_LOCAL_Y
)
MIDDLE_SLOT_HEIGHT = MIDDLE_SLOT_TOP_LOCAL_Z - MIDDLE_SLOT_BOTTOM_LOCAL_Z

BOTTOM_CLEARANCE = 0.003
GRASP_TCP_Z_CLEARANCE = 0.018
PRE_GRASP_CLEARANCE = 0.095
PRE_GRASP_HIGH_EXTRA = 0.080
LIFT_CLEARANCE = 0.110
PRE_SLOT_CLEARANCE = 0.120
PRE_SLOT_HIGH_EXTRA = 0.060
SHALLOW_RELEASE_BOTTOM_CLEARANCE = 0.002
DEEP_INSERT_STAGE1_CLEARANCE = 0.060
DEEP_INSERT_STAGE2_CLEARANCE = 0.025
VERTICAL_RETREAT_CLEARANCE = PRE_SLOT_CLEARANCE

CARTESIAN_EEF_STEP = 0.005
CARTESIAN_FRACTION_MIN = 0.999

PLACEMENT_STRATEGY = "ROTATED_DEEP_INSERT"
YELLOW_NAV_ACTIVE_TIMEOUT_SEC = 20.0
YELLOW_NAV_TF_READY_TIMEOUT_SEC = 12.0
YELLOW_STATION_DOCK_MAX_SEC = 80.0
YELLOW_STATION_DOCK_SETTLE_SEC = 0.3
YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE_X = 0.720
YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE_Y = 0.000
YELLOW_STATION_A_FINE_DOCK_X_TOLERANCE = 0.020
YELLOW_STATION_A_FINE_DOCK_Y_TOLERANCE = 0.040
YELLOW_STATION_A_FINE_DOCK_MIN_SAFE_X = 0.660
YELLOW_STATION_A_FINE_DOCK_MAX_LINEAR_SPEED = 0.100
YELLOW_STATION_A_FINE_DOCK_LINEAR_GAIN = 0.70
YELLOW_STATION_A_FINE_DOCK_TIMEOUT_SEC = 35.0
YELLOW_STATION_A_FINE_DOCK_STALLED_SEC = 8.0
YELLOW_STATION_A_FINE_DOCK_PROGRESS_EPS = 0.004
SLOT_ALIGNMENT_YAW = -math.pi / 2.0
SLOT_ALIGNMENT_QUAT_XYZW = [
    math.cos(SLOT_ALIGNMENT_YAW / 2.0),
    math.sin(SLOT_ALIGNMENT_YAW / 2.0),
    0.0,
    0.0,
]

GRIPPER_OPEN_INNER_GAP = 0.092
GRIPPER_JOINT_MAX = 0.035
YELLOW_CUBE_TACTILE_START_POSITION = 0.0
YELLOW_CUBE_TACTILE_STEP_POSITION = 0.00025
YELLOW_CUBE_THEORETICAL_CONTACT_Q = (
    GRIPPER_OPEN_INNER_GAP - MATERIAL_CUBE_YELLOW_COLLISION_SIZE[1]
) / 2.0
YELLOW_CUBE_TACTILE_MAX_POSITION = min(
    GRIPPER_JOINT_MAX,
    YELLOW_CUBE_THEORETICAL_CONTACT_Q + 0.003,
)

PLACE_SETTLE_SEC = 1.0
PLACE_VALIDATION_XY_TOLERANCE = 0.008
PLACE_VALIDATION_Z_TOLERANCE = 0.012
PLACE_VALIDATION_YAW_TOLERANCE = 0.15

GRIPPER_FINGER_COLLISION_SIZE = (0.045, 0.012, 0.075)
GRIPPER_TACTILE_PROBE_COLLISION_SIZE = (0.045, 0.012, 0.120)
GRIPPER_BASE_VISUAL_SIZE = (0.045, 0.130, 0.040)
GRIPPER_LEFT_FINGER_JOINT_ORIGIN_Y = 0.052
GRIPPER_RIGHT_FINGER_JOINT_ORIGIN_Y = -0.052
GRIPPER_TCP_FROM_TOOL0_Z = 0.105
GRIPPER_FINGER_CENTER_FROM_TOOL0_Z = 0.022 + 0.0575
GRIPPER_BASE_CENTER_FROM_TOOL0_Z = 0.022


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def local_to_world(
    local: tuple[float, float, float],
    parent_pose: Pose3D = AGING_RACK_WORLD_POSE,
) -> tuple[float, float, float]:
    rx, ry = _rotate_xy(local[0], local[1], parent_pose.yaw)
    return parent_pose.x + rx, parent_pose.y + ry, parent_pose.z + local[2]


def world_to_base(
    world: tuple[float, float, float],
    base_pose: Pose2D | dict[str, float],
    base_link_world_z: float = BASE_LINK_WORLD_Z,
) -> list[float]:
    if isinstance(base_pose, dict):
        base_x = float(base_pose["x"])
        base_y = float(base_pose["y"])
        base_yaw = float(base_pose["yaw"])
    else:
        base_x = base_pose.x
        base_y = base_pose.y
        base_yaw = base_pose.yaw
    dx = float(world[0]) - base_x
    dy = float(world[1]) - base_y
    c = math.cos(-base_yaw)
    s = math.sin(-base_yaw)
    return [
        c * dx - s * dy,
        s * dx + c * dy,
        float(world[2]) - base_link_world_z,
    ]


def cube_center_world() -> tuple[float, float, float]:
    return (
        MATERIAL_CUBE_YELLOW_WORLD_POSE.x + MATERIAL_CUBE_YELLOW_COLLISION_POSE.x,
        MATERIAL_CUBE_YELLOW_WORLD_POSE.y + MATERIAL_CUBE_YELLOW_COLLISION_POSE.y,
        MATERIAL_CUBE_YELLOW_WORLD_POSE.z + MATERIAL_CUBE_YELLOW_COLLISION_POSE.z,
    )


def cube_top_world_z() -> float:
    return cube_center_world()[2] + MATERIAL_CUBE_YELLOW_COLLISION_SIZE[2] / 2.0


def grasp_tcp_world() -> tuple[float, float, float]:
    center = cube_center_world()
    return center[0], center[1], center[2] + GRASP_TCP_Z_CLEARANCE


def pre_grasp_tcp_world() -> tuple[float, float, float]:
    target = grasp_tcp_world()
    return target[0], target[1], target[2] + PRE_GRASP_CLEARANCE


def pre_grasp_high_tcp_world() -> tuple[float, float, float]:
    target = pre_grasp_tcp_world()
    return target[0], target[1], target[2] + PRE_GRASP_HIGH_EXTRA


def lift_tcp_world() -> tuple[float, float, float]:
    target = grasp_tcp_world()
    return target[0], target[1], target[2] + LIFT_CLEARANCE


def middle_slot_center_world() -> tuple[float, float, float]:
    local = (
        (MIDDLE_SLOT_LEFT_BOUNDARY_LOCAL_X + MIDDLE_SLOT_RIGHT_BOUNDARY_LOCAL_X) / 2.0,
        (MIDDLE_SLOT_BACK_BOUNDARY_LOCAL_Y + MIDDLE_SLOT_FRONT_BOUNDARY_LOCAL_Y) / 2.0,
        MIDDLE_SLOT_BOTTOM_LOCAL_Z
        + MATERIAL_CUBE_YELLOW_COLLISION_SIZE[2] / 2.0
        + BOTTOM_CLEARANCE,
    )
    return local_to_world(local)


def middle_slot_bounds_world() -> dict[str, tuple[float, float]]:
    return {
        "x": (
            AGING_RACK_WORLD_POSE.x + MIDDLE_SLOT_LEFT_BOUNDARY_LOCAL_X,
            AGING_RACK_WORLD_POSE.x + MIDDLE_SLOT_RIGHT_BOUNDARY_LOCAL_X,
        ),
        "y": (
            AGING_RACK_WORLD_POSE.y + MIDDLE_SLOT_BACK_BOUNDARY_LOCAL_Y,
            AGING_RACK_WORLD_POSE.y + MIDDLE_SLOT_FRONT_BOUNDARY_LOCAL_Y,
        ),
        "z": (
            AGING_RACK_WORLD_POSE.z + MIDDLE_SLOT_BOTTOM_LOCAL_Z,
            AGING_RACK_WORLD_POSE.z + MIDDLE_SLOT_TOP_LOCAL_Z,
        ),
    }


def slot_tcp_world(z_clearance: float) -> tuple[float, float, float]:
    center = middle_slot_center_world()
    return center[0], center[1], center[2] + GRASP_TCP_Z_CLEARANCE + z_clearance


def pre_slot_high_tcp_world() -> tuple[float, float, float]:
    return slot_tcp_world(PRE_SLOT_CLEARANCE + PRE_SLOT_HIGH_EXTRA)


def pre_slot_tcp_world() -> tuple[float, float, float]:
    return slot_tcp_world(PRE_SLOT_CLEARANCE)


def insert_stage1_tcp_world() -> tuple[float, float, float]:
    return slot_tcp_world(DEEP_INSERT_STAGE1_CLEARANCE)


def insert_stage2_tcp_world() -> tuple[float, float, float]:
    return slot_tcp_world(DEEP_INSERT_STAGE2_CLEARANCE)


def insert_final_tcp_world() -> tuple[float, float, float]:
    return slot_tcp_world(0.0)


def vertical_retreat_tcp_world() -> tuple[float, float, float]:
    return slot_tcp_world(VERTICAL_RETREAT_CLEARANCE)


def cube_slot_clearance() -> dict[str, float]:
    return {
        "width": MIDDLE_SLOT_WIDTH - MATERIAL_CUBE_YELLOW_COLLISION_SIZE[0],
        "depth": MIDDLE_SLOT_DEPTH - MATERIAL_CUBE_YELLOW_COLLISION_SIZE[1],
        "height": MIDDLE_SLOT_HEIGHT - MATERIAL_CUBE_YELLOW_COLLISION_SIZE[2],
    }


def cube_slot_per_side_clearance() -> dict[str, float]:
    clearance = cube_slot_clearance()
    return {
        "width": clearance["width"] / 2.0,
        "depth": clearance["depth"] / 2.0,
        "height_top": clearance["height"] - BOTTOM_CLEARANCE,
    }


def gripper_finger_position_for_cube() -> float:
    return YELLOW_CUBE_THEORETICAL_CONTACT_Q


def gripper_outer_width_while_holding_cube() -> float:
    q = gripper_finger_position_for_cube()
    left_center_y = GRIPPER_LEFT_FINGER_JOINT_ORIGIN_Y - q
    right_center_y = GRIPPER_RIGHT_FINGER_JOINT_ORIGIN_Y + q
    half_thickness = GRIPPER_FINGER_COLLISION_SIZE[1] / 2.0
    return (left_center_y + half_thickness) - (right_center_y - half_thickness)


def gripper_cube_unrotated_slot_width_extent() -> float:
    return max(
        MATERIAL_CUBE_YELLOW_COLLISION_SIZE[0],
        gripper_outer_width_while_holding_cube(),
    )


def gripper_cube_unrotated_slot_depth_extent() -> float:
    return max(
        MATERIAL_CUBE_YELLOW_COLLISION_SIZE[1],
        GRIPPER_FINGER_COLLISION_SIZE[0],
        GRIPPER_TACTILE_PROBE_COLLISION_SIZE[0],
    )


def gripper_cube_rotated_slot_width_extent() -> float:
    return gripper_cube_unrotated_slot_depth_extent()


def gripper_cube_rotated_slot_depth_extent() -> float:
    return gripper_cube_unrotated_slot_width_extent()


def deep_insert_physically_possible() -> bool:
    return (
        gripper_cube_rotated_slot_width_extent() <= MIDDLE_SLOT_WIDTH
        and gripper_cube_rotated_slot_depth_extent() <= MIDDLE_SLOT_DEPTH
        and gripper_tactile_probe_bottom_world_z_at_deep_final()
        >= slot_inner_bottom_world_z()
    )


def recommended_place_xy_tolerance() -> float:
    return min(0.008, max(0.0, cube_slot_per_side_clearance()["width"] - 0.002))


def slot_inner_bottom_world_z() -> float:
    return AGING_RACK_WORLD_POSE.z + MIDDLE_SLOT_BOTTOM_LOCAL_Z


def slot_top_world_z() -> float:
    return AGING_RACK_WORLD_POSE.z + MIDDLE_SLOT_TOP_LOCAL_Z


def desired_cube_bottom_world_z() -> float:
    return middle_slot_center_world()[2] - MATERIAL_CUBE_YELLOW_COLLISION_SIZE[2] / 2.0


def gripper_feature_world_z_span_at_tcp(
    tcp_world_z: float,
    center_from_tool0_z: float,
    feature_z_size: float,
) -> tuple[float, float]:
    center_world_z = (
        float(tcp_world_z) + GRIPPER_TCP_FROM_TOOL0_Z - float(center_from_tool0_z)
    )
    half = float(feature_z_size) / 2.0
    return center_world_z - half, center_world_z + half


def gripper_finger_world_z_span_at_deep_final() -> tuple[float, float]:
    return gripper_feature_world_z_span_at_tcp(
        insert_final_tcp_world()[2],
        GRIPPER_FINGER_CENTER_FROM_TOOL0_Z,
        GRIPPER_FINGER_COLLISION_SIZE[2],
    )


def gripper_tactile_probe_world_z_span_at_deep_final() -> tuple[float, float]:
    return gripper_feature_world_z_span_at_tcp(
        insert_final_tcp_world()[2],
        GRIPPER_FINGER_CENTER_FROM_TOOL0_Z,
        GRIPPER_TACTILE_PROBE_COLLISION_SIZE[2],
    )


def gripper_tactile_probe_bottom_world_z_at_deep_final() -> float:
    return gripper_tactile_probe_world_z_span_at_deep_final()[0]


def gripper_body_visual_world_z_span_at_deep_final() -> tuple[float, float]:
    return gripper_feature_world_z_span_at_tcp(
        insert_final_tcp_world()[2],
        GRIPPER_BASE_CENTER_FROM_TOOL0_Z,
        GRIPPER_BASE_VISUAL_SIZE[2],
    )


def rotated_deep_insert_clearance() -> dict[str, float]:
    width = MIDDLE_SLOT_WIDTH - gripper_cube_rotated_slot_width_extent()
    depth = MIDDLE_SLOT_DEPTH - gripper_cube_rotated_slot_depth_extent()
    return {
        "width_total": width,
        "width_per_side": width / 2.0,
        "depth_total": depth,
        "depth_per_side": depth / 2.0,
    }


def stage_collision_check(stage: str) -> dict[str, bool | float]:
    targets = {
        "INSERT_STAGE1": insert_stage1_tcp_world(),
        "INSERT_STAGE2": insert_stage2_tcp_world(),
        "INSERT_FINAL": insert_final_tcp_world(),
    }
    tcp = targets[stage]
    cube_center_z = tcp[2] - GRASP_TCP_Z_CLEARANCE
    cube_bottom = cube_center_z - MATERIAL_CUBE_YELLOW_COLLISION_SIZE[2] / 2.0
    probe_bottom = gripper_feature_world_z_span_at_tcp(
        tcp[2],
        GRIPPER_FINGER_CENTER_FROM_TOOL0_Z,
        GRIPPER_TACTILE_PROBE_COLLISION_SIZE[2],
    )[0]
    clearances = rotated_deep_insert_clearance()
    return {
        "footprint_ok": (
            clearances["width_total"] >= 0.0
            and clearances["depth_total"] >= 0.0
        ),
        "cube_bottom_above_inner_bottom": cube_bottom >= slot_inner_bottom_world_z(),
        "probe_bottom_above_inner_bottom": probe_bottom >= slot_inner_bottom_world_z(),
        "cube_bottom": cube_bottom,
        "probe_bottom": probe_bottom,
    }


def shallow_release_cube_center_world() -> tuple[float, float, float]:
    center = middle_slot_center_world()
    return (
        center[0],
        center[1],
        slot_top_world_z()
        + MATERIAL_CUBE_YELLOW_COLLISION_SIZE[2] / 2.0
        + SHALLOW_RELEASE_BOTTOM_CLEARANCE,
    )


def insert_shallow_tcp_world() -> tuple[float, float, float]:
    center = shallow_release_cube_center_world()
    return center[0], center[1], center[2] + GRASP_TCP_Z_CLEARANCE
