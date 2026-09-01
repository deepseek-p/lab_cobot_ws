"""Fixed geometry for the test-tube insertion validation task."""
from __future__ import annotations

import math


BASE_LINK_WORLD_Z = 0.155

# Temporary validation-only safe reference pose.  With yaw=pi/2, the rack row
# at world y=-1.95 is 0.69 m in front of base_link.  Offline geometry filtering
# confirms this pose keeps base_link clear of station_b_table; the final tube
# task pose must be chosen by tube_insert_base_feasibility.
TUBE_INSERT_VALIDATION_BASE_POSE = {
    "x": 0.30,
    "y": -2.64,
    "yaw": math.pi / 2.0,
}

TEST_TUBE_RADIUS = 0.011
TEST_TUBE_LIP_RADIUS = 0.013
TEST_TUBE_LENGTH = 0.125
TEST_TUBE_GRASP_HEIGHT = 0.083
TEST_TUBE_PRE_GRASP_CLEARANCE = 0.095
TEST_TUBE_LIFT_CLEARANCE = 0.100

GRIPPER_JOINT_MIN = 0.0
GRIPPER_JOINT_MAX = 0.035
GRIPPER_OPEN_INNER_GAP = 0.092
TUBE_PRE_OPEN_GAP = 0.034
TUBE_POST_INSERT_RELEASE_GAP = 0.040
TUBE_TACTILE_STEP = 0.00025
TUBE_TACTILE_MAX_POSITION = 0.034


def symmetric_gripper_command_for_gap(gap_m: float) -> float:
    """Return one finger joint command for a symmetric target inner gap."""
    return (
        GRIPPER_OPEN_INNER_GAP - float(gap_m)
    ) / 2.0


TUBE_PRE_GRASP_POSITION = symmetric_gripper_command_for_gap(TUBE_PRE_OPEN_GAP)
TUBE_POST_INSERT_RELEASE_POSITION = symmetric_gripper_command_for_gap(
    TUBE_POST_INSERT_RELEASE_GAP
)
TUBE_TACTILE_START_POSITION = TUBE_PRE_GRASP_POSITION
TEST_TUBE_DIAMETER = 2.0 * TEST_TUBE_RADIUS
TUBE_THEORETICAL_CONTACT_POSITION = symmetric_gripper_command_for_gap(
    TEST_TUBE_DIAMETER
)

RACK_SLOT_LOCAL_X = (-0.12, -0.06, 0.0, 0.06, 0.12)
RACK_SLOT_LOCAL_Y = 0.0
RACK_BASE_TOP_Z = 0.012
RACK_SLOT_MOUTH_Z = 0.064
RACK_INSERT_PRE_CLEARANCE = 0.120
RACK_INSERT_STAGE1_CLEARANCE = 0.050
RACK_INSERT_STAGE2_CLEARANCE = 0.020


def world_to_base(point: dict, base_pose: dict | None = None) -> dict:
    """Transform a world/map point into the validation base_link frame."""
    base = TUBE_INSERT_VALIDATION_BASE_POSE if base_pose is None else base_pose
    dx = float(point["x"]) - float(base["x"])
    dy = float(point["y"]) - float(base["y"])
    yaw = float(base["yaw"])
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return {
        "x": cos_yaw * dx + sin_yaw * dy,
        "y": -sin_yaw * dx + cos_yaw * dy,
        "z": float(point["z"]) - BASE_LINK_WORLD_Z,
    }


def tcp_pose_from_tube_bottom(bottom_base: dict, grasp_height: float) -> list[float]:
    return [
        float(bottom_base["x"]),
        float(bottom_base["y"]),
        float(bottom_base["z"]) + float(grasp_height),
    ]


TEST_TUBE_1_WORLD_POSE = {
    "x": 0.36,
    "y": -1.95,
    "z": 0.762,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
TEST_TUBE_1_BOTTOM_BASE = world_to_base(TEST_TUBE_1_WORLD_POSE)
TEST_TUBE_1_GRASP_TCP = tcp_pose_from_tube_bottom(
    TEST_TUBE_1_BOTTOM_BASE,
    TEST_TUBE_GRASP_HEIGHT,
)
TEST_TUBE_1_PRE_GRASP_TCP = [
    TEST_TUBE_1_GRASP_TCP[0],
    TEST_TUBE_1_GRASP_TCP[1],
    TEST_TUBE_1_GRASP_TCP[2] + TEST_TUBE_PRE_GRASP_CLEARANCE,
]
TEST_TUBE_1_LIFT_TCP = [
    TEST_TUBE_1_GRASP_TCP[0],
    TEST_TUBE_1_GRASP_TCP[1],
    TEST_TUBE_1_GRASP_TCP[2] + TEST_TUBE_LIFT_CLEARANCE,
]

TEST_TUBE_1 = {
    "entity_name": "test_tube_1",
    "model_uri": "model://test_tube",
    "link_name": "link",
    "world_pose": TEST_TUBE_1_WORLD_POSE,
    "bottom_base": TEST_TUBE_1_BOTTOM_BASE,
    "grasp_height": TEST_TUBE_GRASP_HEIGHT,
    "grasp_tcp": TEST_TUBE_1_GRASP_TCP,
    "pre_grasp_tcp": TEST_TUBE_1_PRE_GRASP_TCP,
    "lift_tcp": TEST_TUBE_1_LIFT_TCP,
}

TEST_TUBE_RACK_2_WORLD_POSE = {
    "x": 0.12,
    "y": -1.95,
    "z": 0.75,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
TEST_TUBE_RACK_2_MIDDLE_SLOT_WORLD = {
    "x": TEST_TUBE_RACK_2_WORLD_POSE["x"] + RACK_SLOT_LOCAL_X[2],
    "y": TEST_TUBE_RACK_2_WORLD_POSE["y"] + RACK_SLOT_LOCAL_Y,
    "z": TEST_TUBE_RACK_2_WORLD_POSE["z"] + RACK_BASE_TOP_Z,
}
TEST_TUBE_RACK_2_SLOT_MOUTH_WORLD = {
    "x": TEST_TUBE_RACK_2_MIDDLE_SLOT_WORLD["x"],
    "y": TEST_TUBE_RACK_2_MIDDLE_SLOT_WORLD["y"],
    "z": TEST_TUBE_RACK_2_WORLD_POSE["z"] + RACK_SLOT_MOUTH_Z,
}
TEST_TUBE_RACK_2_MIDDLE_SLOT_BASE = world_to_base(
    TEST_TUBE_RACK_2_MIDDLE_SLOT_WORLD
)
TEST_TUBE_RACK_2_SLOT_MOUTH_BASE = world_to_base(
    TEST_TUBE_RACK_2_SLOT_MOUTH_WORLD
)


def insert_tcp(clearance_above_mouth: float) -> list[float]:
    bottom = {
        "x": TEST_TUBE_RACK_2_SLOT_MOUTH_BASE["x"],
        "y": TEST_TUBE_RACK_2_SLOT_MOUTH_BASE["y"],
        "z": TEST_TUBE_RACK_2_SLOT_MOUTH_BASE["z"] + float(clearance_above_mouth),
    }
    return tcp_pose_from_tube_bottom(bottom, TEST_TUBE_GRASP_HEIGHT)


TEST_TUBE_RACK_2 = {
    "entity_name": "test_tube_rack_2",
    "model_uri": "model://test_tube_rack",
    "link_name": "link",
    "world_pose": TEST_TUBE_RACK_2_WORLD_POSE,
    "slot_mouth_world": TEST_TUBE_RACK_2_SLOT_MOUTH_WORLD,
    "middle_slot_center_base": TEST_TUBE_RACK_2_MIDDLE_SLOT_BASE,
    "slot_mouth_base": TEST_TUBE_RACK_2_SLOT_MOUTH_BASE,
    "insert_depth": RACK_SLOT_MOUTH_Z - RACK_BASE_TOP_Z,
    "pre_insert_tcp": insert_tcp(RACK_INSERT_PRE_CLEARANCE),
    "insert_stage1_tcp": insert_tcp(RACK_INSERT_STAGE1_CLEARANCE),
    "insert_stage2_tcp": insert_tcp(RACK_INSERT_STAGE2_CLEARANCE),
    "insert_final_tcp": tcp_pose_from_tube_bottom(
        TEST_TUBE_RACK_2_MIDDLE_SLOT_BASE,
        TEST_TUBE_GRASP_HEIGHT,
    ),
}

TUBE_INSERT_CONFIG = {
    "command": "insert_test_tube",
    "target_topic": "/tube_insert_validation/target",
    "status_topic": "/tube_insert_validation/status",
    "test_tube_1": TEST_TUBE_1,
    "test_tube_rack_2": TEST_TUBE_RACK_2,
    "fixed_tcp_quat_xyzw": [1.0, 0.0, 0.0, 0.0],
}
