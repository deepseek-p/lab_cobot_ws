#!/usr/bin/env python3
"""Tube insertion base-pose feasibility diagnostic.

This script is intentionally separate from tube_insert_validation_node.py:
it searches candidate spawn poses and reports why a candidate is rejected
without executing any robot trajectory.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import Counter
from dataclasses import dataclass

try:
    from geometry_msgs.msg import Pose, PoseStamped
    from moveit_msgs.msg import (
        CollisionObject,
        Constraints,
        OrientationConstraint,
        PositionConstraint,
        RobotState,
    )
    from moveit_msgs.srv import (
        ApplyPlanningScene,
        GetCartesianPath,
        GetMotionPlan,
        GetPositionFK,
        GetPositionIK,
        GetStateValidity,
    )
    import rclpy
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
except ModuleNotFoundError:
    Pose = PoseStamped = CollisionObject = RobotState = None
    Constraints = OrientationConstraint = PositionConstraint = None
    ApplyPlanningScene = GetCartesianPath = GetPositionFK = None
    GetMotionPlan = GetPositionIK = GetStateValidity = None
    JointState = SolidPrimitive = None
    rclpy = None
    remove_ros_args = None
    Node = object

from lab_cobot_bringup.tube_insert_config import (
    BASE_LINK_WORLD_Z,
    RACK_INSERT_PRE_CLEARANCE,
    RACK_INSERT_STAGE1_CLEARANCE,
    RACK_INSERT_STAGE2_CLEARANCE,
    TEST_TUBE_1_WORLD_POSE,
    TEST_TUBE_GRASP_HEIGHT,
    TEST_TUBE_LENGTH,
    TEST_TUBE_LIFT_CLEARANCE,
    TEST_TUBE_PRE_GRASP_CLEARANCE,
    TEST_TUBE_RADIUS,
    TEST_TUBE_RACK_2_SLOT_MOUTH_WORLD,
    TEST_TUBE_RACK_2_WORLD_POSE,
    TUBE_INSERT_CONFIG,
    TUBE_INSERT_VALIDATION_BASE_POSE,
    tcp_pose_from_tube_bottom,
    world_to_base,
)
try:
    from lab_cobot_manipulation.pick_place_node import (
        GRIPPER_TCP_LINK,
        HOME_CONFIG,
        UR_JOINTS,
    )
except ModuleNotFoundError:
    GRIPPER_TCP_LINK = "gripper_tcp"
    HOME_CONFIG = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
    UR_JOINTS = [
        "ur_shoulder_pan_joint",
        "ur_shoulder_lift_joint",
        "ur_elbow_joint",
        "ur_wrist_1_joint",
        "ur_wrist_2_joint",
        "ur_wrist_3_joint",
    ]

try:
    from lab_cobot_moveit.tube_insert_scene import (
        build_tube_insert_planning_scene,
        make_attach_test_tube_scene,
        make_detach_test_tube_scene,
    )
except ModuleNotFoundError:
    build_tube_insert_planning_scene = None
    make_attach_test_tube_scene = None
    make_detach_test_tube_scene = None


STATION_B_TABLE = {
    "name": "station_b_table",
    "center": (0.30, -1.70, 0.375),
    "size": (1.6, 1.2, 0.75),
    "yaw": 0.0,
}
RACK_SIZE = (0.31, 0.10, 0.064)
RACK1_WORLD_POSE = {"x": 0.48, "y": -1.95, "z": 0.75, "yaw": 0.0}
RACK2_WORLD_POSE = TEST_TUBE_RACK_2_WORLD_POSE
BASE_BODY_SIZE = (0.55, 0.50, 0.15)
BASE_BODY_CENTER_Z_WORLD = BASE_LINK_WORLD_Z
PILLAR_SIZE = (0.12, 0.12, 0.30)
PILLAR_CENTER_Z_WORLD = BASE_LINK_WORLD_Z + 0.075 + 0.15
TUBE_FIXED_QUAT_XYZW = tuple(TUBE_INSERT_CONFIG["fixed_tcp_quat_xyzw"])
HIGH_APPROACH_Z_VALUES = (0.82, 0.84, 0.86, 0.88, 0.90)
ARM_CLEARANCE_SAMPLE_COUNT = 20
PROTECTED_ARM_LINK_GEOMETRY_HALF_Z = {
    "ur_upper_arm_link": 0.08,
    "ur_forearm_link": 0.07,
    "ur_wrist_1_link": 0.06,
    "ur_wrist_2_link": 0.06,
    "ur_wrist_3_link": 0.06,
}
ARM_LINK_TABLE_SAFETY_MARGIN = {
    "ur_upper_arm_link": -0.18,
    "ur_forearm_link": 0.05,
    "ur_wrist_1_link": 0.05,
    "ur_wrist_2_link": 0.05,
    "ur_wrist_3_link": 0.05,
}


@dataclass(frozen=True)
class BasePose:
    x: float
    y: float
    yaw: float


@dataclass
class CandidateResult:
    pose: BasePose
    spawn_safe: bool
    spawn_min_clearance: float
    spawn_reject_reason: str = ""
    failure_reason: str = ""
    ik_all: bool = False
    trajectory_safe: bool = False
    min_arm_table_clearance: float | None = None
    closest_link: str = ""
    score: float = -1.0


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    half = float(yaw) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def world_to_candidate_base(point: dict, pose: BasePose) -> dict:
    return world_to_base(
        point,
        {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
    )


def rotate_point(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def oriented_rect_corners(
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
    yaw: float,
) -> list[tuple[float, float]]:
    hx = size_x * 0.5
    hy = size_y * 0.5
    corners = []
    for lx, ly in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy)):
        rx, ry = rotate_point(lx, ly, yaw)
        corners.append((center_x + rx, center_y + ry))
    return corners


def project(points: list[tuple[float, float]], axis: tuple[float, float]):
    values = [x * axis[0] + y * axis[1] for x, y in points]
    return min(values), max(values)


def rect_clearance_or_penetration(
    a_corners: list[tuple[float, float]],
    b_corners: list[tuple[float, float]],
) -> float:
    axes = []
    for corners in (a_corners, b_corners):
        for i in range(2):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % len(corners)]
            ex, ey = x2 - x1, y2 - y1
            length = math.hypot(ex, ey)
            if length <= 1e-9:
                continue
            axes.append((-ey / length, ex / length))
    min_overlap = float("inf")
    max_gap = 0.0
    separated = False
    for axis in axes:
        amin, amax = project(a_corners, axis)
        bmin, bmax = project(b_corners, axis)
        gap = max(amin - bmax, bmin - amax)
        if gap > 0.0:
            separated = True
            max_gap = max(max_gap, gap)
        else:
            min_overlap = min(min_overlap, min(amax, bmax) - max(amin, bmin))
    return max_gap if separated else -min_overlap


def box_bounds(center, size):
    return tuple(
        (
            float(center[0]) - float(size[0]) / 2.0,
            float(center[0]) + float(size[0]) / 2.0,
            float(center[1]) - float(size[1]) / 2.0,
            float(center[1]) + float(size[1]) / 2.0,
            float(center[2]) - float(size[2]) / 2.0,
            float(center[2]) + float(size[2]) / 2.0,
        )
    )


def print_world_geometry() -> None:
    table_bounds = box_bounds(STATION_B_TABLE["center"], STATION_B_TABLE["size"])
    rack1_center = (RACK1_WORLD_POSE["x"], RACK1_WORLD_POSE["y"], 0.75 + 0.032)
    rack2_center = (RACK2_WORLD_POSE["x"], RACK2_WORLD_POSE["y"], 0.75 + 0.032)
    rack1_bounds = box_bounds(rack1_center, RACK_SIZE)
    rack2_bounds = box_bounds(rack2_center, RACK_SIZE)
    tube_bottom = TEST_TUBE_1_WORLD_POSE
    tube_center = (
        tube_bottom["x"],
        tube_bottom["y"],
        tube_bottom["z"] + TEST_TUBE_LENGTH / 2.0,
    )
    tube_top = (
        tube_bottom["x"],
        tube_bottom["y"],
        tube_bottom["z"] + TEST_TUBE_LENGTH,
    )
    insert = TEST_TUBE_RACK_2_SLOT_MOUTH_WORLD
    print("TABLE_WORLD_BOUNDS name=station_b_table bounds=%s" % (table_bounds,))
    print("RACK1_WORLD_BOUNDS name=test_tube_rack_1 bounds=%s" % (rack1_bounds,))
    print("RACK2_WORLD_BOUNDS name=test_tube_rack_2 bounds=%s" % (rack2_bounds,))
    print(
        "TUBE_PICK_WORLD_POINT x=%.4f y=%.4f z=%.4f"
        % (tube_bottom["x"], tube_bottom["y"], tube_bottom["z"])
    )
    print(
        "TUBE_GEOMETRY radius=%.4f length=%.4f center=(%.4f,%.4f,%.4f) "
        "top=(%.4f,%.4f,%.4f) grasp_height=%.4f"
        % (
            TEST_TUBE_RADIUS,
            TEST_TUBE_LENGTH,
            tube_center[0],
            tube_center[1],
            tube_center[2],
            tube_top[0],
            tube_top[1],
            tube_top[2],
            TEST_TUBE_GRASP_HEIGHT,
        )
    )
    print(
        "TUBE_INSERT_WORLD_POINT x=%.4f y=%.4f z=%.4f"
        % (insert["x"], insert["y"], insert["z"])
    )
    print(
        "INSERT_SLOT_WORLD_AXIS x=0.0000 y=0.0000 z=-1.0000 "
        "opening_half_width_y=0.0220 opening_pitch_x=0.0600 "
        "required_insertion_depth=%.4f"
        % (0.064 - 0.012)
    )


def candidate_grid() -> list[BasePose]:
    xs = [round(0.15 + 0.05 * i, 4) for i in range(7)]
    ys = [round(-2.70 + 0.02 * i, 4) for i in range(11)]
    yaw_offsets = [-20, -15, -10, -5, 0, 5, 10, 15, 20]
    return [
        BasePose(x, y, math.pi / 2.0 + math.radians(deg))
        for x in xs
        for y in ys
        for deg in yaw_offsets
    ]


def spawn_geometry_filter(pose: BasePose) -> CandidateResult:
    table = STATION_B_TABLE
    table_rect = oriented_rect_corners(
        table["center"][0],
        table["center"][1],
        table["size"][0],
        table["size"][1],
        table["yaw"],
    )
    base_rect = oriented_rect_corners(
        pose.x,
        pose.y,
        BASE_BODY_SIZE[0],
        BASE_BODY_SIZE[1],
        pose.yaw,
    )
    clearance = rect_clearance_or_penetration(base_rect, table_rect)
    result = CandidateResult(
        pose=pose,
        spawn_safe=clearance >= 0.0,
        spawn_min_clearance=clearance,
    )
    if clearance < 0.0:
        result.spawn_reject_reason = (
            "SPAWN_REJECTED_COLLISION robot_link=base_link "
            "environment_object=station_b_table penetration=%.4f"
            % (-clearance)
        )
    return result


def target_sequence_for_pose(pose: BasePose, high_z: float) -> dict[str, list[float]]:
    tube_bottom_base = world_to_candidate_base(TEST_TUBE_1_WORLD_POSE, pose)
    rack_slot_base = world_to_candidate_base(TEST_TUBE_RACK_2_SLOT_MOUTH_WORLD, pose)
    rack_middle_base = world_to_candidate_base(
        {
            "x": TEST_TUBE_RACK_2_WORLD_POSE["x"],
            "y": TEST_TUBE_RACK_2_WORLD_POSE["y"],
            "z": TEST_TUBE_RACK_2_WORLD_POSE["z"] + 0.012,
        },
        pose,
    )
    grasp = tcp_pose_from_tube_bottom(tube_bottom_base, TEST_TUBE_GRASP_HEIGHT)
    pre_grasp = [
        grasp[0],
        grasp[1],
        grasp[2] + TEST_TUBE_PRE_GRASP_CLEARANCE,
    ]
    lift = [grasp[0], grasp[1], grasp[2] + TEST_TUBE_LIFT_CLEARANCE]

    def insert_target(clearance: float) -> list[float]:
        return tcp_pose_from_tube_bottom(
            {
                "x": rack_slot_base["x"],
                "y": rack_slot_base["y"],
                "z": rack_slot_base["z"] + clearance,
            },
            TEST_TUBE_GRASP_HEIGHT,
        )

    insert_final = tcp_pose_from_tube_bottom(
        rack_middle_base,
        TEST_TUBE_GRASP_HEIGHT,
    )
    return {
        "HIGH_APPROACH": [pre_grasp[0], pre_grasp[1], high_z],
        "PRE_GRASP": pre_grasp,
        "GRASP": grasp,
        "LIFT": lift,
        "TRANSFER": insert_target(RACK_INSERT_PRE_CLEARANCE),
        "PRE_INSERT": insert_target(RACK_INSERT_PRE_CLEARANCE),
        "INSERT_STAGE1": insert_target(RACK_INSERT_STAGE1_CLEARANCE),
        "INSERT_STAGE2": insert_target(RACK_INSERT_STAGE2_CLEARANCE),
        "INSERT": insert_final,
        "RETREAT": insert_target(RACK_INSERT_PRE_CLEARANCE),
    }


def print_offline_summary(results: list[CandidateResult]) -> None:
    spawn_safe = [result for result in results if result.spawn_safe]
    print("TUBE_INSERT_BASE_FEASIBILITY_SUMMARY")
    print("TOTAL_CANDIDATES count=%d" % len(results))
    print("SPAWN_SAFE_CANDIDATES count=%d" % len(spawn_safe))
    print("IK_ALL_PASS_CANDIDATES count=not_run_without_moveit")
    print("TRAJECTORY_SAFE_CANDIDATES count=not_run_without_moveit")
    for result in results[:10]:
        pose = result.pose
        print(
            "candidate base_world=(%.3f,%.3f,%.6f) spawn_safe=%s "
            "spawn_min_clearance=%.4f %s"
            % (
                pose.x,
                pose.y,
                pose.yaw,
                "true" if result.spawn_safe else "false",
                result.spawn_min_clearance,
                result.spawn_reject_reason,
            )
        )
    best_spawn = sorted(spawn_safe, key=lambda item: item.spawn_min_clearance, reverse=True)[:5]
    print("TOP5_SPAWN_GEOMETRY_ONLY")
    for result in best_spawn:
        pose = result.pose
        targets = target_sequence_for_pose(pose, 0.86)
        print(
            "base_world=(%.3f,%.3f,%.6f) spawn_clearance=%.4f "
            "pre_grasp=(%.3f,%.3f,%.3f) pre_insert=(%.3f,%.3f,%.3f)"
            % (
                pose.x,
                pose.y,
                pose.yaw,
                result.spawn_min_clearance,
                targets["PRE_GRASP"][0],
                targets["PRE_GRASP"][1],
                targets["PRE_GRASP"][2],
                targets["PRE_INSERT"][0],
                targets["PRE_INSERT"][1],
                targets["PRE_INSERT"][2],
            )
        )


class TubeInsertBaseFeasibility(Node):
    def __init__(self) -> None:
        super().__init__("tube_insert_base_feasibility")
        self.carried_tube_attached = False
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.validity_client = self.create_client(
            GetStateValidity,
            "/check_state_validity",
        )
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
        )
        self.motion_plan_client = self.create_client(
            GetMotionPlan,
            "/plan_kinematic_path",
        )
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

    def services_available(self, timeout_sec: float) -> bool:
        clients = (
            self.ik_client,
            self.validity_client,
            self.fk_client,
            self.cartesian_client,
            self.motion_plan_client,
            self.scene_client,
        )
        deadline = time.monotonic() + timeout_sec
        for client in clients:
            remaining = max(0.0, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=remaining):
                return False
        return True

    def pose_msg(self, xyz: list[float]) -> Pose:
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.x = TUBE_FIXED_QUAT_XYZW[0]
        pose.orientation.y = TUBE_FIXED_QUAT_XYZW[1]
        pose.orientation.z = TUBE_FIXED_QUAT_XYZW[2]
        pose.orientation.w = TUBE_FIXED_QUAT_XYZW[3]
        return pose

    def home_joint_state(self) -> JointState:
        state = JointState()
        state.name = list(UR_JOINTS)
        state.position = [float(value) for value in HOME_CONFIG]
        return state

    def robot_state(self, joint_state: JointState) -> RobotState:
        state = RobotState()
        state.is_diff = True
        state.joint_state = joint_state
        return state

    def compute_ik(self, label: str, xyz: list[float], seed: JointState) -> JointState | None:
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator"
        request.ik_request.ik_link_name = GRIPPER_TCP_LINK
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.pose = self.pose_msg(xyz)
        request.ik_request.robot_state = self.robot_state(seed)
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 1
        response = self.call(self.ik_client, request, 1.5)
        if response is None:
            print("%s IK_VALID=false error=no_response" % label)
            return None
        valid = int(response.error_code.val) == 1
        print(
            "%s IK_VALID=%s error_code=%d"
            % (label, "true" if valid else "false", int(response.error_code.val))
        )
        return response.solution.joint_state if valid else None

    def state_valid(self, label: str, joint_state: JointState) -> bool:
        request = GetStateValidity.Request()
        request.group_name = "ur_manipulator"
        request.robot_state = self.robot_state(joint_state)
        response = self.call(self.validity_client, request, 1.0)
        if response is None:
            print("%s STATE_VALID=false contacts=unknown" % label)
            return False
        contacts = list(getattr(response, "contacts", []))
        print(
            "%s STATE_VALID=%s contacts=%d"
            % (label, "true" if response.valid else "false", len(contacts))
        )
        if contacts:
            contact = contacts[0]
            print(
                "%s FIRST_CONTACT %s <-> %s depth=%.6f"
                % (
                    label,
                    contact.contact_body_1,
                    contact.contact_body_2,
                    float(contact.depth),
                )
            )
        return bool(response.valid)

    def apply_scene_for_pose(self, pose: BasePose) -> bool:
        if build_tube_insert_planning_scene is not None:
            request = ApplyPlanningScene.Request()
            request.scene = build_tube_insert_planning_scene(
                pose.x,
                pose.y,
                pose.yaw,
            )
            response = self.call(self.scene_client, request, 1.0)
            return bool(response and response.success)
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = [
            self.box_object(
                "station_b_table",
                "base_link",
                self.world_box_center_in_base(STATION_B_TABLE["center"], pose),
                STATION_B_TABLE["size"],
                STATION_B_TABLE["yaw"] - pose.yaw,
            )
        ]
        request.scene.world.collision_objects.extend(self.rack_objects_for_pose(pose))
        response = self.call(self.scene_client, request, 1.0)
        return bool(response and response.success)

    def detach_carried_tube_if_needed(self) -> bool:
        if not self.carried_tube_attached:
            return True
        if make_detach_test_tube_scene is None:
            self.carried_tube_attached = False
            return True
        request = ApplyPlanningScene.Request()
        request.scene = make_detach_test_tube_scene()
        response = self.call(self.scene_client, request, 1.0)
        if response is None or not response.success:
            print("DETACH_CARRIED_TUBE_SCENE_FAILED")
            return False
        self.carried_tube_attached = False
        print("CARRIED_TUBE_DETACHED id=test_tube_1")
        return True

    def reset_scene_for_pose(self, pose: BasePose) -> bool:
        if not self.detach_carried_tube_if_needed():
            return False
        ok = self.apply_scene_for_pose(pose)
        if ok:
            print(
                "CANDIDATE_SCENE_RESET base_world=(%.3f,%.3f,%.6f) "
                "world_tube=test_tube_1 attached_tube=false"
                % (pose.x, pose.y, pose.yaw)
            )
        return ok

    def attach_carried_tube(self) -> bool:
        if self.carried_tube_attached:
            return True
        if make_attach_test_tube_scene is None:
            print("ATTACH_CARRIED_TUBE_SCENE_FAILED reason=helper_unavailable")
            return False
        request = ApplyPlanningScene.Request()
        request.scene = make_attach_test_tube_scene()
        response = self.call(self.scene_client, request, 1.0)
        if response is None or not response.success:
            print("ATTACH_CARRIED_TUBE_SCENE_FAILED")
            return False
        self.carried_tube_attached = True
        print(
            "CARRIED_TUBE_ATTACHED id=test_tube_1 attach_link=%s "
            "world_tube_removed=true"
            % GRIPPER_TCP_LINK
        )
        return True

    def rack_objects_for_pose(self, pose: BasePose):
        objects = []
        rack_specs = (
            ("test_tube_rack_1", RACK1_WORLD_POSE),
            ("test_tube_rack_2", RACK2_WORLD_POSE),
        )
        local_boxes = (
            ("base", (0.0, 0.0, 0.006), (0.31, 0.10, 0.012)),
            ("wall_front", (0.0, 0.036, 0.037), (0.31, 0.028, 0.050)),
            ("wall_back", (0.0, -0.036, 0.037), (0.31, 0.028, 0.050)),
            ("divider_1", (-0.09, 0.0, 0.037), (0.020, 0.044, 0.050)),
            ("divider_2", (-0.03, 0.0, 0.037), (0.020, 0.044, 0.050)),
            ("divider_3", (0.03, 0.0, 0.037), (0.020, 0.044, 0.050)),
            ("divider_4", (0.09, 0.0, 0.037), (0.020, 0.044, 0.050)),
            ("endcap_1", (-0.145, 0.0, 0.037), (0.020, 0.044, 0.050)),
            ("endcap_2", (0.145, 0.0, 0.037), (0.020, 0.044, 0.050)),
        )
        for rack_name, rack_pose in rack_specs:
            rack_yaw = float(rack_pose.get("yaw", 0.0))
            for box_name, local_center, size in local_boxes:
                lx, ly = rotate_point(local_center[0], local_center[1], rack_yaw)
                world_center = {
                    "x": float(rack_pose["x"]) + lx,
                    "y": float(rack_pose["y"]) + ly,
                    "z": float(rack_pose["z"]) + float(local_center[2]),
                }
                center = self.world_box_center_in_base(
                    (world_center["x"], world_center["y"], world_center["z"]),
                    pose,
                )
                objects.append(
                    self.box_object(
                        "%s_%s" % (rack_name, box_name),
                        "base_link",
                        center,
                        size,
                        rack_yaw - pose.yaw,
                    )
                )
        return objects

    def world_box_center_in_base(self, center, pose: BasePose):
        base_xy = world_to_candidate_base(
            {"x": center[0], "y": center[1], "z": center[2]},
            pose,
        )
        return (base_xy["x"], base_xy["y"], base_xy["z"])

    def box_object(self, object_id: str, frame_id: str, center, size, yaw: float):
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in size]
        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        pose.position.z = float(center[2])
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def call(self, client, request, timeout_sec: float):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.monotonic() >= deadline:
                future.cancel()
                return None
        return future.result() if future.done() else None

    def evaluate_candidate_with_services(self, result: CandidateResult) -> CandidateResult:
        if not result.spawn_safe:
            result.failure_reason = "spawn_collision"
            return result
        best_all = False
        try:
            for high_z in HIGH_APPROACH_Z_VALUES:
                if not self.reset_scene_for_pose(result.pose):
                    print("APPLY_SCENE_FAILED base_world=(%.3f,%.3f,%.6f)" % (
                        result.pose.x,
                        result.pose.y,
                        result.pose.yaw,
                    ))
                    result.failure_reason = "planning_scene"
                    return result
                targets = target_sequence_for_pose(result.pose, high_z)
                seed = self.home_joint_state()
                all_ok = True
                carried_for_ik = False
                for stage in (
                    "HIGH_APPROACH",
                    "PRE_GRASP",
                    "GRASP",
                    "LIFT",
                    "PRE_INSERT",
                    "INSERT",
                    "RETREAT",
                ):
                    if stage == "LIFT" and not carried_for_ik:
                        if not self.attach_carried_tube():
                            result.failure_reason = "attached_tube_collision"
                            all_ok = False
                            break
                        carried_for_ik = True
                    solution = self.compute_ik(stage, targets[stage], seed)
                    if solution is None:
                        result.failure_reason = self.ik_failure_reason(stage)
                        all_ok = False
                        break
                    if not self.state_valid(stage, solution):
                        result.failure_reason = "world_collision"
                        all_ok = False
                        break
                    seed = solution
                best_all = best_all or all_ok
                if all_ok:
                    if not self.reset_scene_for_pose(result.pose):
                        result.failure_reason = "planning_scene"
                        return result
                    trajectory_safe, min_clearance, closest_link, reason = (
                        self.plan_and_check_sequence(targets)
                    )
                    result.trajectory_safe = trajectory_safe
                    result.min_arm_table_clearance = min_clearance
                    result.closest_link = closest_link
                    result.failure_reason = "" if trajectory_safe else reason
                    break
        finally:
            self.reset_scene_for_pose(result.pose)
        result.ik_all = best_all
        result.score = self.score(result) if result.trajectory_safe else -1.0
        return result

    def current_base_ik_precheck(self, pose: BasePose) -> bool:
        if not self.reset_scene_for_pose(pose):
            print("CURRENT_BASE_PRECHECK scene=false")
            return False
        for high_z in HIGH_APPROACH_Z_VALUES:
            targets = target_sequence_for_pose(pose, high_z)
            seed = self.home_joint_state()
            high = self.compute_ik(
                "CURRENT_HIGH_APPROACH",
                targets["HIGH_APPROACH"],
                seed,
            )
            high_ok = high is not None and self.state_valid(
                "CURRENT_HIGH_APPROACH",
                high,
            )
            pre_grasp = self.compute_ik(
                "CURRENT_PRE_GRASP",
                targets["PRE_GRASP"],
                high if high is not None else seed,
            )
            pre_grasp_ok = pre_grasp is not None and self.state_valid(
                "CURRENT_PRE_GRASP",
                pre_grasp,
            )
            pre_insert = self.compute_ik(
                "CURRENT_PRE_INSERT",
                targets["PRE_INSERT"],
                pre_grasp if pre_grasp is not None else seed,
            )
            pre_insert_ok = pre_insert is not None and self.state_valid(
                "CURRENT_PRE_INSERT",
                pre_insert,
            )
            print(
                "CURRENT_BASE_IK_PRECHECK high_z=%.3f "
                "HIGH_APPROACH_IK=%s PRE_GRASP_IK=%s PRE_INSERT_IK=%s"
                % (
                    high_z,
                    "true" if high_ok else "false",
                    "true" if pre_grasp_ok else "false",
                    "true" if pre_insert_ok else "false",
                )
            )
            if high_ok and pre_grasp_ok and pre_insert_ok:
                return True
        return False

    def ik_failure_reason(self, stage: str) -> str:
        return {
            "HIGH_APPROACH": "high_approach_ik",
            "PRE_GRASP": "pre_grasp_ik",
            "GRASP": "grasp_ik",
            "LIFT": "lift",
            "PRE_INSERT": "pre_insert_ik",
            "INSERT": "insert_ik",
            "RETREAT": "retreat_ik",
        }.get(stage, "ik")

    def score(self, result: CandidateResult) -> float:
        clearance = (
            result.min_arm_table_clearance
            if result.min_arm_table_clearance is not None
            else 0.0
        )
        yaw_penalty = abs(result.pose.yaw - math.pi / 2.0)
        return 10.0 * clearance + result.spawn_min_clearance - 0.1 * yaw_penalty

    def plan_and_check_sequence(self, targets: dict[str, list[float]]):
        current = self.home_joint_state()
        baseline = self.arm_clearance_baseline(current)
        if not baseline:
            return False, None, "", "arm_clearance"
        segments = (
            ("HOME_TO_HIGH_APPROACH", "ompl", targets["HIGH_APPROACH"]),
            ("HIGH_APPROACH_TO_PRE_GRASP", "ompl", targets["PRE_GRASP"]),
            ("DESCEND_GRASP", "cartesian", targets["GRASP"]),
            ("LIFT", "cartesian", targets["LIFT"]),
            ("TRANSFER_TO_PRE_INSERT", "ompl", targets["PRE_INSERT"]),
            ("INSERT_STAGE1", "cartesian", targets["INSERT_STAGE1"]),
            ("INSERT_STAGE2", "cartesian", targets["INSERT_STAGE2"]),
            ("INSERT", "cartesian", targets["INSERT"]),
            ("RETREAT", "cartesian", targets["RETREAT"]),
        )
        global_min = float("inf")
        closest_link = ""
        for label, mode, target in segments:
            if label == "LIFT":
                if not self.attach_carried_tube():
                    return False, None, "", "attached_tube_collision"
            trajectory = (
                self.plan_cartesian(label, current, target)
                if mode == "cartesian"
                else self.plan_ompl(label, current, target)
            )
            if trajectory is None or not trajectory.points:
                print("%s TRAJECTORY_SAFE=false reason=planning_failed" % label)
                reason = "cartesian_fraction" if mode == "cartesian" else "trajectory_planning"
                return False, None, "", reason
            ok, segment_min, segment_link = self.check_trajectory(
                label,
                trajectory,
                current,
                baseline,
            )
            if not ok:
                reason = "arm_clearance" if segment_link else "world_collision"
                return False, segment_min, segment_link, reason
            if segment_min is not None and segment_min < global_min:
                global_min = segment_min
                closest_link = segment_link
            current = self.joint_state_for_trajectory_point(
                trajectory.joint_names,
                trajectory.points[-1],
                current,
            )
        return True, global_min if global_min != float("inf") else None, closest_link, ""

    def plan_ompl(self, label: str, start: JointState, target: list[float]):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = "ur_manipulator"
        motion.num_planning_attempts = 1
        motion.allowed_planning_time = 3.0
        motion.start_state = self.robot_state(start)
        motion.goal_constraints = [self.pose_constraints(target)]
        response = self.call(self.motion_plan_client, request, 4.0)
        if response is None:
            return None
        code = int(response.motion_plan_response.error_code.val)
        print("%s PLAN_OMPL error_code=%d" % (label, code))
        if code != 1:
            return None
        return response.motion_plan_response.trajectory.joint_trajectory

    def plan_cartesian(self, label: str, start: JointState, target: list[float]):
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.group_name = "ur_manipulator"
        request.link_name = GRIPPER_TCP_LINK
        request.start_state = self.robot_state(start)
        request.waypoints = [self.pose_msg(target)]
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        response = self.call(self.cartesian_client, request, 4.0)
        if response is None:
            return None
        print("%s PLAN_CARTESIAN fraction=%.3f" % (label, float(response.fraction)))
        if float(response.fraction) < 0.95:
            return None
        return response.solution.joint_trajectory

    def pose_constraints(self, target: list[float]):
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = "base_link"
        position.link_name = GRIPPER_TCP_LINK
        position.constraint_region.primitive_poses = [self.pose_msg(target)]
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        position.constraint_region.primitives = [sphere]
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = "base_link"
        orientation.link_name = GRIPPER_TCP_LINK
        pose = self.pose_msg(target)
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = 0.05
        orientation.absolute_y_axis_tolerance = 0.05
        orientation.absolute_z_axis_tolerance = 0.05
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        return constraints

    def arm_clearance_baseline(self, home: JointState):
        heights = self.arm_link_heights(home)
        if heights is None:
            return {}
        limits = {}
        for link, _origin_z, min_z in heights:
            margin = float(ARM_LINK_TABLE_SAFETY_MARGIN[link])
            limits[link] = STATION_B_TABLE["center"][2] + STATION_B_TABLE["size"][2] / 2.0 + margin
            print(
                "ARM_TABLE_CLEARANCE_LIMIT link=%s home_min_z=%.4f "
                "table_surface_z=%.4f home_clearance=%.4f "
                "required_margin=%.4f limit_z=%.4f"
                % (
                    link,
                    min_z,
                    self.table_surface_z(),
                    min_z - self.table_surface_z(),
                    margin,
                    limits[link],
                )
            )
        return limits

    def table_surface_z(self) -> float:
        return float(STATION_B_TABLE["center"][2]) + float(STATION_B_TABLE["size"][2]) / 2.0

    def arm_link_heights(self, joint_state: JointState):
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = list(PROTECTED_ARM_LINK_GEOMETRY_HALF_Z)
        request.robot_state = self.robot_state(joint_state)
        response = self.call(self.fk_client, request, 1.0)
        if response is None:
            return None
        by_link = {
            name: pose
            for name, pose in zip(response.fk_link_names, response.pose_stamped)
        }
        heights = []
        for link, half_z in PROTECTED_ARM_LINK_GEOMETRY_HALF_Z.items():
            pose = by_link.get(link)
            if pose is None:
                return None
            origin_world_z = BASE_LINK_WORLD_Z + float(pose.pose.position.z)
            heights.append((link, origin_world_z, origin_world_z - half_z))
        return heights

    def check_trajectory(
        self,
        label: str,
        trajectory: JointTrajectory,
        start: JointState,
        baseline: dict[str, float],
    ):
        indices = self.sample_indices(len(trajectory.points))
        min_clearance = float("inf")
        closest_link = ""
        for index in indices:
            state = self.joint_state_for_trajectory_point(
                trajectory.joint_names,
                trajectory.points[index],
                start,
            )
            if not self.state_valid("%s_POINT_%d" % (label, index), state):
                print("%s TRAJECTORY_SAFE=false reason=state_collision point=%d" % (label, index))
                return False, None, ""
            heights = self.arm_link_heights(state)
            if heights is None:
                return False, None, ""
            for link, _origin_z, link_min_z in heights:
                limit = baseline[link]
                margin = float(ARM_LINK_TABLE_SAFETY_MARGIN[link])
                clearance = link_min_z - self.table_surface_z()
                valid = clearance >= margin
                if clearance < min_clearance:
                    min_clearance = clearance
                    closest_link = link
                print(
                    "ARM_TABLE_CLEARANCE stage=%s link=%s link_min_z=%.4f "
                    "table_surface_z=%.4f clearance=%.4f required_margin=%.4f "
                    "valid=%s"
                    % (
                        label,
                        link,
                        link_min_z,
                        self.table_surface_z(),
                        clearance,
                        margin,
                        "true" if valid else "false",
                    )
                )
                if link_min_z < limit:
                    print(
                        "ARM_TABLE_CLEARANCE_VIOLATION stage=%s trajectory_point=%d "
                        "link=%s link_min_z=%.4f table_surface_z=%.4f "
                        "clearance=%.4f required_margin=%.4f"
                        % (
                            label,
                            index,
                            link,
                            link_min_z,
                            self.table_surface_z(),
                            clearance,
                            margin,
                        )
                    )
                    return False, link_min_z, link
        print(
            "%s TRAJECTORY_SAFE=true min_arm_table_clearance=%.4f "
            "closest_link=%s"
            % (label, min_clearance, closest_link)
        )
        return True, min_clearance, closest_link

    def sample_indices(self, count: int) -> list[int]:
        if count <= ARM_CLEARANCE_SAMPLE_COUNT:
            return list(range(count))
        return sorted({
            round(i * (count - 1) / (ARM_CLEARANCE_SAMPLE_COUNT - 1))
            for i in range(ARM_CLEARANCE_SAMPLE_COUNT)
        })

    def joint_state_for_trajectory_point(
        self,
        names: list[str],
        point,
        start: JointState,
    ) -> JointState:
        by_name = {
            name: float(position)
            for name, position in zip(start.name, start.position)
        }
        by_name.update({
            name: float(position)
            for name, position in zip(names, point.positions)
        })
        state = JointState()
        state.name = [name for name in UR_JOINTS if name in by_name]
        state.position = [by_name[name] for name in state.name]
        return state


def _non_ros_argv(argv: list[str]) -> list[str]:
    """Return argv with ROS-specific launch/run arguments removed."""
    if remove_ros_args is None:
        return list(argv)
    try:
        return list(remove_ros_args(args=argv))
    except Exception:  # noqa: BLE001
        filtered = []
        skipping_ros_args = False
        for value in argv:
            if value == "--ros-args":
                skipping_ros_args = True
                continue
            if skipping_ros_args:
                if value == "--":
                    skipping_ros_args = False
                continue
            filtered.append(value)
        return filtered


def main(argv: list[str] | None = None) -> None:
    raw_argv = list(sys.argv if argv is None else argv)
    parser = argparse.ArgumentParser()
    parser.add_argument("--offline-only", action="store_true")
    parser.add_argument("--service-timeout", type=float, default=2.0)
    parser.add_argument("--current-base-only", action="store_true")
    args = parser.parse_args(_non_ros_argv(raw_argv)[1:])

    print_world_geometry()
    current = BasePose(
        TUBE_INSERT_VALIDATION_BASE_POSE["x"],
        TUBE_INSERT_VALIDATION_BASE_POSE["y"],
        TUBE_INSERT_VALIDATION_BASE_POSE["yaw"],
    )
    safe_reference = BasePose(0.30, -2.64, math.pi / 2.0)
    unsafe_reference = BasePose(0.30, -2.50, math.pi / 2.0)
    for label, pose in (
        ("CURRENT_CONFIG", current),
        ("REFERENCE_SAFE_Y_NEG_2_640", safe_reference),
        ("REFERENCE_UNSAFE_Y_NEG_2_500", unsafe_reference),
    ):
        result = spawn_geometry_filter(pose)
        print(
            "%s base_world=(%.3f,%.3f,%.6f) spawn_safe=%s clearance=%.4f %s"
            % (
                label,
                pose.x,
                pose.y,
                pose.yaw,
                "true" if result.spawn_safe else "false",
                result.spawn_min_clearance,
                result.spawn_reject_reason,
            )
        )

    results = [spawn_geometry_filter(pose) for pose in candidate_grid()]
    if args.offline_only:
        print_offline_summary(results)
        return

    if rclpy is None:
        print("ROS_PYTHON_UNAVAILABLE; reporting offline geometry only")
        print_offline_summary(results)
        return

    rclpy.init()
    node = TubeInsertBaseFeasibility()
    try:
        if not node.services_available(args.service_timeout):
            print("MOVEIT_SERVICES_UNAVAILABLE; reporting offline geometry only")
            print_offline_summary(results)
            return
        current_precheck_ok = node.current_base_ik_precheck(current)
        print(
            "CURRENT_BASE_PRECHECK_RESULT HIGH_APPROACH_PRE_GRASP_PRE_INSERT=%s"
            % ("true" if current_precheck_ok else "false")
        )
        if args.current_base_only:
            current_result = node.evaluate_candidate_with_services(
                spawn_geometry_filter(current)
            ) if current_precheck_ok else spawn_geometry_filter(current)
            print("TUBE_INSERT_BASE_FEASIBILITY_SUMMARY")
            print("TOTAL_CANDIDATES count=1")
            print(
                "SPAWN_SAFE_CANDIDATES count=%d"
                % (1 if current_result.spawn_safe else 0)
            )
            print(
                "IK_ALL_PASS_CANDIDATES count=%d"
                % (1 if current_result.ik_all else 0)
            )
            print(
                "TRAJECTORY_SAFE_CANDIDATES count=%d"
                % (1 if current_result.trajectory_safe else 0)
            )
            if current_result.trajectory_safe:
                print(
                    "FOUND_FULLY_FEASIBLE_BASE_POSE base_x=%.3f base_y=%.3f "
                    "base_yaw=%.6f"
                    % (current.x, current.y, current.yaw)
                )
            else:
                print("NO_FULLY_FEASIBLE_BASE_POSE")
            return
        evaluated = [
            node.evaluate_candidate_with_services(result)
            for result in results
            if result.spawn_safe
        ]
        full = [result for result in evaluated if result.trajectory_safe]
        ik_all = [result for result in evaluated if result.ik_all]
        failure_counts = Counter(
            result.failure_reason or "unknown"
            for result in evaluated
            if not result.trajectory_safe
        )
        print("TUBE_INSERT_BASE_FEASIBILITY_SUMMARY")
        print("TOTAL_CANDIDATES count=%d" % len(results))
        print("SPAWN_SAFE_CANDIDATES count=%d" % len(evaluated))
        print("IK_ALL_PASS_CANDIDATES count=%d" % len(ik_all))
        print("TRAJECTORY_SAFE_CANDIDATES count=%d" % len(full))
        if full:
            best = sorted(full, key=lambda item: item.score, reverse=True)[0]
            print(
                "FOUND_FULLY_FEASIBLE_BASE_POSE base_x=%.3f base_y=%.3f "
                "base_yaw=%.6f"
                % (best.pose.x, best.pose.y, best.pose.yaw)
            )
        else:
            print("NO_FULLY_FEASIBLE_BASE_POSE")
        for reason in (
            "spawn_collision",
            "pre_grasp_ik",
            "grasp_ik",
            "lift",
            "pre_insert_ik",
            "insert_ik",
            "world_collision",
            "attached_tube_collision",
            "arm_clearance",
            "cartesian_fraction",
            "trajectory_planning",
            "planning_scene",
            "unknown",
        ):
            if failure_counts.get(reason, 0):
                print("FAILURE_REASON reason=%s count=%d" % (reason, failure_counts[reason]))
        ranked = sorted(full, key=lambda item: item.score, reverse=True)[:5]
        for rank, result in enumerate(ranked, start=1):
            pose = result.pose
            print(
                "FULLY_FEASIBLE_CANDIDATE rank=%d base_x=%.3f base_y=%.3f "
                "base_yaw=%.6f spawn_clearance=%.4f "
                "minimum_table_clearance=%.4f nearest_robot_link=%s "
                "PRE_GRASP_IK=true PRE_INSERT_IK=true "
                "trajectory_safe=true arm_clearance=true score=%.4f"
                % (
                    rank,
                    pose.x,
                    pose.y,
                    pose.yaw,
                    result.spawn_min_clearance,
                    result.min_arm_table_clearance
                    if result.min_arm_table_clearance is not None
                    else float("nan"),
                    result.closest_link,
                    result.score,
                )
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
