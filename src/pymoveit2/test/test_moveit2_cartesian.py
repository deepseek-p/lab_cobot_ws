"""Regression tests for MoveIt2 Cartesian path request construction."""
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetCartesianPath
from pymoveit2.moveit2 import MoveIt2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class _Stamp:
    def to_msg(self):
        return Time()


class _Clock:
    def now(self):
        return _Stamp()


class _Node:
    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return self

    def warn(self, _message):
        return None


class _FakeCartesianService:
    """Capture the request and return a canned Cartesian path response."""

    srv_name = "compute_cartesian_path"

    def __init__(self, fraction=1.0):
        self.fraction = fraction
        self.captured_request = None

    def wait_for_service(self, timeout_sec=None):
        return True

    def call(self, request):
        self.captured_request = request
        return SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            fraction=self.fraction,
            solution=SimpleNamespace(joint_trajectory="fake-trajectory"),
        )


def _make_moveit_with_tcp_goal(fraction=1.0):
    """Assemble a bare MoveIt2 with a gripper_tcp goal in base_link frame."""
    moveit2 = object.__new__(MoveIt2)
    moveit2._node = _Node()
    moveit2._MoveIt2__end_effector_name = "ur_tool0"
    moveit2._MoveIt2__base_link_name = "ur_base_link"
    moveit2._MoveIt2__cartesian_path_request = GetCartesianPath.Request()

    goal = MoveGroup.Goal()
    constraints = Constraints()
    position = PositionConstraint()
    position.header.frame_id = "base_link"
    position.link_name = "gripper_tcp"
    position.constraint_region.primitive_poses.append(Pose())
    position.constraint_region.primitive_poses[0].position.x = 0.82
    position.constraint_region.primitive_poses[0].position.z = 0.745
    constraints.position_constraints.append(position)
    orientation = OrientationConstraint()
    orientation.header.frame_id = "base_link"
    orientation.link_name = "gripper_tcp"
    orientation.orientation.w = 1.0
    constraints.orientation_constraints.append(orientation)
    goal.request.goal_constraints.append(constraints)
    moveit2._MoveIt2__move_action_goal = goal

    service = _FakeCartesianService(fraction=fraction)
    moveit2._plan_cartesian_path_service = service
    return moveit2, service


def test_cartesian_request_uses_goal_constraint_link_and_frame():
    # 根因回归(E2E 实测):cartesian 分支曾把 link_name 写死为
    # end_effector(ur_tool0)、frame 回落 ur_base_link,而约束里的目标
    # 是 gripper_tcp@base_link——双重错位导致直线终点完全错误,
    # 多数轮靠 fraction 截断侥幸,规划报错轮直接 FAILED。
    moveit2, service = _make_moveit_with_tcp_goal()

    result = moveit2._plan_cartesian_path()

    assert result == "fake-trajectory"
    assert service.captured_request.link_name == "gripper_tcp"
    assert service.captured_request.header.frame_id == "base_link"


def test_cartesian_request_falls_back_to_defaults_without_link_info():
    # 约束未填 link/frame 时保持旧默认(end_effector@base_link_name)
    moveit2, service = _make_moveit_with_tcp_goal()
    goal = moveit2._MoveIt2__move_action_goal
    goal.request.goal_constraints[-1].position_constraints[-1].link_name = ""
    goal.request.goal_constraints[-1].position_constraints[
        -1
    ].header.frame_id = ""

    moveit2._plan_cartesian_path()

    assert service.captured_request.link_name == "ur_tool0"
    assert service.captured_request.header.frame_id == "ur_base_link"


def test_cartesian_partial_fraction_is_rejected():
    # fraction<阈值 = 直线被障碍/限位截断,执行部分轨迹会停在中途
    # 假装成功(descend 语义被破坏)——必须判失败走上层重试。
    moveit2, _service = _make_moveit_with_tcp_goal(fraction=0.4)

    assert moveit2._plan_cartesian_path() is None


def test_move_to_pose_cartesian_uses_cartesian_plan_before_execute():
    moveit2 = object.__new__(MoveIt2)
    moveit2._MoveIt2__execute_via_moveit = True
    calls = {}

    def fake_plan(**kwargs):
        calls["plan"] = kwargs
        return "cartesian-trajectory"

    def fake_execute(trajectory, **kwargs):
        calls["execute"] = trajectory
        calls["execute_kwargs"] = kwargs

    moveit2.plan = fake_plan
    moveit2.execute = fake_execute

    moveit2.move_to_pose(
        position=(0.8, 0.0, 0.75),
        quat_xyzw=(1.0, 0.0, 0.0, 0.0),
        frame_id="base_link",
        target_link="gripper_tcp",
        cartesian=True,
    )

    assert calls["plan"]["cartesian"] is True
    assert calls["plan"]["target_link"] == "gripper_tcp"
    assert calls["plan"]["frame_id"] == "base_link"
    assert calls["execute"] == "cartesian-trajectory"
    assert calls["execute_kwargs"]["via_moveit"] is True


def test_execute_adds_monotonic_timing_for_cartesian_trajectory_points():
    moveit2 = object.__new__(MoveIt2)
    moveit2._MoveIt2__ignore_new_calls_while_executing = False
    moveit2._MoveIt2__is_executing = False
    moveit2._MoveIt2__is_motion_requested = False
    moveit2._MoveIt2__last_execution_succeeded = True
    captured = {}

    def fake_send(goal):
        captured["goal"] = goal

    moveit2._send_goal_async_follow_joint_trajectory = fake_send

    trajectory = JointTrajectory()
    trajectory.joint_names = ["ur_wrist_3_joint"]
    trajectory.points = [JointTrajectoryPoint(), JointTrajectoryPoint()]
    trajectory.points[0].positions = [0.0]
    trajectory.points[1].positions = [0.1]

    moveit2.execute(trajectory)

    assert captured["goal"].trajectory.points[0].time_from_start.nanosec > 0
    assert (
        captured["goal"].trajectory.points[1].time_from_start.nanosec
        > captured["goal"].trajectory.points[0].time_from_start.nanosec
    )
    assert moveit2._MoveIt2__last_execution_succeeded is False


def test_execute_via_moveit_wraps_cartesian_trajectory_for_moveit_execution():
    moveit2 = object.__new__(MoveIt2)
    moveit2._MoveIt2__ignore_new_calls_while_executing = False
    moveit2._MoveIt2__is_executing = False
    moveit2._MoveIt2__is_motion_requested = False
    moveit2._MoveIt2__last_execution_succeeded = True
    captured = {}

    def fake_send(goal):
        captured["goal"] = goal

    moveit2._send_goal_async_execute_trajectory = fake_send

    trajectory = JointTrajectory()
    trajectory.joint_names = ["ur_wrist_3_joint"]
    point = JointTrajectoryPoint()
    point.positions = [0.1]
    point.time_from_start.nanosec = 100000000
    trajectory.points = [point]

    moveit2.execute(trajectory, via_moveit=True)

    assert captured["goal"].trajectory.joint_trajectory is trajectory
    assert moveit2._MoveIt2__last_execution_succeeded is False


def test_joint_path_constraints_are_written_to_move_group_request():
    moveit2 = object.__new__(MoveIt2)
    moveit2._MoveIt2__joint_names = [
        "ur_wrist_1_joint",
        "ur_wrist_2_joint",
        "ur_wrist_3_joint",
    ]
    moveit2._MoveIt2__move_action_goal = MoveGroup.Goal()

    moveit2.set_joint_path_constraints(
        [-1.0, 4.0, -0.2],
        tolerance=[0.7, 0.8, 0.4],
    )

    constraints = (
        moveit2._MoveIt2__move_action_goal.request.path_constraints
        .joint_constraints
    )
    assert [constraint.joint_name for constraint in constraints] == [
        "ur_wrist_1_joint",
        "ur_wrist_2_joint",
        "ur_wrist_3_joint",
    ]
    assert [constraint.position for constraint in constraints] == pytest.approx(
        [-1.0, 4.0, -0.2]
    )
    assert [constraint.tolerance_above for constraint in constraints] == pytest.approx(
        [0.7, 0.8, 0.4]
    )

    moveit2.clear_path_constraints()

    assert (
        moveit2._MoveIt2__move_action_goal.request.path_constraints
        .joint_constraints
        == []
    )
