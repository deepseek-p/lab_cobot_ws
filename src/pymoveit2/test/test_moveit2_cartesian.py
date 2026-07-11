"""Regression tests for MoveIt2 Cartesian path request construction."""
from pathlib import Path
from types import SimpleNamespace
import sys

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
