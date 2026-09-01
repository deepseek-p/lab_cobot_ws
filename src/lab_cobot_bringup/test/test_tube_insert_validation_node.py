"""Contracts for the fixed-coordinate test-tube insertion validation node."""

import inspect
from types import SimpleNamespace

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
import pytest
from sensor_msgs.msg import JointState

from lab_cobot_bringup import tube_insert_validation_node
from lab_cobot_bringup.tube_insert_config import TUBE_INSERT_CONFIG
from lab_cobot_bringup.tube_insert_validation_node import (
    DEFAULT_CARTESIAN_MAX_STEP,
    INSERT_FINAL_CARTESIAN_MAX_STEP,
    OMPL_JOINT_COST_WEIGHTS,
    SHOULDER_PAN_JOINT,
    TUBE_PRE_CLOSE_POSITION_TOLERANCE,
    TubeInsertValidation,
)


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", str(message)))

    def warn(self, message):
        self.messages.append(("warn", str(message)))

    def error(self, message):
        self.messages.append(("error", str(message)))


def _joint_state():
    joint_state = JointState()
    joint_state.name = [
        "ur_shoulder_pan_joint",
        "ur_shoulder_lift_joint",
        "ur_elbow_joint",
        "ur_wrist_1_joint",
        "ur_wrist_2_joint",
        "ur_wrist_3_joint",
    ]
    joint_state.position = [0.12, -1.33, 1.0, -1.2, -1.5, 0.0]
    return joint_state


def _finger_joint_state(left, right, stamp_sec=0.0):
    joint_state = JointState()
    joint_state.header.stamp.sec = int(stamp_sec)
    joint_state.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1.0e9)
    joint_state.name = [
        "gripper_left_finger_joint",
        "gripper_right_finger_joint",
    ]
    joint_state.position = [left, right]
    return joint_state


def _node_without_ros():
    node = TubeInsertValidation.__new__(TubeInsertValidation)
    node.logger = FakeLogger()
    node.get_logger = lambda: node.logger
    node._publish_status = lambda text: setattr(node, "last_status", text)
    node._active_stage_label = "TEST"
    return node


class FakeGripper:
    def __init__(self):
        self.commands = []
        self.open_calls = 0

    def open(self):
        self.open_calls += 1
        return True

    def command_positions(self, positions):
        self.commands.append(list(positions))
        return True

    def actual_finger_positions(self):
        return (None, None)

    def estimated_gap_mm(self, command):
        return (0.092 - float(command[0]) - float(command[1])) * 1000.0


def _install_fake_time(monkeypatch, *, wall_start=0.0, wall_step=0.1):
    wall = {"value": float(wall_start)}

    def fake_monotonic():
        wall["value"] += float(wall_step)
        return wall["value"]

    monkeypatch.setattr(tube_insert_validation_node.time, "monotonic", fake_monotonic)
    monkeypatch.setattr(tube_insert_validation_node.time, "sleep", lambda _sec: None)
    return wall


def _install_ros_time_sequence(monkeypatch, node, values):
    iterator = iter(values)
    last = {"value": float(values[-1])}

    def fake_ros_now():
        try:
            last["value"] = float(next(iterator))
        except StopIteration:
            pass
        return last["value"]

    monkeypatch.setattr(node, "_ros_now_sec", fake_ros_now)


def _install_feedback_sequence(monkeypatch, node, values):
    iterator = iter(values)
    last = {"value": values[-1]}

    def fake_feedback(**_kwargs):
        try:
            last["value"] = next(iterator)
        except StopIteration:
            pass
        return last["value"]

    monkeypatch.setattr(node, "_current_tube_finger_feedback", fake_feedback)


def _scene_with_ids(ids):
    scene = PlanningScene()
    scene.world.collision_objects = []
    for object_id in ids:
        obj = CollisionObject()
        obj.id = str(object_id)
        scene.world.collision_objects.append(obj)
    return scene


def _expected_tube_ids():
    pose = tube_insert_validation_node.TUBE_INSERT_VALIDATION_BASE_POSE
    scene = tube_insert_validation_node.build_tube_insert_planning_scene(
        pose["x"],
        pose["y"],
        pose["yaw"],
    )
    return {obj.id for obj in scene.world.collision_objects}


def _log_contains(node, text):
    return any(text in message for _level, message in node.logger.messages)


def test_tube_insert_scores_all_ur_joints_equally():
    assert SHOULDER_PAN_JOINT == "ur_shoulder_pan_joint"
    assert OMPL_JOINT_COST_WEIGHTS == {
        "ur_shoulder_pan_joint": 1.0,
        "ur_shoulder_lift_joint": 1.0,
        "ur_elbow_joint": 1.0,
        "ur_wrist_1_joint": 1.0,
        "ur_wrist_2_joint": 1.0,
        "ur_wrist_3_joint": 1.0,
    }


def test_joint_state_distance_uses_shortest_angle_delta_for_all_joints():
    node = _node_without_ros()
    current = _joint_state()
    goal = [
        0.12 + 6.20,
        -1.33,
        1.25,
        -1.30,
        -1.5,
        0.20,
    ]

    distance = node._joint_state_distance(current, goal)  # noqa: SLF001

    assert round(distance, 5) == round(abs(6.20 - 6.283185307179586) + 0.25 + 0.10 + 0.20, 5)


def test_pose_optimization_executes_direct_joint_path_when_safe(monkeypatch):
    node = _node_without_ros()
    node.moveit2 = SimpleNamespace(joint_state=_joint_state())
    goal = [0.13, -1.34, 1.02, -1.18, -1.49, 0.03]
    monkeypatch.setattr(
        node,
        "_goal_joint_candidates_for_pose",
        lambda *_args: [{"index": 0, "positions": goal, "distance": 0.08}],
    )
    monkeypatch.setattr(node, "_trajectory_arm_clearance_ok", lambda *_args: True)
    executed = []
    monkeypatch.setattr(
        tube_insert_validation_node.PickPlace,
        "_execute_trajectory_via_moveit",
        lambda _self, trajectory, _timeout: executed.append(trajectory) or True,
    )

    assert node._move_pose_with_joint_optimization(  # noqa: SLF001
        "PRE_INSERT",
        [0.69, 0.18, 0.862],
        timeout_sec=0.01,
    )

    assert executed
    assert executed[0].joint_names == list(OMPL_JOINT_COST_WEIGHTS)
    assert any(
        "SELECTED_NEAREST_IK stage=PRE_INSERT" in message
        for _level, message in node.logger.messages
    )


def test_goal_positions_are_normalized_to_nearest_equivalent_branch():
    node = _node_without_ros()
    current = _joint_state()
    goal = _joint_state()
    goal.position = [
        current.position[0] + 2 * 3.141592653589793,
        current.position[1],
        current.position[2],
        current.position[3],
        current.position[4],
        current.position[5],
    ]

    normalized = node._normalize_goal_positions_to_start(current, goal)  # noqa: SLF001

    assert normalized[0] == pytest.approx(current.position[0])


def test_cartesian_request_does_not_carry_shoulder_path_constraints(monkeypatch):
    node = _node_without_ros()
    node.moveit2 = SimpleNamespace(joint_state=_joint_state())
    requests = []

    class FakeClient:
        def wait_for_service(self, timeout_sec=None):
            return True

        def call_async(self, request):
            requests.append(request)
            return SimpleNamespace(
                done=lambda: True,
                result=lambda: None,
                cancel=lambda: None,
            )

    node._cartesian_path_client = FakeClient()
    monkeypatch.setattr(
        node,
        "_current_tcp_pose_for_joint_state",
        lambda _state: None,
    )

    assert node._move_cartesian_fraction_checked(  # noqa: SLF001
        "INSERT_STAGE1",
        [0.69, 0.18, 0.79],
    ) == 0.0

    assert requests
    assert requests[0].path_constraints.joint_constraints == []
    assert requests[0].max_step == DEFAULT_CARTESIAN_MAX_STEP


def test_insert_final_uses_local_smaller_cartesian_step(monkeypatch):
    node = _node_without_ros()
    node.moveit2 = SimpleNamespace(joint_state=_joint_state())
    requests = []

    class FakeClient:
        def wait_for_service(self, timeout_sec=None):
            return True

        def call_async(self, request):
            requests.append(request)
            return SimpleNamespace(
                done=lambda: True,
                result=lambda: None,
                cancel=lambda: None,
            )

    node._cartesian_path_client = FakeClient()
    monkeypatch.setattr(
        node,
        "_current_tcp_pose_for_joint_state",
        lambda _state: None,
    )
    monkeypatch.setattr(
        node,
        "_diagnose_insert_final_continuity_scan",
        lambda *_args: None,
    )

    assert node._move_cartesian_fraction_checked(  # noqa: SLF001
        "INSERT_FINAL",
        [0.69, 0.18, 0.69],
    ) == 0.0

    assert requests
    assert requests[0].max_step == INSERT_FINAL_CARTESIAN_MAX_STEP


def test_post_insert_retreat_target_lifts_to_pre_insert_z(monkeypatch):
    node = _node_without_ros()
    rack = TUBE_INSERT_CONFIG["test_tube_rack_2"]
    pose = Pose()
    pose.position.x = 0.691
    pose.position.y = 0.179
    pose.position.z = rack["insert_final_tcp"][2]
    monkeypatch.setattr(
        node,
        "_current_tcp_pose_for_joint_state",
        lambda _state: pose,
    )
    node.moveit2 = SimpleNamespace(joint_state=_joint_state())

    target = node._post_insert_vertical_retreat_target(rack)  # noqa: SLF001

    assert target[:2] == [0.691, 0.179]
    assert target[2] == rack["pre_insert_tcp"][2]


def test_sequence_retracts_after_release_opening_before_return_home():
    source = inspect.getsource(TubeInsertValidation._run_sequence)

    release_opening_index = source.index("self._tube_post_insert_release_opening()")
    retract_index = source.index('"POST_INSERT_VERTICAL_RETREAT"')
    home_index = source.index('self.go_home(stage_label="RETURN_HOME")')

    assert release_opening_index < retract_index < home_index


def test_tube_pre_close_waits_until_actual_reaches_target(monkeypatch):
    node = _node_without_ros()
    node.gripper = FakeGripper()
    _install_fake_time(monkeypatch)
    _install_ros_time_sequence(monkeypatch, node, [1.0, 1.0, 1.2, 1.4, 1.6])
    _install_feedback_sequence(monkeypatch, node, [
        (1.1, 0.0, 0.0),
        (1.2, 0.015, 0.014),
        (1.3, 0.029, 0.029),
    ])

    assert node._tube_pre_close()  # noqa: SLF001

    assert node.gripper.commands[0] == pytest.approx([0.029, 0.029])
    assert _log_contains(node, "TUBE_PRE_CLOSE_DONE")
    assert _log_contains(node, "left_actual=0.029000")


def test_tube_pre_close_fails_when_actual_stays_open(monkeypatch):
    node = _node_without_ros()
    node.gripper = FakeGripper()
    _install_fake_time(monkeypatch)
    _install_ros_time_sequence(
        monkeypatch,
        node,
        [1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.1],
    )
    monkeypatch.setattr(
        node,
        "_current_tube_finger_feedback",
        lambda **_kwargs: (2.0, 0.0, 0.0),
    )

    assert not node._tube_pre_close()  # noqa: SLF001

    assert _log_contains(node, "TUBE_PRE_CLOSE_FAILED reason=POSITION_NOT_REACHED")
    assert not _log_contains(node, "TUBE_PRE_CLOSE_DONE")


def test_tube_pre_close_fails_when_right_finger_not_reached(monkeypatch):
    node = _node_without_ros()
    node.gripper = FakeGripper()
    _install_fake_time(monkeypatch)
    _install_ros_time_sequence(
        monkeypatch,
        node,
        [1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.1],
    )
    monkeypatch.setattr(
        node,
        "_current_tube_finger_feedback",
        lambda **_kwargs: (2.0, 0.029, 0.0),
    )

    assert not node._tube_pre_close()  # noqa: SLF001

    assert _log_contains(node, "TUBE_PRE_CLOSE_FAILED")


def test_tube_pre_close_fails_when_left_finger_not_reached(monkeypatch):
    node = _node_without_ros()
    node.gripper = FakeGripper()
    _install_fake_time(monkeypatch)
    _install_ros_time_sequence(
        monkeypatch,
        node,
        [1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.1],
    )
    monkeypatch.setattr(
        node,
        "_current_tube_finger_feedback",
        lambda **_kwargs: (2.0, 0.0, 0.029),
    )

    assert not node._tube_pre_close()  # noqa: SLF001

    assert _log_contains(node, "TUBE_PRE_CLOSE_FAILED")


def test_old_cached_joint_state_cannot_satisfy_tube_pre_close():
    node = _node_without_ros()
    node.moveit2 = SimpleNamespace(
        joint_state=_finger_joint_state(0.029, 0.029, stamp_sec=9.0)
    )

    reached, left, right, left_error, right_error = node._tube_pre_close_reached(  # noqa: SLF001
        0.029,
        0.029,
        min_feedback_ros_time=10.0,
        tolerance=TUBE_PRE_CLOSE_POSITION_TOLERANCE,
    )

    assert not reached
    assert left is None
    assert right is None
    assert left_error is None
    assert right_error is None


def test_low_rtf_wall_elapsed_does_not_fail_before_sim_timeout(monkeypatch):
    node = _node_without_ros()
    node.gripper = FakeGripper()
    _install_fake_time(monkeypatch, wall_step=8.0)
    _install_ros_time_sequence(monkeypatch, node, [1.0, 1.0, 1.2, 1.4, 1.6])
    _install_feedback_sequence(monkeypatch, node, [
        (1.1, 0.0, 0.0),
        (1.2, 0.010, 0.010),
        (1.3, 0.029, 0.029),
    ])

    assert node._tube_pre_close()  # noqa: SLF001

    assert not _log_contains(node, "TUBE_PRE_CLOSE_CLOCK_STALLED")
    assert not _log_contains(node, "TUBE_PRE_CLOSE_FAILED")


def test_tube_pre_close_reports_clock_stalled(monkeypatch):
    node = _node_without_ros()
    node.gripper = FakeGripper()
    _install_fake_time(monkeypatch, wall_step=4.0)
    _install_ros_time_sequence(monkeypatch, node, [1.0])
    monkeypatch.setattr(
        node,
        "_current_tube_finger_feedback",
        lambda **_kwargs: (None, None, None),
    )

    assert not node._tube_pre_close()  # noqa: SLF001

    assert _log_contains(node, "TUBE_PRE_CLOSE_CLOCK_STALLED")


def test_run_sequence_blocks_descend_when_pre_close_fails(monkeypatch):
    node = _node_without_ros()
    node.gripper = FakeGripper()
    statuses = []
    node._publish_status = statuses.append
    monkeypatch.setattr(node, "_move_ompl_if_needed", lambda *_args: True)
    monkeypatch.setattr(node, "_tube_pre_close", lambda: False)
    cartesian_calls = []
    monkeypatch.setattr(
        node,
        "_require_cartesian",
        lambda *_args: cartesian_calls.append(True) or True,
    )
    monkeypatch.setattr(node, "_release_busy", lambda: None)

    node._run_sequence()  # noqa: SLF001

    assert "TUBE_PRE_CLOSE" in statuses
    assert "FAILED_TUBE_PRE_CLOSE" in statuses
    assert cartesian_calls == []


def test_tube_scene_already_ready_skips_apply(monkeypatch):
    node = _node_without_ros()
    expected_ids = _expected_tube_ids()
    apply_calls = []
    monkeypatch.setattr(
        node,
        "_get_current_planning_scene",
        lambda *_args, **_kwargs: _scene_with_ids(expected_ids),
    )
    monkeypatch.setattr(
        node,
        "_apply_scene_diff",
        lambda *_args, **_kwargs: apply_calls.append(True) or True,
    )

    assert node._apply_tube_insert_scene()  # noqa: SLF001
    assert apply_calls == []
    assert _log_contains(node, "TUBE_SCENE_ALREADY_READY")


def test_tube_scene_missing_then_apply_success_verifies_complete(monkeypatch):
    node = _node_without_ros()
    expected_ids = _expected_tube_ids()
    queries = iter([_scene_with_ids(set()), _scene_with_ids(expected_ids)])
    monkeypatch.setattr(
        node,
        "_get_current_planning_scene",
        lambda *_args, **_kwargs: next(queries),
    )
    monkeypatch.setattr(node, "_apply_scene_diff", lambda *_args, **_kwargs: True)

    assert node._apply_tube_insert_scene()  # noqa: SLF001
    assert _log_contains(node, "TUBE_SCENE_APPLY_START")
    assert _log_contains(node, "TUBE_SCENE_APPLY_SUCCESS")
    assert _log_contains(node, "TUBE_INSERT_SCENE_APPLIED")


def test_tube_scene_apply_timeout_but_ready_after_recheck(monkeypatch):
    node = _node_without_ros()
    expected_ids = _expected_tube_ids()
    queries = iter([_scene_with_ids(set()), _scene_with_ids(expected_ids)])
    monkeypatch.setattr(
        node,
        "_get_current_planning_scene",
        lambda *_args, **_kwargs: next(queries),
    )
    monkeypatch.setattr(node, "_apply_scene_diff", lambda *_args, **_kwargs: False)

    assert node._apply_tube_insert_scene()  # noqa: SLF001
    assert _log_contains(node, "TUBE_SCENE_APPLY_TIMEOUT_RECHECK")
    assert _log_contains(node, "TUBE_SCENE_APPLY_RESPONSE_TIMEOUT_BUT_READY")
    assert _log_contains(node, "TUBE_SCENE_READY_AFTER_RECHECK")


def test_tube_scene_apply_false_and_missing_fails(monkeypatch):
    node = _node_without_ros()
    expected_ids = _expected_tube_ids()
    partial_ids = set(expected_ids)
    partial_ids.remove("test_tube_9")
    queries = iter([_scene_with_ids(set()), _scene_with_ids(partial_ids)])
    monkeypatch.setattr(
        node,
        "_get_current_planning_scene",
        lambda *_args, **_kwargs: next(queries),
    )
    monkeypatch.setattr(node, "_apply_scene_diff", lambda *_args, **_kwargs: False)

    assert not node._apply_tube_insert_scene()  # noqa: SLF001
    assert _log_contains(node, "TUBE_SCENE_MISSING_OBJECTS ids=test_tube_9")


def test_tube_scene_partial_existing_is_not_ready(monkeypatch):
    node = _node_without_ros()
    expected_ids = _expected_tube_ids()
    partial_ids = set(expected_ids)
    partial_ids.remove("station_b_table")
    monkeypatch.setattr(
        node,
        "_get_current_planning_scene",
        lambda *_args, **_kwargs: _scene_with_ids(partial_ids),
    )
    monkeypatch.setattr(node, "_apply_scene_diff", lambda *_args, **_kwargs: False)

    assert not node._apply_tube_insert_scene()  # noqa: SLF001
    assert not _log_contains(node, "TUBE_SCENE_ALREADY_READY")


def test_tube_scene_empty_after_apply_failure_fails(monkeypatch):
    node = _node_without_ros()
    monkeypatch.setattr(
        node,
        "_get_current_planning_scene",
        lambda *_args, **_kwargs: _scene_with_ids(set()),
    )
    monkeypatch.setattr(node, "_apply_scene_diff", lambda *_args, **_kwargs: False)

    assert not node._apply_tube_insert_scene()  # noqa: SLF001
    assert _log_contains(node, "TUBE_SCENE_MISSING_OBJECTS")


def test_unrelated_scene_objects_do_not_satisfy_tube_scene(monkeypatch):
    node = _node_without_ros()
    monkeypatch.setattr(
        node,
        "_get_current_planning_scene",
        lambda *_args, **_kwargs: _scene_with_ids({"unrelated_box"}),
    )
    monkeypatch.setattr(node, "_apply_scene_diff", lambda *_args, **_kwargs: False)

    assert not node._apply_tube_insert_scene()  # noqa: SLF001
    assert not _log_contains(node, "TUBE_SCENE_ALREADY_READY")


def test_tube_scene_readiness_does_not_reference_navigation_or_map():
    source = inspect.getsource(TubeInsertValidation._apply_tube_insert_scene)
    source += inspect.getsource(TubeInsertValidation._tube_scene_missing_object_ids)

    assert "map" not in source
    assert "navigation" not in source
