"""Targeted tests for manipulation repeatability joint-goal tolerance."""
from __future__ import annotations

from pathlib import Path

import pytest


BRINGUP = Path(__file__).resolve().parents[1]


class _FakeMoveIt2:
    def __init__(self):
        self.plan_calls = []
        self.move_to_configuration_calls = []

    def plan(self, **kwargs):
        self.plan_calls.append(kwargs)
        return object()

    def move_to_configuration(self, *args, **kwargs):
        self.move_to_configuration_calls.append((args, kwargs))


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def warn(self, message):
        self.messages.append(("warn", message))

    def error(self, message):
        self.messages.append(("error", message))


class _FakePickPlace:
    def __init__(self, execute_client=True):
        self.moveit2 = _FakeMoveIt2()
        if execute_client:
            self._execute_trajectory_client = object()
        self.normalized = []
        self.executed = []
        self.scaling_calls = []
        self.pose_move_calls = []
        self.logger = _FakeLogger()

    def get_logger(self):
        return self.logger

    def _normalize_wrist_trajectory_to_current(self, trajectory):
        self.normalized.append(trajectory)

    def _execute_trajectory_via_moveit(self, trajectory, timeout_sec):
        self.executed.append((trajectory, timeout_sec))
        return True

    def _with_local_arm_scaling(self, local_speed, callback):
        self.scaling_calls.append(local_speed)
        return callback()

    def _move(self, *args, **kwargs):
        self.pose_move_calls.append((args, kwargs))
        return True


def _load_module():
    import importlib.util

    path = BRINGUP / "lab_cobot_bringup/manipulation_repeatability_node.py"
    spec = importlib.util.spec_from_file_location("repeatability_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _experiment(module, pp, target_mode="joint", tolerance=0.0005):
    exp = object.__new__(module.ManipulationRepeatabilityExperiment)
    exp.pp = pp
    exp.node = pp
    exp.target_mode = target_mode
    exp.target_joints = list(module.DEFAULT_TARGET_JOINTS)
    exp.joint_goal_tolerance = tolerance
    exp.target_position = list(module.DEFAULT_TARGET_POSITION)
    exp.target_quat = list(module.DEFAULT_TARGET_QUAT)
    exp._target_move_capture = {}
    return exp


def test_repeatability_joint_goal_tolerance_default_and_csv_field_are_declared():
    source = (BRINGUP / "lab_cobot_bringup/manipulation_repeatability_node.py").read_text(
        encoding="utf-8"
    )

    assert "DEFAULT_JOINT_GOAL_TOLERANCE = 0.001" in source
    assert 'declare_parameter(\n            "repeatability_joint_goal_tolerance",' in source
    assert '"repeatability_joint_goal_tolerance": self.joint_goal_tolerance' in source


def test_joint_target_plan_receives_repeatability_joint_goal_tolerance(monkeypatch):
    module = _load_module()
    pp = _FakePickPlace(execute_client=True)
    exp = _experiment(module, pp, target_mode="joint", tolerance=0.0005)
    timing_calls = []
    monkeypatch.setattr(
        module.pick_place_module,
        "_ensure_trajectory_timing",
        lambda trajectory: timing_calls.append(trajectory),
    )

    assert exp._move_to_target() is True

    assert pp.moveit2.plan_calls == [
        {
            "joint_positions": module.DEFAULT_TARGET_JOINTS,
            "tolerance_joint_position": 0.0005,
        }
    ]
    assert pp.scaling_calls == [False]
    assert len(pp.normalized) == 1
    assert len(timing_calls) == 1
    assert len(pp.executed) == 1


def test_joint_target_action_fallback_receives_repeatability_joint_goal_tolerance(
    monkeypatch,
):
    module = _load_module()
    pp = _FakePickPlace(execute_client=False)
    exp = _experiment(module, pp, target_mode="joint", tolerance=0.0001)
    monkeypatch.setattr(
        module.pick_place_module,
        "_wait_for_moveit_result",
        lambda *args, **kwargs: True,
    )

    assert exp._move_to_target() is True

    assert pp.moveit2.plan_calls == []
    assert len(pp.moveit2.move_to_configuration_calls) == 1
    args, kwargs = pp.moveit2.move_to_configuration_calls[0]
    assert args[0] == pytest.approx(module.DEFAULT_TARGET_JOINTS)
    assert kwargs == {"tolerance": 0.0001}
    assert pp.scaling_calls == [False]


def test_pose_target_mode_does_not_use_joint_goal_tolerance():
    module = _load_module()
    pp = _FakePickPlace(execute_client=True)
    exp = _experiment(module, pp, target_mode="pose", tolerance=0.0001)

    assert exp._move_to_target() is True

    assert pp.moveit2.plan_calls == []
    assert pp.moveit2.move_to_configuration_calls == []
    assert pp.pose_move_calls
    _args, kwargs = pp.pose_move_calls[0]
    assert "tolerance_joint_position" not in kwargs


def test_moveit_readiness_barrier_requires_existing_planning_and_execution_servers():
    module = _load_module()

    class _Service:
        def wait_for_service(self, timeout_sec):
            return True

    class _Action:
        def wait_for_server(self, timeout_sec):
            return True

    pp = _FakePickPlace(execute_client=True)
    pp.moveit2._plan_kinematic_path_service = _Service()
    pp._execute_trajectory_client = _Action()
    exp = _experiment(module, pp)

    assert exp._wait_for_moveit_ready() is True


def test_readiness_failure_prevents_trial_1_from_starting(tmp_path):
    module = _load_module()
    pp = _FakePickPlace(execute_client=True)
    exp = _experiment(module, pp)
    exp.trials = 1
    exp.initial_config_index = 1
    exp.output_dir = tmp_path / "results"
    exp._wait_for_moveit_ready = lambda: False

    def unexpected_start(*args, **kwargs):
        pytest.fail("trial 1 started before MoveIt readiness")

    pp._move_to_configuration = unexpected_start
    assert exp.run() == exp.output_dir
    assert not exp.output_dir.exists()


def test_start_configuration_failure_skips_target_move_and_writes_failure_record(
    tmp_path,
):
    module = _load_module()
    pp = _FakePickPlace(execute_client=True)
    exp = _experiment(module, pp)
    exp.trials = 1
    exp.initial_config_index = 1
    exp.output_dir = tmp_path / "results"
    exp._wait_for_moveit_ready = lambda: True
    pp._move_to_configuration = lambda *args, **kwargs: False
    exp._move_to_target = lambda: pytest.fail("target move called after start failure")

    csv_path = exp.run()
    import csv

    with csv_path.open(newline="") as handle:
        row = next(csv.DictReader(handle))
    assert row["initial_config_success"] == "0"
    assert row["target_move_success"] == "0"
    assert row["move_success"] == "0"
    assert row["failure_stage"] == "START_CONFIGURATION"
