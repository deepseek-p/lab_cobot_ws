"""Regression tests for MoveIt2 execution status reporting."""
from pathlib import Path
import sys
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from action_msgs.msg import GoalStatus
from pymoveit2.moveit2 import MoveIt2


class _Rate:
    def __init__(self, moveit2):
        self._moveit2 = moveit2

    def sleep(self):
        self._moveit2._MoveIt2__is_motion_requested = False
        self._moveit2._MoveIt2__is_executing = False
        return None


class _NoProgressRate:
    def sleep(self):
        return None


class _Node:
    def get_logger(self):
        return self

    def warn(self, _message):
        return None

    def error(self, _message):
        return None


class _MoveActionClient:
    _action_name = "move_action"


class _Future:
    def __init__(self, result=None):
        self._result = result
        self.callbacks = []

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


class _GoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_calls = 0
        self.result_requested = False
        self.result_future = _Future()

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return _Future()

    def get_result_async(self):
        self.result_requested = True
        return self.result_future


class _MoveResult:
    def __init__(self, status):
        self.status = status


def _moveit2_with_status(status):
    moveit2 = object.__new__(MoveIt2)
    moveit2._MoveIt2__is_motion_requested = True
    moveit2._MoveIt2__is_executing = False
    moveit2._MoveIt2__wait_until_executed_rate = _Rate(moveit2)
    moveit2._MoveIt2__last_execution_succeeded = status
    return moveit2


def _install_move_action_guards(moveit2, generation=4):
    moveit2._MoveIt2__move_goal_handle = None
    moveit2._MoveIt2__move_generation = generation
    moveit2._MoveIt2__move_lock = threading.Lock()


def _moveit2_with_move_action_state():
    moveit2 = object.__new__(MoveIt2)
    moveit2._node = _Node()
    moveit2._MoveIt2__move_action_client = _MoveActionClient()
    moveit2._MoveIt2__is_motion_requested = True
    moveit2._MoveIt2__is_executing = False
    moveit2._MoveIt2__wait_until_executed_rate = _NoProgressRate()
    moveit2._MoveIt2__last_execution_succeeded = True
    _install_move_action_guards(moveit2)
    return moveit2


def _call_response_callback(moveit2, future, gen):
    callback = moveit2._MoveIt2__response_callback_move_action
    try:
        callback(future, gen=gen)
    except TypeError:
        callback(future)


def _call_result_callback(moveit2, future, gen):
    callback = moveit2._MoveIt2__result_callback_move_action
    try:
        callback(future, gen=gen)
    except TypeError:
        callback(future)


def test_wait_until_executed_returns_true_after_success():
    moveit2 = _moveit2_with_status(True)

    assert moveit2.wait_until_executed() is True


def test_wait_until_executed_returns_false_after_failure():
    moveit2 = _moveit2_with_status(False)

    assert moveit2.wait_until_executed() is False


def test_wait_until_executed_times_out_and_clears_motion_state():
    moveit2 = object.__new__(MoveIt2)
    moveit2._MoveIt2__is_motion_requested = True
    moveit2._MoveIt2__is_executing = False
    moveit2._MoveIt2__wait_until_executed_rate = _NoProgressRate()
    moveit2._MoveIt2__last_execution_succeeded = True
    moveit2._node = _Node()
    _install_move_action_guards(moveit2, generation=0)

    assert moveit2.wait_until_executed(timeout_sec=0.0) is False
    assert moveit2._MoveIt2__is_motion_requested is False
    assert moveit2._MoveIt2__is_executing is False


def test_wait_timeout_cancels_goal_and_ignores_stale_success_result():
    moveit2 = _moveit2_with_move_action_state()
    goal_handle = _GoalHandle()
    moveit2._MoveIt2__move_goal_handle = goal_handle
    old_generation = moveit2._MoveIt2__move_generation

    assert moveit2.wait_until_executed(timeout_sec=0.0) is False

    assert goal_handle.cancel_calls == 1
    assert moveit2._MoveIt2__move_generation == old_generation + 1

    moveit2._MoveIt2__is_executing = True
    moveit2._MoveIt2__last_execution_succeeded = False
    _call_result_callback(
        moveit2,
        _Future(_MoveResult(GoalStatus.STATUS_SUCCEEDED)),
        gen=old_generation,
    )

    assert moveit2._MoveIt2__is_executing is True
    assert moveit2._MoveIt2__last_execution_succeeded is False


def test_stale_move_action_response_is_ignored():
    moveit2 = _moveit2_with_move_action_state()
    moveit2._MoveIt2__is_motion_requested = True
    moveit2._MoveIt2__is_executing = False
    goal_handle = _GoalHandle()

    _call_response_callback(
        moveit2,
        _Future(goal_handle),
        gen=moveit2._MoveIt2__move_generation - 1,
    )

    assert goal_handle.result_requested is False
    assert moveit2._MoveIt2__move_goal_handle is None
    assert moveit2._MoveIt2__is_executing is False


def test_accepted_move_action_response_stores_goal_handle_for_cancel():
    moveit2 = _moveit2_with_move_action_state()
    goal_handle = _GoalHandle()

    _call_response_callback(
        moveit2,
        _Future(goal_handle),
        gen=moveit2._MoveIt2__move_generation,
    )

    assert moveit2._MoveIt2__move_goal_handle is goal_handle
    assert goal_handle.result_requested is True
    assert len(goal_handle.result_future.callbacks) == 1
