"""Regression tests for MoveIt2 execution status reporting."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
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


def _moveit2_with_status(status):
    moveit2 = object.__new__(MoveIt2)
    moveit2._MoveIt2__is_motion_requested = True
    moveit2._MoveIt2__is_executing = False
    moveit2._MoveIt2__wait_until_executed_rate = _Rate(moveit2)
    moveit2._MoveIt2__last_execution_succeeded = status
    return moveit2


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

    assert moveit2.wait_until_executed(timeout_sec=0.0) is False
    assert moveit2._MoveIt2__is_motion_requested is False
    assert moveit2._MoveIt2__is_executing is False
