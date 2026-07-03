"""Unit tests for the cross-station task state machine."""
import pytest

from lab_cobot_bringup.task_state_machine import (
    CrossStationTask,
    TaskState,
    STEP_ORDER,
)


def test_initial_state_is_idle():
    t = CrossStationTask()
    assert t.state == TaskState.IDLE
    assert not t.is_terminal()


def test_start_enters_first_step():
    t = CrossStationTask()
    assert t.start() == TaskState.NAV_TO_PICK


def test_full_success_path_reaches_done():
    t = CrossStationTask()
    t.start()
    visited = [t.state]
    # 6 步全部成功
    for _ in range(len(STEP_ORDER)):
        t.on_result(True)
        visited.append(t.state)
    assert t.state == TaskState.DONE
    assert t.is_terminal()
    # 经过了全部步骤
    assert visited[:6] == STEP_ORDER
    assert visited[6] == TaskState.DONE


def test_single_failure_retries_same_step():
    t = CrossStationTask(max_retries=1)
    t.start()
    assert t.state == TaskState.NAV_TO_PICK
    # 失败一次(在 max_retries 内)→ 保持同一步重试
    t.on_result(False)
    assert t.state == TaskState.NAV_TO_PICK
    assert not t.is_terminal()
    # 重试成功 → 推进
    t.on_result(True)
    assert t.state == TaskState.DETECT


def test_exceed_retries_goes_failed():
    t = CrossStationTask(max_retries=1)
    t.start()
    t.on_result(False)  # 第1次失败,重试
    t.on_result(False)  # 第2次失败,超过 max_retries=1
    assert t.state == TaskState.FAILED
    assert t.is_terminal()


def test_zero_retries_fails_immediately():
    t = CrossStationTask(max_retries=0)
    t.start()
    t.on_result(False)
    assert t.state == TaskState.FAILED


def test_no_progress_after_terminal():
    t = CrossStationTask(max_retries=0)
    t.start()
    t.on_result(False)
    assert t.state == TaskState.FAILED
    # 终态后再调 on_result 不变
    assert t.on_result(True) == TaskState.FAILED
    assert t.on_result(False) == TaskState.FAILED


def test_on_result_in_idle_is_noop():
    t = CrossStationTask()
    assert t.on_result(True) == TaskState.IDLE


def test_negative_retries_raises():
    with pytest.raises(ValueError):
        CrossStationTask(max_retries=-1)


def test_retry_counter_resets_between_steps():
    t = CrossStationTask(max_retries=1)
    t.start()
    t.on_result(False)   # NAV_TO_PICK 失败1次
    t.on_result(True)    # 成功 → DETECT, 计数应重置
    t.on_result(False)   # DETECT 失败1次(应允许重试,而非直接失败)
    assert t.state == TaskState.DETECT
    assert not t.is_terminal()
