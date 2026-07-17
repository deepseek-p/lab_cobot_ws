"""跨工位任务状态机单元测试(纯逻辑,headless pytest 可跑)."""
import pytest

from lab_cobot_bringup.task_state_machine import (
    CrossStationTask,
    RouteState,
    SequentialTask,
    StationRouteTask,
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


# ---- SequentialTask: LLM 拆解出的自定义原子动作序列 ----

def test_sequential_task_runs_custom_plan_to_done():
    plan = [TaskState.NAV_TO_PICK, TaskState.DETECT, TaskState.RETURN_HOME]
    t = SequentialTask(plan, max_retries=1)
    t.start()
    visited = [t.state]
    for _ in range(len(plan)):
        t.on_result(True)
        visited.append(t.state)
    assert visited[:3] == plan
    assert t.state == TaskState.DONE
    assert t.is_terminal()


def test_sequential_task_rejects_empty_plan():
    with pytest.raises(ValueError):
        SequentialTask([], max_retries=1)


def test_sequential_task_rejects_non_plannable_states():
    for bad in (TaskState.IDLE, TaskState.DONE, TaskState.FAILED):
        with pytest.raises(ValueError):
            SequentialTask([TaskState.NAV_TO_PICK, bad], max_retries=1)


def test_sequential_task_mid_plan_retry_exhaustion_fails():
    plan = [TaskState.NAV_TO_PICK, TaskState.DETECT, TaskState.RETURN_HOME]
    t = SequentialTask(plan, max_retries=1)
    t.start()
    t.on_result(True)    # NAV_TO_PICK 成功 → DETECT
    t.on_result(False)   # DETECT 失败1次,重试
    t.on_result(False)   # 第2次失败,超过 max_retries=1
    assert t.state == TaskState.FAILED
    assert t.is_terminal()


def test_cross_station_task_is_default_sequential_plan():
    t = CrossStationTask(max_retries=2)
    assert isinstance(t, SequentialTask)
    assert t.steps == STEP_ORDER
    assert t.max_retries == 2


# ---- StationRouteTask: 单站与固定巡航路线 ----

def test_station_route_task_visits_stations_in_order():
    route = ["home", "station_a", "inspection_zone", "home"]
    task = StationRouteTask(route, max_retries=1)

    assert task.state == RouteState.IDLE
    assert task.start() == "home"
    visited = [task.current_station]
    while not task.is_terminal():
        task.on_result(True)
        if task.current_station is not None:
            visited.append(task.current_station)

    assert visited == route
    assert task.state == RouteState.DONE


def test_station_route_task_retries_current_station_once():
    task = StationRouteTask(["station_a", "station_b"], max_retries=1)
    task.start()

    task.on_result(False)
    assert task.current_station == "station_a"
    assert task.state == RouteState.RUNNING
    assert task.has_next_station

    task.on_result(True)
    assert task.current_station == "station_b"
    assert not task.has_next_station


def test_station_route_task_fails_after_retry_exhaustion():
    task = StationRouteTask(["tooling_zone"], max_retries=1)
    task.start()

    task.on_result(False)
    task.on_result(False)

    assert task.state == RouteState.FAILED
    assert task.current_station == "tooling_zone"


def test_station_route_task_rejects_empty_route():
    with pytest.raises(ValueError, match="stations"):
        StationRouteTask([])
