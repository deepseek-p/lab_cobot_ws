"""mission 接入 LLM 任务拆解的接线测试(Fake 注入,离线可跑)."""
from std_msgs.msg import String

from lab_cobot_bringup import mission_node
from lab_cobot_bringup.mission_node import MissionNode
from lab_cobot_bringup.task_planner import (
    NavigationRequest,
    PlannerConfig,
    PlanResult,
)
from lab_cobot_bringup.task_state_machine import STEP_ORDER, TaskState
from lab_cobot_navigation.waypoints import CRUISE_ROUTE, get_waypoint


class FakeLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def _make_node(executed, published):
    node = MissionNode.__new__(MissionNode)
    node._busy = True
    node._wait_for_navigation_ready = lambda: True
    node._execute = lambda state: executed.append(state) or True
    node._publish = lambda state: published.append(state.name)
    node.get_logger = lambda: FakeLogger()
    return node


def test_mission_executes_planned_sequence(monkeypatch):
    plan = [TaskState.NAV_TO_PICK, TaskState.DETECT, TaskState.RETURN_HOME]
    monkeypatch.setattr(
        mission_node,
        "plan_actions",
        lambda instruction, config=None: PlanResult(plan, "llm", "test"),
    )
    executed, published = [], []
    node = _make_node(executed, published)
    node._instruction = "去A工位检查一下样件然后回家"

    MissionNode._run_mission(node)

    assert executed == plan
    assert published[-1] == "DONE"
    assert node._busy is False


def test_mission_uses_rule_fallback_for_invalid_llm_plan(monkeypatch):
    # planner 层已把非法 LLM 计划降级为规则回退,mission 只消费 PlanResult
    monkeypatch.setattr(
        mission_node,
        "plan_actions",
        lambda instruction, config=None: PlanResult(
            list(STEP_ORDER), "fallback_rule", "PlanValidationError"
        ),
    )
    executed, published = [], []
    node = _make_node(executed, published)
    node._instruction = "把样件从A送到B"

    MissionNode._run_mission(node)

    assert executed == STEP_ORDER
    assert published[-1] == "DONE"


def test_on_instruction_records_text_before_starting_thread(monkeypatch):
    starts = []

    class FakeThread:
        def __init__(self, target, daemon):
            # 线程构造时指令必须已就位(daemon 线程读 self._instruction)
            starts.append(getattr(node, "_instruction", None))

        def start(self):
            pass

    monkeypatch.setattr(mission_node, "Thread", FakeThread)
    node = MissionNode.__new__(MissionNode)
    node._busy = False
    node.get_logger = lambda: FakeLogger()

    msg = String()
    msg.data = "去A工位检查一下样件然后回家"
    MissionNode._on_instruction(node, msg)

    assert starts == ["去A工位检查一下样件然后回家"]
    assert node._busy is True


def test_default_instruction_without_config_runs_full_sequence():
    # 守住诚实 E2E 语义:默认指令 + 无 planner 配置 → 完整跨工位流程
    executed, published = [], []
    node = _make_node(executed, published)
    node._instruction = "把样件从A送到B"

    MissionNode._run_mission(node)

    assert executed == STEP_ORDER
    assert published[-1] == "DONE"


def test_mission_planner_config_reads_key_from_environment(monkeypatch):
    monkeypatch.setenv("LLM_API_KEY", "sk-env-test")
    config = mission_node.build_planner_config(
        llm_enabled=True,
        api_base="https://example.com",
        model="test-model",
        timeout_sec=5.0,
    )
    assert isinstance(config, PlannerConfig)
    assert config.api_key == "sk-env-test"
    assert config.llm_enabled is True


def _make_pick_step_node(events, pick_ok=True, tuck_ok=True):
    class FakePickPlace:
        def pick(self, pose, refine_cb=None):
            events.append("pick")
            return pick_ok

        def go_home(self):
            events.append("tuck")
            return tuck_ok

    node = MissionNode.__new__(MissionNode)
    node.get_logger = lambda: FakeLogger()
    node._latest_task_detection = [0.8, 0.0, 0.63]
    node._finish_station_step = (
        lambda state, ok: events.append("retreat") or ok
    )
    node.pp = FakePickPlace()
    return node


def test_pick_step_tucks_arm_after_retreat_before_navigation():
    # PICK 成功链:抓取 → 退避 → 持物收臂。收臂必须在退避之后
    # (车已离台,收臂弧远离台面),导航之前(调头旋转不再横扫工位)。
    events = []
    node = _make_pick_step_node(events)

    assert MissionNode._execute(node, TaskState.PICK)
    assert events == ["pick", "retreat", "tuck"]


def test_pick_step_skips_tuck_when_pick_fails():
    events = []
    node = _make_pick_step_node(events, pick_ok=False)

    assert not MissionNode._execute(node, TaskState.PICK)
    assert "tuck" not in events


def test_pick_step_tuck_failure_degrades_without_failing_task():
    # 收臂失败只降级为旧行为(伸展位形导航),不把新增步骤变成任务级失败点。
    events = []
    node = _make_pick_step_node(events, tuck_ok=False)

    assert MissionNode._execute(node, TaskState.PICK)
    assert events == ["pick", "retreat", "tuck"]


class FakeRoutePickPlace:
    def __init__(self, events, go_home_ok=True):
        self.events = events
        self.go_home_ok = go_home_ok

    def go_home(self):
        self.events.append("arm_home")
        return self.go_home_ok


def _make_route_node(events, statuses):
    node = MissionNode.__new__(MissionNode)
    node.pp = FakeRoutePickPlace(events)
    node._base_pose_in_map = lambda timeout_sec=2.0: [0.0, 0.0, 0.0]
    node._navigate = lambda station: events.append(("navigate", station)) or True
    node._dock_to_station_pose = (
        lambda station: events.append(("dock", station)) or True
    )
    node._retreat_from_route_station = (
        lambda station: events.append(("depart", station)) or True
    )
    node._publish_status = statuses.append
    node._stop_base = lambda duration: events.append(("stop", duration))
    node.get_logger = lambda: FakeLogger()
    return node


def test_single_station_navigation_stops_at_requested_station():
    events, statuses = [], []
    node = _make_route_node(events, statuses)
    request = NavigationRequest(mode="single", stations=("inspection_zone",))

    assert MissionNode._run_navigation_request(node, request)

    assert events == [
        "arm_home",
        ("navigate", "inspection_zone"),
        ("dock", "inspection_zone"),
    ]
    assert statuses == [
        "NAV_TO_STATION:inspection_zone",
        "ARRIVED:inspection_zone",
        "DONE",
    ]


def test_cruise_from_non_home_returns_home_before_confirmed_route():
    events, statuses = [], []
    node = _make_route_node(events, statuses)
    request = NavigationRequest(mode="cruise", stations=CRUISE_ROUTE)

    assert MissionNode._run_navigation_request(node, request)

    navigated = [
        event[1]
        for event in events
        if isinstance(event, tuple) and event[0] == "navigate"
    ]
    assert navigated == list(CRUISE_ROUTE)
    departed = [
        event[1]
        for event in events
        if isinstance(event, tuple) and event[0] == "depart"
    ]
    assert departed == [
        "station_a",
        "tooling_zone",
        "aging_zone",
        "station_b",
    ]
    assert statuses[0:2] == ["RETURN_HOME", "ARRIVED:home"]
    assert statuses[-3:] == ["RETURN_HOME", "ARRIVED:home", "DONE"]


def test_cruise_at_home_skips_duplicate_initial_home_goal():
    events, statuses = [], []
    node = _make_route_node(events, statuses)
    home = get_waypoint("home")
    node._base_pose_in_map = lambda timeout_sec=2.0: [
        home["x"],
        home["y"],
        home["yaw"],
    ]
    request = NavigationRequest(mode="cruise", stations=CRUISE_ROUTE)

    assert MissionNode._run_navigation_request(node, request)

    navigated = [
        event[1]
        for event in events
        if isinstance(event, tuple) and event[0] == "navigate"
    ]
    assert navigated == list(CRUISE_ROUTE[1:])
    assert statuses[0] == "NAV_TO_STATION:station_a"


def test_route_failure_retries_then_stops_and_publishes_compatible_failed():
    events, statuses = [], []
    node = _make_route_node(events, statuses)
    node._navigate = lambda station: events.append(("navigate", station)) or False
    request = NavigationRequest(mode="single", stations=("tooling_zone",))

    assert not MissionNode._run_navigation_request(node, request)

    assert events.count(("navigate", "tooling_zone")) == 2
    assert not any(event == ("dock", "tooling_zone") for event in events)
    assert any(event[0] == "stop" for event in events if isinstance(event, tuple))
    assert statuses[-2:] == [
        "FAILED:navigation_failed:tooling_zone",
        "FAILED",
    ]


def test_route_docking_failure_reports_station_and_stops():
    events, statuses = [], []
    node = _make_route_node(events, statuses)
    node._dock_to_station_pose = (
        lambda station: events.append(("dock", station)) or False
    )
    request = NavigationRequest(mode="single", stations=("aging_zone",))

    assert not MissionNode._run_navigation_request(node, request)

    assert events.count(("navigate", "aging_zone")) == 2
    assert events.count(("dock", "aging_zone")) == 2
    assert statuses[-2:] == [
        "FAILED:docking_failed:aging_zone",
        "FAILED",
    ]


def test_route_arm_stow_failure_sends_no_navigation_goal():
    events, statuses = [], []
    node = _make_route_node(events, statuses)
    node.pp = FakeRoutePickPlace(events, go_home_ok=False)
    request = NavigationRequest(mode="single", stations=("station_a",))

    assert not MissionNode._run_navigation_request(node, request)

    assert not any(
        isinstance(event, tuple) and event[0] == "navigate"
        for event in events
    )
    assert statuses[-2:] == ["FAILED:arm_not_stowed", "FAILED"]


def test_run_mission_dispatches_navigation_before_legacy_planner(monkeypatch):
    request = NavigationRequest(mode="single", stations=("station_b",))
    monkeypatch.setattr(
        mission_node,
        "parse_navigation_request",
        lambda _instruction: request,
    )

    def fail_if_legacy_planner_runs(*_args, **_kwargs):
        raise AssertionError("legacy planner must not run")

    monkeypatch.setattr(mission_node, "plan_actions", fail_if_legacy_planner_runs)
    events = []
    node = MissionNode.__new__(MissionNode)
    node._instruction = "去B工位"
    node._planner_config = None
    node._busy = True
    node._wait_for_navigation_ready = lambda: events.append("ready") or True
    node._run_navigation_request = (
        lambda actual: events.append(("route", actual)) or True
    )
    node.get_logger = lambda: FakeLogger()

    MissionNode._run_mission(node)

    assert events == ["ready", ("route", request)]
    assert node._busy is False


def test_invalid_station_fails_before_waiting_for_nav2():
    events = []
    node = MissionNode.__new__(MissionNode)
    node._instruction = "导航到充电区"
    node._planner_config = None
    node._busy = True
    node._wait_for_navigation_ready = lambda: events.append("ready") or True
    node._publish_failure = (
        lambda reason, station=None: events.append(("failure", reason, station))
    )
    node.get_logger = lambda: FakeLogger()

    MissionNode._run_mission(node)

    assert events == [("failure", "unknown_station", "充电区")]
    assert node._busy is False
