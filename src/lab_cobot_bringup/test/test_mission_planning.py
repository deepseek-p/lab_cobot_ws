"""mission 接入 LLM 任务拆解的接线测试(Fake 注入,离线可跑)."""
from std_msgs.msg import String

from lab_cobot_bringup import mission_node
from lab_cobot_bringup.mission_node import MissionNode
from lab_cobot_bringup.task_planner import PlannerConfig, PlanResult
from lab_cobot_bringup.task_state_machine import STEP_ORDER, TaskState


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
