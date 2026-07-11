"""LLM 任务拆解规划器单元测试(纯逻辑,fake HTTP 注入,离线可跑)."""
import json

import pytest

from lab_cobot_bringup.task_planner import (
    DEFAULT_FULL_PLAN,
    MAX_PLAN_LENGTH,
    PlanValidationError,
    PlannerConfig,
    plan_actions,
    rule_based_plan,
    SYSTEM_PROMPT,
    validate_plan,
)
from lab_cobot_bringup.task_state_machine import STEP_ORDER, TaskState


def _fake_llm(actions):
    """构造返回固定动作序列的 fake http_post."""
    def http_post(url, headers, payload, timeout_sec):
        content = json.dumps({"actions": actions})
        return {"choices": [{"message": {"content": content}}]}
    return http_post


def _enabled_config():
    return PlannerConfig(llm_enabled=True, api_key="test-key")


# ---- validate_plan: 动作序列校验器 ----

def test_validate_accepts_full_default_sequence():
    names = [s.name for s in STEP_ORDER]
    assert validate_plan(names) == STEP_ORDER


def test_validate_rejects_unknown_action_name():
    with pytest.raises(PlanValidationError):
        validate_plan(["NAV_TO_PICK", "FLY_TO_MOON"])


def test_validate_rejects_pick_without_nav_and_detect_prerequisites():
    with pytest.raises(PlanValidationError):
        validate_plan(["PICK"])
    with pytest.raises(PlanValidationError):
        validate_plan(["NAV_TO_PICK", "PICK"])  # 缺 DETECT


def test_validate_rejects_place_without_pick_and_nav_prerequisites():
    with pytest.raises(PlanValidationError):
        validate_plan(["NAV_TO_PLACE", "PLACE"])  # 缺 PICK
    with pytest.raises(PlanValidationError):
        validate_plan(["NAV_TO_PICK", "DETECT", "PICK", "PLACE"])  # 缺 NAV_TO_PLACE


def test_validate_rejects_empty_plan():
    with pytest.raises(PlanValidationError):
        validate_plan([])


def test_validate_rejects_overlong_plan():
    names = ["RETURN_HOME"] * (MAX_PLAN_LENGTH + 1)
    with pytest.raises(PlanValidationError):
        validate_plan(names)


# ---- rule_based_plan: 离线关键词回退 ----

def test_rule_default_instruction_maps_to_full_sequence():
    assert rule_based_plan("把样件从A送到B") == DEFAULT_FULL_PLAN


def test_rule_inspect_station_a_maps_to_detect_and_return():
    plan = rule_based_plan("去A工位检查一下样件然后回家")
    assert plan == [TaskState.NAV_TO_PICK, TaskState.DETECT, TaskState.RETURN_HOME]


def test_rule_return_home_only():
    assert rule_based_plan("回到起始位置待命") == [TaskState.RETURN_HOME]


def test_rule_unknown_instruction_falls_back_to_full_sequence():
    # 保底原则:未命中的指令宁可做完整流程,不做非法/空计划
    assert rule_based_plan("给我唱首歌") == DEFAULT_FULL_PLAN
    assert rule_based_plan("") == DEFAULT_FULL_PLAN


def test_rule_non_sample_transport_maps_to_inspection_only():
    assert rule_based_plan("把试剂瓶送到B工位") == [
        TaskState.NAV_TO_PICK,
        TaskState.DETECT,
        TaskState.RETURN_HOME,
    ]
    assert rule_based_plan("把工具盒搬到B工位") == [
        TaskState.NAV_TO_PICK,
        TaskState.DETECT,
        TaskState.RETURN_HOME,
    ]


def test_rule_sample_transport_still_maps_to_full_sequence():
    assert rule_based_plan("把样件从A送到B") == DEFAULT_FULL_PLAN


def test_system_prompt_declares_only_aruco_sample_is_graspable():
    assert "只有贴 ArUco 码的白色样件可抓取搬运" in SYSTEM_PROMPT
    assert "试剂瓶/工具盒等器具只能识别查看" in SYSTEM_PROMPT


# ---- plan_actions: LLM 编排与回退 ----

def test_disabled_llm_uses_rule_fallback_without_http_call():
    calls = []

    def spy_http(url, headers, payload, timeout_sec):
        calls.append(url)
        return {}

    result = plan_actions(
        "把样件从A送到B",
        PlannerConfig(llm_enabled=False, api_key="key"),
        http_post=spy_http,
    )
    assert result.source == "fallback_disabled"
    assert result.steps == DEFAULT_FULL_PLAN
    assert calls == []


def test_missing_api_key_skips_http_and_falls_back():
    calls = []

    def spy_http(url, headers, payload, timeout_sec):
        calls.append(url)
        return {}

    result = plan_actions(
        "把样件从A送到B",
        PlannerConfig(llm_enabled=True, api_key=None),
        http_post=spy_http,
    )
    assert result.source == "fallback_disabled"
    assert calls == []


def test_llm_valid_json_plan_is_used():
    result = plan_actions(
        "去A工位检查一下样件然后回家",
        _enabled_config(),
        http_post=_fake_llm(["NAV_TO_PICK", "DETECT", "RETURN_HOME"]),
    )
    assert result.source == "llm"
    assert result.steps == [
        TaskState.NAV_TO_PICK,
        TaskState.DETECT,
        TaskState.RETURN_HOME,
    ]


def test_llm_http_error_falls_back_to_rule():
    def broken_http(url, headers, payload, timeout_sec):
        raise OSError("network unreachable")

    result = plan_actions("把样件从A送到B", _enabled_config(), http_post=broken_http)
    assert result.source == "fallback_rule"
    assert result.steps == DEFAULT_FULL_PLAN
    assert "network" in result.detail


def test_llm_malformed_json_falls_back_to_rule():
    def bad_json_http(url, headers, payload, timeout_sec):
        return {"choices": [{"message": {"content": "抱歉,我做不到"}}]}

    result = plan_actions("把样件从A送到B", _enabled_config(), http_post=bad_json_http)
    assert result.source == "fallback_rule"
    assert result.steps == DEFAULT_FULL_PLAN


def test_llm_invalid_plan_falls_back_to_rule():
    # LLM 返回结构合法但违反前置约束的计划 → 校验拦截 → 规则回退
    result = plan_actions(
        "把样件从A送到B",
        _enabled_config(),
        http_post=_fake_llm(["PLACE"]),
    )
    assert result.source == "fallback_rule"
    assert result.steps == DEFAULT_FULL_PLAN


def test_llm_json_code_fence_is_tolerated():
    def fenced_http(url, headers, payload, timeout_sec):
        content = '```json\n{"actions": ["RETURN_HOME"]}\n```'
        return {"choices": [{"message": {"content": content}}]}

    result = plan_actions("回家", _enabled_config(), http_post=fenced_http)
    assert result.source == "llm"
    assert result.steps == [TaskState.RETURN_HOME]


def test_llm_request_payload_carries_instruction_and_json_mode():
    captured = {}

    def spy_http(url, headers, payload, timeout_sec):
        captured.update(url=url, headers=headers, payload=payload)
        content = json.dumps({"actions": ["RETURN_HOME"]})
        return {"choices": [{"message": {"content": content}}]}

    config = PlannerConfig(llm_enabled=True, api_key="sk-test")
    plan_actions("回家待命", config, http_post=spy_http)

    assert captured["url"].endswith("/chat/completions")
    assert captured["headers"]["Authorization"] == "Bearer sk-test"
    assert captured["payload"]["temperature"] == 0
    assert captured["payload"]["response_format"] == {"type": "json_object"}
    user_messages = [
        m for m in captured["payload"]["messages"] if m["role"] == "user"
    ]
    assert any("回家待命" in m["content"] for m in user_messages)
