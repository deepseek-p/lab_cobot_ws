"""
LLM 任务拆解规划器:把中文指令拆解为原子动作序列(带离线规则回退).

设计要点:
- 纯逻辑模块,不 import rclpy;HTTP 通过 http_post 可注入,单测离线可跑
- llm_enabled 默认 False:诚实 E2E 与 CI 绝不出网;演示时经 launch 参数打开
- 任何 LLM 侧失败(网络/坏 JSON/违反前置约束)都降级到规则回退,流程不中断
- API key 只从环境变量读入后经 PlannerConfig 传入,本模块无任何硬编码密钥
"""
from __future__ import annotations

import json
import re
import urllib.request
from dataclasses import dataclass
from typing import Callable, Optional

from lab_cobot_bringup.task_state_machine import STEP_ORDER, TaskState
from lab_cobot_navigation.waypoints import CRUISE_ROUTE, normalize_station_name

MAX_PLAN_LENGTH = 12
DEFAULT_API_BASE = "https://api.deepseek.com"
DEFAULT_MODEL = "deepseek-chat"
DEFAULT_TIMEOUT_SEC = 10.0

# 保底计划:宁可执行完整搬运流程,不执行非法/空计划
DEFAULT_FULL_PLAN = list(STEP_ORDER)

_ACTION_BY_NAME = {state.name: state for state in STEP_ORDER}

SYSTEM_PROMPT = """你是实验室协作机器人的任务规划器。把用户的中文指令拆解为原子动作序列。

可用原子动作(只能使用这些名字,一字不差):
- NAV_TO_PICK: 导航到取件工位A并精确停靠
- DETECT: 在当前位置用相机识别样件
- PICK: 抓取样件(前置:本序列中之前必须已有 NAV_TO_PICK 和 DETECT)
- NAV_TO_PLACE: 导航到放置工位B并精确停靠
- PLACE: 放置样件(前置:本序列中之前必须已有 PICK 和 NAV_TO_PLACE)
- RETURN_HOME: 底盘回起始点并收拢机械臂

能力边界:
场景中只有贴 ArUco 码的白色样件可抓取搬运;试剂瓶/工具盒等器具只能识别查看。
若指令要求搬运样件以外的物体,输出巡检序列,禁止 PICK/PLACE。

规则:
1. 只输出 JSON,格式 {"actions": ["动作1", "动作2", ...]},禁止任何解释文字。
2. 序列最多 12 步,至少 1 步。
3. 指令含义不明确或超出能力范围时,输出完整搬运序列
   ["NAV_TO_PICK","DETECT","PICK","NAV_TO_PLACE","PLACE","RETURN_HOME"]。

示例:
指令: 把样件从A送到B
{"actions": ["NAV_TO_PICK", "DETECT", "PICK", "NAV_TO_PLACE", "PLACE", "RETURN_HOME"]}
指令: 去A工位检查一下样件然后回来
{"actions": ["NAV_TO_PICK", "DETECT", "RETURN_HOME"]}
指令: 把试剂瓶送到B工位
{"actions": ["NAV_TO_PICK", "DETECT", "RETURN_HOME"]}
指令: 回到起始位置待命
{"actions": ["RETURN_HOME"]}
"""

HttpPost = Callable[[str, dict, dict, float], dict]


class PlanValidationError(ValueError):
    """动作序列违反合法性或前置约束."""


class NavigationRequestError(ValueError):
    """Navigation instruction names an invalid station."""

    def __init__(self, station: str):
        self.station = station
        super().__init__(f"未知工位: {station}")


@dataclass(frozen=True)
class PlannerConfig:
    """规划器配置;api_key 由调用方从环境变量注入."""

    llm_enabled: bool = False
    api_base: str = DEFAULT_API_BASE
    model: str = DEFAULT_MODEL
    timeout_sec: float = DEFAULT_TIMEOUT_SEC
    api_key: Optional[str] = None


@dataclass(frozen=True)
class PlanResult:
    """拆解结果:steps 为 TaskState 序列,source 标注来源供日志/答辩取证."""

    steps: list
    source: str  # "llm" | "fallback_rule" | "fallback_disabled"
    detail: str = ""


@dataclass(frozen=True)
class NavigationRequest:
    """Deterministic single-station or cruise navigation request."""

    mode: str
    stations: tuple[str, ...]


_CRUISE_COMMANDS = frozenset(
    (
        "巡航所有工位",
        "巡航全部工位",
        "按顺序访问全部工位并回家",
    )
)
_COMPOUND_MISSION_MARKERS = (
    "然后",
    "并且",
    "检查",
    "确认",
    "识别",
    "抓取",
    "搬运",
    "送到",
    "放到",
    "回来",
    "回家",
)


def parse_navigation_request(instruction: str) -> Optional[NavigationRequest]:
    """Parse deterministic navigation commands before the legacy planner."""
    text = (instruction or "").strip().rstrip("。！？!?").strip()
    compact = re.sub(r"\s+", "", text)
    if compact in _CRUISE_COMMANDS:
        return NavigationRequest(mode="cruise", stations=tuple(CRUISE_ROUTE))

    direct = re.fullmatch(r"(?:请)?(?:导航到|前往|移动到|去)\s*(.+)", text)
    if direct is None:
        return None

    target = direct.group(1).strip()
    if any(marker in target for marker in _COMPOUND_MISSION_MARKERS):
        return None
    for suffix in ("看一下", "看看", "一下"):
        if target.endswith(suffix):
            target = target[:-len(suffix)].strip()
            break

    try:
        station = normalize_station_name(target)
    except KeyError as exc:
        raise NavigationRequestError(target) from exc
    return NavigationRequest(mode="single", stations=(station,))


def validate_plan(action_names) -> list[TaskState]:
    """校验动作名序列合法性与前置约束,通过则返回 TaskState 序列."""
    names = list(action_names)
    if not names:
        raise PlanValidationError("计划为空")
    if len(names) > MAX_PLAN_LENGTH:
        raise PlanValidationError(f"计划超长: {len(names)} > {MAX_PLAN_LENGTH}")
    unknown = [n for n in names if n not in _ACTION_BY_NAME]
    if unknown:
        raise PlanValidationError(f"未知动作: {unknown}")

    seen: set[str] = set()
    for name in names:
        if name == "PICK":
            missing = {"NAV_TO_PICK", "DETECT"} - seen
            if missing:
                raise PlanValidationError(f"PICK 缺前置: {sorted(missing)}")
        if name == "PLACE":
            missing = {"PICK", "NAV_TO_PLACE"} - seen
            if missing:
                raise PlanValidationError(f"PLACE 缺前置: {sorted(missing)}")
        seen.add(name)
    return [_ACTION_BY_NAME[n] for n in names]


def rule_based_plan(instruction: str) -> list[TaskState]:
    """离线关键词规则拆解;未命中一律回退完整搬运序列(保底原则)."""
    text = (instruction or "").strip()
    has_inspect = any(kw in text for kw in ("检查", "看看", "看一下", "确认"))
    mentions_pick_station = any(kw in text for kw in ("A工位", "工位A", "A 工位"))
    only_home = any(kw in text for kw in ("回家", "回到起始", "待命", "返回原位"))
    mentions_transport = any(kw in text for kw in ("送到", "搬到", "放到", "搬运"))
    mentions_non_sample_object = any(
        kw in text for kw in ("试剂瓶", "工具盒", "工具箱")
    )
    mentions_sample = "样件" in text

    if mentions_transport and mentions_non_sample_object and not mentions_sample:
        return [TaskState.NAV_TO_PICK, TaskState.DETECT, TaskState.RETURN_HOME]
    if has_inspect and mentions_pick_station and not mentions_transport:
        return [TaskState.NAV_TO_PICK, TaskState.DETECT, TaskState.RETURN_HOME]
    if only_home and not mentions_transport and not has_inspect:
        return [TaskState.RETURN_HOME]
    return list(DEFAULT_FULL_PLAN)


def plan_actions(
    instruction: str,
    config: Optional[PlannerConfig] = None,
    http_post: Optional[HttpPost] = None,
) -> PlanResult:
    """指令 → 动作序列:优先 LLM 拆解,失败降级规则回退,永不抛出."""
    cfg = config or PlannerConfig()
    if not cfg.llm_enabled or not cfg.api_key:
        return PlanResult(
            steps=rule_based_plan(instruction),
            source="fallback_disabled",
            detail="llm_enabled=False" if not cfg.llm_enabled else "缺少 LLM_API_KEY",
        )

    post = http_post or _urllib_http_post
    try:
        response = post(
            cfg.api_base.rstrip("/") + "/chat/completions",
            {
                "Authorization": f"Bearer {cfg.api_key}",
                "Content-Type": "application/json",
            },
            {
                "model": cfg.model,
                "temperature": 0,
                "response_format": {"type": "json_object"},
                "messages": [
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": f"指令: {instruction}"},
                ],
            },
            cfg.timeout_sec,
        )
        content = response["choices"][0]["message"]["content"]
        steps = validate_plan(_parse_actions_json(content))
        return PlanResult(steps=steps, source="llm", detail=cfg.model)
    except Exception as exc:  # noqa: BLE001 规划失败必须降级而非中断任务
        return PlanResult(
            steps=rule_based_plan(instruction),
            source="fallback_rule",
            detail=f"{type(exc).__name__}: {exc}",
        )


def _parse_actions_json(content: str) -> list:
    """解析 LLM 输出的 {"actions": [...]},容忍 ```json 围栏."""
    text = content.strip()
    fence = re.match(r"^```(?:json)?\s*(.*?)\s*```$", text, flags=re.DOTALL)
    if fence:
        text = fence.group(1)
    data = json.loads(text)
    actions = data.get("actions")
    if not isinstance(actions, list):
        raise PlanValidationError("JSON 中缺少 actions 列表")
    return actions


def _urllib_http_post(url: str, headers: dict, payload: dict, timeout_sec: float) -> dict:
    """标准库实现的 JSON POST(唯一 I/O 点,测试中被注入替换)."""
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers=headers,
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout_sec) as response:
        return json.loads(response.read().decode("utf-8"))
