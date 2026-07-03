"""
Cross-station single-object task state machine.

运行时由 mission_node 驱动:每个状态对应一个动作(导航/识别/抓取/放置),
执行后用 on_result(success) 推进、重试或判失败。
"""
from __future__ import annotations

from enum import Enum, auto


class TaskState(Enum):
    IDLE = auto()
    NAV_TO_PICK = auto()      # 导航到取件工位 A
    DETECT = auto()            # ArUco 识别样件
    PICK = auto()              # MoveIt 抓取 + 吸附
    NAV_TO_PLACE = auto()      # 导航到放置工位 B
    PLACE = auto()             # MoveIt 放置 + 释放
    RETURN_HOME = auto()       # 返回 home
    DONE = auto()              # 成功终态
    FAILED = auto()            # 失败终态


# 正常执行顺序(不含终态)
STEP_ORDER = [
    TaskState.NAV_TO_PICK,
    TaskState.DETECT,
    TaskState.PICK,
    TaskState.NAV_TO_PLACE,
    TaskState.PLACE,
    TaskState.RETURN_HOME,
]


class CrossStationTask:
    """
    State machine for cross-station pick-and-place.

    - start(): 从 IDLE 进入第一步
    - on_result(success): 成功→下一步;失败→在 max_retries 内重试,超出→FAILED
    - is_terminal(): 是否到达 DONE / FAILED
    """

    def __init__(self, max_retries: int = 1):
        if max_retries < 0:
            raise ValueError("max_retries 不能为负")
        self.max_retries = max_retries
        self.state = TaskState.IDLE
        self._idx = -1
        self.attempts = 0  # 当前步已尝试次数

    def start(self) -> TaskState:
        self._idx = 0
        self.state = STEP_ORDER[0]
        self.attempts = 0
        return self.state

    def is_terminal(self) -> bool:
        return self.state in (TaskState.DONE, TaskState.FAILED)

    def on_result(self, success: bool) -> TaskState:
        # IDLE 或终态下不响应
        if self.state == TaskState.IDLE or self.is_terminal():
            return self.state
        if success:
            self._advance()
        else:
            self.attempts += 1
            if self.attempts > self.max_retries:
                self.state = TaskState.FAILED
            # 否则保持当前状态以重试
        return self.state

    def _advance(self) -> None:
        self._idx += 1
        self.attempts = 0
        if self._idx >= len(STEP_ORDER):
            self.state = TaskState.DONE
        else:
            self.state = STEP_ORDER[self._idx]
