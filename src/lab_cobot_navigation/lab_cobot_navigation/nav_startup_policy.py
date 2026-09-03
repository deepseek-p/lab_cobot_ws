"""Nav2 启动就绪策略(纯逻辑,不依赖 ROS 运行时,便于 headless 单测)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

LIFECYCLE_CONFIGURE = 1
LIFECYCLE_ACTIVATE = 3

STATE_UNCONFIGURED = 1
STATE_INACTIVE = 2
STATE_ACTIVE = 3

MAP_SERVER_NAME = "map_server"
AMCL_NAME = "amcl"


@dataclass(frozen=True)
class Readiness:
    map_received: bool
    map_to_odom_ready: bool
    lifecycle_states: Dict[str, int]


@dataclass(frozen=True)
class RecoveryPlan:
    node_name: str
    transitions: Tuple[int, ...]


def transitions_to_active(state_id: int) -> Tuple[int, ...]:
    """返回把 lifecycle 节点带到 active 的转换 id 序列."""
    if state_id == STATE_ACTIVE:
        return ()
    if state_id == STATE_INACTIVE:
        return (LIFECYCLE_ACTIVATE,)
    # unconfigured / unknown / 中间态都按 configure+activate 幂等重试
    return (LIFECYCLE_CONFIGURE, LIFECYCLE_ACTIVATE)


def decide_recovery(readiness: Readiness) -> Optional[RecoveryPlan]:
    """根据地图与 TF 就绪状态决定下一步:None 表示已就绪."""
    if readiness.map_received and readiness.map_to_odom_ready:
        return None
    node_name = MAP_SERVER_NAME if not readiness.map_received else AMCL_NAME
    state_id = readiness.lifecycle_states.get(node_name, STATE_UNCONFIGURED)
    return RecoveryPlan(
        node_name=node_name,
        transitions=transitions_to_active(state_id),
    )
