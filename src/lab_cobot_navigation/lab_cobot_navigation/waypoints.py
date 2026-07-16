"""
工位 waypoint 表与查询(纯逻辑,可单元测试).

坐标为 map 系下的机器人停靠位姿:停在工作台前、朝向工作台(+y).
五功能区沿北侧共享走廊排布,其中 station_a / station_b 保持原有任务语义。
"""
from __future__ import annotations

import math
from typing import Dict, List

# name -> {x, y, yaw(rad)}
WAYPOINTS: Dict[str, Dict[str, float]] = {
    "station_a": {"x": -2.15, "y": 0.78, "yaw": math.pi / 2.0},
    "inspection_zone": {"x": 2.05, "y": 0.55, "yaw": math.pi / 2.0},
    "tooling_zone": {"x": -2.05, "y": -1.95, "yaw": math.pi / 2.0},
    "aging_zone": {"x": 0.10, "y": 1.30, "yaw": math.pi / 2.0},
    "station_b": {"x": 0.15, "y": -1.96, "yaw": math.pi / 2.0},
    "home": {"x": 2.25, "y": -2.10, "yaw": 0.0},
}


def get_waypoint(name: str) -> Dict[str, float]:
    if name not in WAYPOINTS:
        raise KeyError(f"未知工位: {name}(可用: {list_stations()})")
    return dict(WAYPOINTS[name])


def list_stations() -> List[str]:
    return sorted(WAYPOINTS.keys())


def yaw_to_quat(yaw: float):
    """平面 yaw -> 四元数 (x,y,z,w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
