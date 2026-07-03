"""工位 waypoint 表与查询(纯逻辑,可单元测试)。

坐标为 map 系下的机器人停靠位姿:停在工作台前、朝向工作台(+y)。
工作台位置见 lab_cobot_gazebo/worlds/lab.world(工位A 2.0,1.5;工位B -2.0,1.5)。
"""
from __future__ import annotations

import math
from typing import Dict, List

# name -> {x, y, yaw(rad)}
WAYPOINTS: Dict[str, Dict[str, float]] = {
    "station_a": {"x": 2.0, "y": 0.85, "yaw": math.pi / 2.0},   # 工位A前,朝 +y
    "station_b": {"x": -2.0, "y": 0.45, "yaw": math.pi / 2.0},  # 工位B前,朝 +y
    "home": {"x": 0.0, "y": 0.0, "yaw": 0.0},
}


def get_waypoint(name: str) -> Dict[str, float]:
    if name not in WAYPOINTS:
        raise KeyError(f"未知工位: {name}(可用: {list_stations()})")
    return dict(WAYPOINTS[name])


def list_stations() -> List[str]:
    return sorted(WAYPOINTS.keys())


def yaw_to_quat(yaw: float):
    """平面 yaw -> 四元数 (x,y,z,w)。"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
