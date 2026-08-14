"""
工位 waypoint 表与查询(纯逻辑,可单元测试).

坐标为 map 系下的机器人停靠位姿:停在工作台前、朝向工作台(+y).
五功能区沿北侧共享走廊排布。规范键(station_a / tooling_zone / aging_zone /
station_b / inspection_zone / home)保持稳定以兼容 mission 代码;物理语义
(2026-08-13 起)为:station_a=物料区, tooling_zone=工装工具区,
aging_zone=板卡测试台(原老化桌), station_b=老化实验台(原板卡桌),
inspection_zone=高压试验区。

自 2026-08-12 起,STATION_SPECS 是路线、dock、方向和 clearance 的唯一运行时来源。
WAYPOINTS 从 nav_pose 派生,保持向后兼容。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float
    frame_id: str = "map"


@dataclass(frozen=True)
class RouteLeg:
    name: str
    pose: Pose2D
    dock_station: str | None = None


@dataclass(frozen=True)
class WorkSurface:
    center_x: float
    center_y: float
    size_x: float
    size_y: float


@dataclass(frozen=True)
class StationSpec:
    name: str
    nav_legs: tuple[RouteLeg, ...]
    nav_pose: Pose2D
    dock_pose: Pose2D
    approach_side: str
    work_surface: WorkSurface | None
    reference_object_xy: tuple[float, float] | None
    clearance_m: float


# ── 机器人几何常量 ──────────────────────────────────────────
_CHASSIS_LENGTH = 0.55
_CHASSIS_WIDTH = 0.50
_WORKTABLE_HALF_Y = 0.60  # 1.2m 桌面 / 2

# ── 台面站 dock y 计算 ───────────────────────────────────────
# 所有台面站: 机器人从南侧接近,朝北 (yaw=π/2)
# dock_pose.y = table_center_y - table_half_y - clearance - chassis_half_length


def _dock_y(table_center_y: float, clearance: float = 0.18) -> float:
    return (
        table_center_y - _WORKTABLE_HALF_Y - clearance - _CHASSIS_LENGTH / 2.0
    )


# ── 唯一运行时 Station 数据源 ─────────────────────────────────

STATION_SPECS: Dict[str, StationSpec] = {
    "station_a": StationSpec(
        name="station_a",
        nav_legs=(
            RouteLeg(
                name="station_a",
                pose=Pose2D(x=-4.30, y=2.57, yaw=math.pi / 2.0),
                dock_station="station_a",
            ),
        ),
        nav_pose=Pose2D(x=-4.30, y=2.57, yaw=math.pi / 2.0),
        dock_pose=Pose2D(x=-4.30, y=_dock_y(3.80), yaw=math.pi / 2.0),
        approach_side="south",
        work_surface=WorkSurface(
            center_x=-4.30, center_y=3.80, size_x=1.60, size_y=1.20,
        ),
        reference_object_xy=(-4.16, 3.46),
        clearance_m=0.18,
    ),
    "inspection_zone": StationSpec(
        name="inspection_zone",
        nav_legs=(
            RouteLeg(
                name="inspection_zone_south_corridor_entry",
                pose=Pose2D(x=2.00, y=1.10, yaw=math.pi / 2.0),
                dock_station=None,
            ),
            RouteLeg(
                name="inspection_zone",
                pose=Pose2D(x=4.10, y=1.10, yaw=math.pi / 2.0),
                dock_station="inspection_zone",
            ),
        ),
        nav_pose=Pose2D(x=4.10, y=1.10, yaw=math.pi / 2.0),
        dock_pose=Pose2D(x=4.10, y=1.10, yaw=math.pi / 2.0),
        approach_side="none",
        work_surface=None,
        reference_object_xy=None,
        clearance_m=0.60,
    ),
    "tooling_zone": StationSpec(
        name="tooling_zone",
        nav_legs=(
            RouteLeg(
                name="tooling_zone_corridor_entry",
                pose=Pose2D(x=1.90, y=-3.23, yaw=math.pi / 2.0),
                dock_station=None,
            ),
            RouteLeg(
                name="tooling_zone",
                pose=Pose2D(x=-4.10, y=-3.53, yaw=math.pi / 2.0),
                dock_station="tooling_zone",
            ),
        ),
        nav_pose=Pose2D(x=-4.10, y=-3.53, yaw=math.pi / 2.0),
        dock_pose=Pose2D(x=-4.10, y=_dock_y(-2.30), yaw=math.pi / 2.0),
        approach_side="south",
        work_surface=WorkSurface(
            center_x=-4.10, center_y=-2.30, size_x=1.60, size_y=1.20,
        ),
        reference_object_xy=(-3.88, -2.04),
        clearance_m=0.18,
    ),
    "aging_zone": StationSpec(
        name="aging_zone",
        nav_legs=(
            RouteLeg(
                name="aging_zone_south_entry",
                pose=Pose2D(x=2.00, y=-5.05, yaw=math.pi / 2.0),
                dock_station=None,
            ),
            RouteLeg(
                name="aging_zone_east_corridor",
                pose=Pose2D(x=2.00, y=2.97, yaw=math.pi / 2.0),
                dock_station=None,
            ),
            RouteLeg(
                name="aging_zone",
                pose=Pose2D(x=0.20, y=2.97, yaw=math.pi / 2.0),
                dock_station="aging_zone",
            ),
        ),
        nav_pose=Pose2D(x=0.20, y=2.97, yaw=math.pi / 2.0),
        dock_pose=Pose2D(x=0.20, y=_dock_y(4.20), yaw=math.pi / 2.0),
        approach_side="south",
        work_surface=WorkSurface(
            center_x=0.20, center_y=4.20, size_x=1.60, size_y=1.20,
        ),
        reference_object_xy=(0.20, 4.26),
        clearance_m=0.18,
    ),
    "station_b": StationSpec(
        name="station_b",
        nav_legs=(
            RouteLeg(
                name="station_b",
                pose=Pose2D(x=0.30, y=-2.93, yaw=math.pi / 2.0),
                dock_station="station_b",
            ),
        ),
        nav_pose=Pose2D(x=0.30, y=-2.93, yaw=math.pi / 2.0),
        dock_pose=Pose2D(x=0.30, y=_dock_y(-1.70), yaw=math.pi / 2.0),
        approach_side="south",
        work_surface=WorkSurface(
            center_x=0.30, center_y=-1.70, size_x=1.60, size_y=1.20,
        ),
        reference_object_xy=(0.02, -1.44),
        clearance_m=0.18,
    ),
    "home": StationSpec(
        name="home",
        nav_legs=(
            RouteLeg(
                name="home",
                pose=Pose2D(x=4.50, y=-4.20, yaw=0.0),
                dock_station="home",
            ),
        ),
        nav_pose=Pose2D(x=4.50, y=-4.20, yaw=0.0),
        dock_pose=Pose2D(x=4.50, y=-4.20, yaw=0.0),
        approach_side="none",
        work_surface=None,
        reference_object_xy=None,
        clearance_m=0.60,
    ),
}


# ── 向后兼容 API ─────────────────────────────────────────────

# WAYPOINTS 从 STATION_SPECS.nav_pose 派生
WAYPOINTS: Dict[str, Dict[str, float]] = {
    name: {
        "x": spec.nav_pose.x, "y": spec.nav_pose.y, "yaw": spec.nav_pose.yaw,
    }
    for name, spec in STATION_SPECS.items()
}

CRUISE_ROUTE = (
    "home",
    "station_a",
    "inspection_zone",
    "tooling_zone",
    "aging_zone",
    "station_b",
    "home",
)

_STATION_ALIASES = {
    "station_a": "station_a",
    "a工位": "station_a",
    "工位a": "station_a",
    "物料区": "station_a",
    "inspection_zone": "inspection_zone",
    "检测区": "inspection_zone",
    "高压试验区": "inspection_zone",
    "tooling_zone": "tooling_zone",
    "工具区": "tooling_zone",
    "工装区": "tooling_zone",
    "工装工具区": "tooling_zone",
    "aging_zone": "aging_zone",
    "板卡测试台": "aging_zone",
    "station_b": "station_b",
    "b工位": "station_b",
    "工位b": "station_b",
    "老化实验台": "station_b",
    "老化区": "station_b",
    "home": "home",
    "起始点": "home",
}


def get_station_spec(name: str) -> StationSpec:
    """按规范名返回 StationSpec."""
    canonical = normalize_station_name(name)
    return STATION_SPECS[canonical]


def get_waypoint(name: str) -> Dict[str, float]:
    if name not in WAYPOINTS:
        raise KeyError(f"未知工位: {name}(可用: {list_stations()})")
    return dict(WAYPOINTS[name])


def list_stations() -> List[str]:
    return sorted(WAYPOINTS.keys())


def normalize_station_name(name: str) -> str:
    """Normalize a station alias to its canonical waypoint name."""
    key = "".join(str(name).strip().lower().split())
    station = _STATION_ALIASES.get(key)
    if station is None:
        raise KeyError(f"未知工位: {name}(可用: {list_stations()})")
    return station


def yaw_to_quat(yaw: float):
    """平面 yaw -> 四元数 (x,y,z,w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
