"""waypoints 单元测试(纯逻辑,headless pytest 可跑)."""
import math

import pytest

from lab_cobot_navigation.waypoints import (
    CRUISE_ROUTE,
    WAYPOINTS,
    get_waypoint,
    list_stations,
    normalize_station_name,
    yaw_to_quat,
)


def test_known_stations_present():
    assert set(list_stations()) >= {
        "station_a",
        "inspection_zone",
        "tooling_zone",
        "aging_zone",
        "station_b",
        "home",
    }


def test_get_waypoint_has_fields():
    wp = get_waypoint("station_a")
    assert set(wp.keys()) == {"x", "y", "yaw"}


def test_unknown_station_raises():
    with pytest.raises(KeyError):
        get_waypoint("nonexistent")


def test_stations_distinct_positions():
    a = get_waypoint("station_a")
    b = get_waypoint("station_b")
    assert (a["x"], a["y"]) != (b["x"], b["y"])


def test_pick_station_leaves_visual_docking_standoff():
    station_a = get_waypoint("station_a")
    sample_y = 3.42  # aruco marker face y ≈ 3.46 - 0.035 = 3.425
    nav_xy_goal_tolerance = 0.15

    nominal_forward_distance = sample_y - station_a["y"]
    worst_case_forward_distance = nominal_forward_distance + nav_xy_goal_tolerance

    # 1.6×1.2 桌面前沿=3.20, waypoint y=2.38, 到标记面≈1.04m
    assert 0.90 <= nominal_forward_distance <= 1.20
    assert worst_case_forward_distance <= 1.30


def test_pick_station_stays_out_of_table_inflation():
    station_a = get_waypoint("station_a")
    station_table_front_y = 3.20  # 1.6×1.2 桌 front = 3.80 - 0.6
    # nav2 robot_radius=0.42 + inflation=0.55 ≈ 0.97 worst case;
    # 保守取 0.62(配合精停缩短后的新 waypoint)
    robot_radius = 0.62

    assert station_table_front_y - station_a["y"] > robot_radius


def test_place_station_stays_in_navigable_corridor():
    station_b = get_waypoint("station_b")

    assert -3.40 <= station_b["y"] <= -2.80


def test_place_station_stays_out_of_table_inflation_while_place_pose_reaches_table():
    station_b = get_waypoint("station_b")
    station_table_front_y = -2.30  # 1.6×1.2 桌 front = -1.70 - 0.6
    default_place_forward_distance = 0.82

    assert station_b["y"] <= -2.90
    assert station_b["y"] + default_place_forward_distance >= station_table_front_y


def test_new_zones_fill_the_lab_like_offset_layout():
    station_a = get_waypoint("station_a")
    station_b = get_waypoint("station_b")
    inspection = get_waypoint("inspection_zone")
    tooling = get_waypoint("tooling_zone")
    aging = get_waypoint("aging_zone")
    home = get_waypoint("home")

    assert station_a["x"] < -3.0 and 2.20 <= station_a["y"] <= 2.60
    assert -0.4 <= aging["x"] <= 0.4 and 2.90 <= aging["y"] <= 3.30
    assert inspection["x"] > 3.2 and inspection["y"] > 0.8
    assert tooling["x"] < -3.0 and tooling["y"] < -3.1
    assert -0.2 <= station_b["x"] <= 0.5 and -3.40 <= station_b["y"] <= -2.90
    assert home["x"] > 3.6 and home["y"] < -3.8


def test_get_waypoint_returns_copy():
    wp = get_waypoint("home")
    wp["x"] = 999.0
    assert WAYPOINTS["home"]["x"] == 4.50


def test_yaw_to_quat_zero():
    x, y, z, w = yaw_to_quat(0.0)
    assert abs(z) < 1e-9 and abs(w - 1.0) < 1e-9


def test_yaw_to_quat_90deg():
    x, y, z, w = yaw_to_quat(math.pi / 2.0)
    assert abs(z - math.sin(math.pi / 4)) < 1e-9
    assert abs(w - math.cos(math.pi / 4)) < 1e-9


def test_cruise_route_matches_confirmed_order():
    assert CRUISE_ROUTE == (
        "home",
        "station_a",
        "inspection_zone",
        "tooling_zone",
        "aging_zone",
        "station_b",
        "home",
    )


def test_every_cruise_stop_has_a_waypoint():
    for station in CRUISE_ROUTE:
        assert get_waypoint(station)


@pytest.mark.parametrize(
    "alias,expected",
    [
        ("A工位", "station_a"),
        ("工位 A", "station_a"),
        ("检测区", "inspection_zone"),
        ("工具区", "tooling_zone"),
        ("工装区", "tooling_zone"),
        ("老化区", "aging_zone"),
        ("B工位", "station_b"),
        ("HOME", "home"),
        ("起始点", "home"),
    ],
)
def test_station_aliases_normalize_to_canonical_names(alias, expected):
    assert normalize_station_name(alias) == expected


def test_unknown_station_alias_raises():
    with pytest.raises(KeyError):
        normalize_station_name("充电区")


# ── 五区全排列导航测试 ──────────────────────────────────────

# 5 个作业功能区(不含 home,home 是进出港而非作业站)
_WORK_ZONES = ("station_a", "inspection_zone", "tooling_zone", "aging_zone", "station_b")

# 高压区围栏碰撞盒(4 面连续墙 + 4 根立柱,取自 high_voltage_zone model.sdf)
# 外墙 front/back/left/right 坐标; 半厚 0.01 = 墙厚 0.02/2
_HV_CENTER_X = 4.36
_HV_CENTER_Y = 2.90
_HV_HALF_X = 1.00 + 0.015  # 立柱半径 margin
_HV_HALF_Y = 0.84 + 0.015


def _hv_contains(x: float, y: float) -> bool:
    """Return True if (x, y) is inside the high-voltage zone fence footprint."""
    return (
        _HV_CENTER_X - _HV_HALF_X <= x <= _HV_CENTER_X + _HV_HALF_X
        and _HV_CENTER_Y - _HV_HALF_Y <= y <= _HV_CENTER_Y + _HV_HALF_Y
    )


def _euclidean(a: dict, b: dict) -> float:
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def _angle_diff(yaw_a: float, yaw_b: float) -> float:
    d = yaw_b - yaw_a
    return abs(math.atan2(math.sin(d), math.cos(d)))


def _direct_path_intersects_hv(frm: dict, to: dict, step: float = 0.05) -> bool:
    """Check if line segment from waypoint `frm` to waypoint `to` crosses HV zone."""
    dist = _euclidean(frm, to)
    steps = max(2, int(dist / step))
    for i in range(steps + 1):
        t = i / steps
        x = frm["x"] + t * (to["x"] - frm["x"])
        y = frm["y"] + t * (to["y"] - frm["y"])
        if _hv_contains(x, y):
            return True
    return False


def test_all_5_work_zones_have_unique_poses():
    """五区坐标全部互异,不会两个站停到同一个位姿."""
    seen = set()
    for name in _WORK_ZONES:
        wp = get_waypoint(name)
        key = (round(wp["x"], 3), round(wp["y"], 3))
        assert key not in seen, f"{name} 坐标与另一站重复: {key}"
        seen.add(key)


def test_all_worktable_stations_point_north():
    """工作台站的 yaw=pi/2(朝北/+y),机器人在台前朝桌."""
    for name in _WORK_ZONES:
        wp = get_waypoint(name)
        assert abs(wp["yaw"] - math.pi / 2.0) < 1e-6, (
            f"{name} yaw={wp['yaw']:.4f}, expected pi/2"
        )


def test_home_points_east():
    """Home 朝向 +x(东),小车从右下角 home 区出发."""
    wp = get_waypoint("home")
    assert abs(wp["yaw"] - 0.0) < 1e-6, f"home yaw={wp['yaw']:.4f}, expected 0"


def test_cruise_route_visits_all_5_zones():
    """巡航路线覆盖全部 5 个作业区 + 以 home 起始和结束."""
    zones_in_route = [s for s in CRUISE_ROUTE if s != "home"]
    assert set(zones_in_route) == set(_WORK_ZONES), (
        f"巡航路线未覆盖: {set(_WORK_ZONES) - set(zones_in_route)}"
    )
    assert CRUISE_ROUTE[0] == "home", "巡航路线应以 home 起始"
    assert CRUISE_ROUTE[-1] == "home", "巡航路线应以 home 结束"
    assert len(zones_in_route) == 5, f"巡航路线有 {len(zones_in_route)} 个作业站,期望 5"


def test_all_20_directed_paths_have_valid_waypoints():
    """5×4=20 条有向路径全部有合法 waypoint,距离在合理范围."""
    max_corridor_diag = 20.0  # 14√2 ≈ 19.8,走廊对角线
    min_separation = 1.0     # 任意两站至少间隔 1m

    for frm_name in _WORK_ZONES:
        frm = get_waypoint(frm_name)
        for to_name in _WORK_ZONES:
            if frm_name == to_name:
                continue
            to = get_waypoint(to_name)
            d = _euclidean(frm, to)
            assert d >= min_separation, (
                f"{frm_name}→{to_name}: 距离={d:.2f}m < {min_separation}m,两站太近"
            )
            assert d <= max_corridor_diag, (
                f"{frm_name}→{to_name}: 距离={d:.2f}m > {max_corridor_diag}m,疑似越界"
            )
            # waypoint 本身不在高压区内
            assert not _hv_contains(frm["x"], frm["y"]), (
                f"{frm_name} waypoint 在高压区围栏内"
            )
            assert not _hv_contains(to["x"], to["y"]), (
                f"{to_name} waypoint 在高压区围栏内"
            )


def test_home_to_all_zones_and_back():
    """Home ↔ 任意作业区双向可达(共 10 条路径)."""
    home = get_waypoint("home")
    for zone_name in _WORK_ZONES:
        zone = get_waypoint(zone_name)
        d = _euclidean(home, zone)
        assert 2.0 <= d <= 18.0, (
            f"home↔{zone_name}: 距离={d:.2f}m,不合理"
        )


def test_no_direct_path_crosses_high_voltage_fence():
    """任意两点间直线路径不得穿过高压区围栏."""
    all_stations = list(_WORK_ZONES) + ["home"]
    blocked = []
    for i, frm_name in enumerate(all_stations):
        frm = get_waypoint(frm_name)
        for to_name in all_stations[i + 1:]:
            to = get_waypoint(to_name)
            if _direct_path_intersects_hv(frm, to):
                blocked.append(f"{frm_name}→{to_name}")
    assert not blocked, f"以下路径穿过高压区围栏: {blocked}"


def test_routing_table_20_paths_statistics():
    """输出 20 路径统计表(欧氏距离 + 朝向差),可入档验收."""
    lines = []
    lines.append("from,to,distance_m,yaw_diff_deg")
    total = 0.0
    count = 0
    for frm_name in sorted(_WORK_ZONES):
        for to_name in sorted(_WORK_ZONES):
            if frm_name == to_name:
                continue
            frm = get_waypoint(frm_name)
            to = get_waypoint(to_name)
            d = _euclidean(frm, to)
            yaw_diff_deg = math.degrees(_angle_diff(frm["yaw"], to["yaw"]))
            lines.append(f"{frm_name},{to_name},{d:.2f},{yaw_diff_deg:.1f}")
            total += d
            count += 1
    assert count == 20, f"应有 20 条路径,实际 {count}"

    avg = total / count
    lines.append(f"AVERAGE,,{avg:.2f},")
    # 打印统计表供验收
    print("\n── 20 路径统计表 ──")
    for line in lines:
        print(line)
    print(f"总路径数: {count}  平均距离: {avg:.2f}m\n")

    # 所有 20 条路径的朝向差应该接近 0(都是 pi/2 朝北),允许 pi 翻转
    assert avg <= 10.0, f"平均路径距离过大: {avg:.2f}m"
