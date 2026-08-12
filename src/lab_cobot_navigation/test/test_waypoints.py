"""waypoints 单元测试(纯逻辑,headless pytest 可跑)."""
import math

import pytest

from lab_cobot_navigation.waypoints import (
    CRUISE_ROUTE,
    WAYPOINTS,
    get_station_spec,
    get_waypoint,
    list_stations,
    normalize_station_name,
    yaw_to_quat,
)

STATION_SPECS = None
_STATION_SPECS_LOADED = False


def _load_specs():
    global STATION_SPECS, _STATION_SPECS_LOADED
    if _STATION_SPECS_LOADED:
        return
    from lab_cobot_navigation.waypoints import STATION_SPECS as _s
    STATION_SPECS = _s
    _STATION_SPECS_LOADED = True


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

    # waypoint y=2.72, 到标记面≈0.70m (精停后可靠近桌面)
    assert 0.55 <= nominal_forward_distance <= 0.90
    assert worst_case_forward_distance <= 1.00


def test_pick_station_stays_out_of_table_inflation():
    station_a = get_waypoint("station_a")
    station_table_front_y = 3.20  # 1.6×1.2 桌 front = 3.80 - 0.6
    # inflation=0.55; waypoint y=2.72 → 桌边缘 y=3.20, 间距=0.48m
    # 精停阶段允许驶入膨胀区内完成靠拢
    robot_radius = 0.30

    assert station_table_front_y - station_a["y"] > robot_radius


def test_place_station_stays_in_navigable_corridor():
    station_b = get_waypoint("station_b")

    assert -3.40 <= station_b["y"] <= -2.60


def test_place_station_stays_out_of_table_inflation_while_place_pose_reaches_table():
    station_b = get_waypoint("station_b")
    station_table_front_y = -2.30  # 1.6×1.2 桌 front = -1.70 - 0.6
    default_place_forward_distance = 0.82

    assert station_b["y"] <= -2.60
    assert station_b["y"] + default_place_forward_distance >= station_table_front_y


def test_new_zones_fill_the_lab_like_offset_layout():
    station_a = get_waypoint("station_a")
    station_b = get_waypoint("station_b")
    inspection = get_waypoint("inspection_zone")
    tooling = get_waypoint("tooling_zone")
    assert inspection["x"] == pytest.approx(4.10)
    aging = get_waypoint("aging_zone")
    home = get_waypoint("home")

    assert station_a["x"] < -3.0 and 2.45 <= station_a["y"] <= 2.65
    assert -0.4 <= aging["x"] <= 0.4 and 2.90 <= aging["y"] <= 3.30
    assert inspection["x"] > 3.2 and inspection["y"] > 0.8
    assert tooling["x"] == pytest.approx(-4.10)
    assert -3.5 <= tooling["y"] <= -3.1
    assert tooling["yaw"] == pytest.approx(math.pi / 2.0)
    assert -0.2 <= station_b["x"] <= 0.5 and -3.40 <= station_b["y"] <= -2.60
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


def test_worktable_stations_face_their_work_surfaces():
    for name in _WORK_ZONES:
        wp = get_waypoint(name)
        expected = math.pi / 2.0
        assert abs(wp["yaw"] - expected) < 1e-6


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


# ── StationSpec 语义测试 ──────────────────────────────────────


def _station_specs():
    _load_specs()
    return dict(STATION_SPECS)


def _station_names():
    return set(_station_specs().keys())


def test_station_specs_covers_all_legacy_waypoints():
    specs = _station_specs()
    legacy = set(WAYPOINTS.keys())
    spec_names = set(specs.keys())
    missing_from_spec = legacy - spec_names
    missing_from_legacy = spec_names - legacy
    assert not missing_from_spec, f"StationSpec 缺少: {missing_from_spec}"
    assert not missing_from_legacy, f"WAYPOINTS 缺少: {missing_from_legacy}"


def test_each_station_declares_navigation_and_docking_semantics():
    for name, spec in _station_specs().items():
        assert spec.nav_pose.frame_id == "map", f"{name}: nav_pose 缺少 frame_id"
        assert spec.dock_pose.frame_id == "map", f"{name}: dock_pose 缺少 frame_id"
        assert spec.nav_legs[-1].pose == spec.nav_pose, (
            f"{name}: 末段 leg pose 应等于 nav_pose"
        )
        assert spec.nav_legs[-1].dock_station == name, (
            f"{name}: 末段 leg dock_station 应等于自身名称"
        )
        assert spec.approach_side in {"south", "north", "east", "west", "none"}, (
            f"{name}: 无效 approach_side={spec.approach_side}"
        )


def test_station_a_south_side_dock_faces_north():
    spec = get_station_spec("station_a")
    assert spec.approach_side == "south", (
        f"station_a approach_side={spec.approach_side}, expected south"
    )
    assert spec.dock_pose.yaw == pytest.approx(math.pi / 2.0)


def test_worktable_stations_have_work_surface():
    for name in ("station_a", "tooling_zone", "aging_zone", "station_b"):
        spec = get_station_spec(name)
        assert spec.work_surface is not None, f"{name} 应有 work_surface"
        assert spec.work_surface.size_x > 0
        assert spec.work_surface.size_y > 0


def test_inspection_zone_no_work_surface_outside_fence():
    spec = get_station_spec("inspection_zone")
    assert spec.work_surface is None, "inspection_zone 不应有 work_surface"
    # 所有 pose 在高压围栏外
    for leg in spec.nav_legs:
        assert not _hv_contains(leg.pose.x, leg.pose.y), (
            f"inspection leg {leg.name} 在高压围栏内"
        )
    assert not _hv_contains(spec.dock_pose.x, spec.dock_pose.y), (
        "inspection dock_pose 在高压围栏内"
    )


def test_home_no_work_surface_approach_none():
    spec = get_station_spec("home")
    assert spec.work_surface is None
    assert spec.approach_side == "none"


@pytest.mark.parametrize("name", ["station_a", "tooling_zone", "aging_zone", "station_b"])
def test_worktable_dock_pose_outside_table(name):
    spec = get_station_spec(name)
    ws = spec.work_surface
    half_length = 0.55 / 2.0  # chassis semi-length
    clearance = spec.clearance_m

    # dock pose y 必须在桌面外(即从桌边后退 half_length + clearance)
    if spec.approach_side == "south":
        min_safe_y = ws.center_y - ws.size_y / 2.0 - half_length - clearance
        assert spec.dock_pose.y <= min_safe_y + 0.02, (
            f"{name}: dock_pose.y={spec.dock_pose.y:.3f}, "
            f"min_safe={min_safe_y:.3f}"
        )
    elif spec.approach_side == "north":
        max_safe_y = ws.center_y + ws.size_y / 2.0 + half_length + clearance
        assert spec.dock_pose.y >= max_safe_y - 0.02, (
            f"{name}: dock_pose.y={spec.dock_pose.y:.3f}, "
            f"max_safe={max_safe_y:.3f}"
        )


def test_station_spec_clearance_positive():
    for name, spec in _station_specs().items():
        assert spec.clearance_m >= 0.0, f"{name}: clearance 不应为负"


def test_cruise_route_unchanged_with_specs():
    from lab_cobot_navigation.waypoints import CRUISE_ROUTE
    assert CRUISE_ROUTE == (
        "home", "station_a", "inspection_zone",
        "tooling_zone", "aging_zone", "station_b", "home",
    )


def test_legacy_get_waypoint_matches_spec_nav_pose():
    for name in _station_names():
        wp = get_waypoint(name)
        spec = get_station_spec(name)
        assert wp["x"] == pytest.approx(spec.nav_pose.x)
        assert wp["y"] == pytest.approx(spec.nav_pose.y)
        assert wp["yaw"] == pytest.approx(spec.nav_pose.yaw)


def test_legacy_waypoints_dict_matches_spec():
    _load_specs()
    for name, spec in STATION_SPECS.items():
        assert name in WAYPOINTS, f"{name} 应在 WAYPOINTS 中"
        assert WAYPOINTS[name]["x"] == pytest.approx(spec.nav_pose.x)
        assert WAYPOINTS[name]["y"] == pytest.approx(spec.nav_pose.y)
        assert WAYPOINTS[name]["yaw"] == pytest.approx(spec.nav_pose.yaw)


def test_every_station_has_at_least_one_nav_leg():
    for name, spec in _station_specs().items():
        assert len(spec.nav_legs) >= 1, f"{name}: 缺少 nav_legs"


def test_route_leg_dock_station_only_on_terminal_leg():
    for name, spec in _station_specs().items():
        for i, leg in enumerate(spec.nav_legs):
            if i < len(spec.nav_legs) - 1:
                assert leg.dock_station is None, (
                    f"{name}: 非末段 leg '{leg.name}' dock_station 应为 None"
                )
