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
    sample_y = 1.72
    nav_xy_goal_tolerance = 0.12

    nominal_forward_distance = sample_y - station_a["y"]
    worst_case_forward_distance = nominal_forward_distance + nav_xy_goal_tolerance

    assert 0.85 <= nominal_forward_distance <= 1.00
    assert worst_case_forward_distance <= 1.12


def test_pick_station_stays_out_of_table_inflation():
    station_a = get_waypoint("station_a")
    station_table_front_y = 1.60
    robot_radius = 0.72

    assert station_table_front_y - station_a["y"] > robot_radius


def test_place_station_stays_in_navigable_corridor():
    station_b = get_waypoint("station_b")

    assert -2.05 <= station_b["y"] <= -1.88


def test_place_station_stays_out_of_table_inflation_while_place_pose_reaches_table():
    station_b = get_waypoint("station_b")
    station_table_front_y = -1.15
    default_place_forward_distance = 0.82

    assert station_b["y"] <= -1.88
    assert station_b["y"] + default_place_forward_distance >= station_table_front_y


def test_new_zones_fill_the_lab_like_offset_layout():
    station_a = get_waypoint("station_a")
    station_b = get_waypoint("station_b")
    inspection = get_waypoint("inspection_zone")
    tooling = get_waypoint("tooling_zone")
    aging = get_waypoint("aging_zone")
    home = get_waypoint("home")

    assert station_a["x"] < -1.5 and 0.70 <= station_a["y"] <= 0.85
    assert -0.2 <= aging["x"] <= 0.2 and aging["y"] > 1.2
    assert inspection["x"] > 1.6 and inspection["y"] > 0.4
    assert tooling["x"] < -1.5 and tooling["y"] < -1.5
    assert -0.1 <= station_b["x"] <= 0.3 and station_b["y"] < -1.85
    assert home["x"] > 1.8 and home["y"] < -1.9


def test_get_waypoint_returns_copy():
    wp = get_waypoint("home")
    wp["x"] = 999.0
    assert WAYPOINTS["home"]["x"] == 2.25


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
