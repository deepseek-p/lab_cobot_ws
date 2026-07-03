"""waypoints 单元测试(纯逻辑,headless pytest 可跑)。"""
import math

import pytest

from lab_cobot_navigation.waypoints import (
    get_waypoint,
    list_stations,
    yaw_to_quat,
    WAYPOINTS,
)


def test_known_stations_present():
    assert set(list_stations()) >= {"station_a", "station_b", "home"}


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


def test_pick_station_is_within_manipulator_reach():
    station_a = get_waypoint("station_a")
    sample_y = 1.5
    nav_xy_goal_tolerance = 0.12

    nominal_forward_distance = sample_y - station_a["y"]
    worst_case_forward_distance = nominal_forward_distance + nav_xy_goal_tolerance

    assert 0.60 <= nominal_forward_distance <= 0.70
    assert worst_case_forward_distance <= 0.78


def test_place_station_stays_in_navigable_corridor():
    station_b = get_waypoint("station_b")

    assert station_b["y"] <= 0.70


def test_place_station_stays_out_of_table_inflation_while_place_pose_reaches_table():
    station_b = get_waypoint("station_b")
    station_table_front_y = 1.20
    default_place_forward_distance = 0.82

    assert station_b["y"] <= 0.50
    assert station_b["y"] + default_place_forward_distance >= station_table_front_y


def test_get_waypoint_returns_copy():
    wp = get_waypoint("home")
    wp["x"] = 999.0
    assert WAYPOINTS["home"]["x"] == 0.0  # 原表不被修改


def test_yaw_to_quat_zero():
    x, y, z, w = yaw_to_quat(0.0)
    assert abs(z) < 1e-9 and abs(w - 1.0) < 1e-9


def test_yaw_to_quat_90deg():
    x, y, z, w = yaw_to_quat(math.pi / 2.0)
    assert abs(z - math.sin(math.pi / 4)) < 1e-9
    assert abs(w - math.cos(math.pi / 4)) < 1e-9
