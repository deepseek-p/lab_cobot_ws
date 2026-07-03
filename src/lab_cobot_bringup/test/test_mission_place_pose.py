"""Mission placement geometry regression tests."""

import math

import pytest

from lab_cobot_bringup.mission_node import DEFAULT_PLACE_POSE
from lab_cobot_navigation.waypoints import get_waypoint


def _base_to_map(base_xy, station):
    x, y = base_xy
    yaw = station["yaw"]
    return (
        station["x"] + x * math.cos(yaw) - y * math.sin(yaw),
        station["y"] + x * math.sin(yaw) + y * math.cos(yaw),
    )


def test_default_place_pose_targets_reachable_station_b_table_front():
    station_b = get_waypoint("station_b")
    base_link_world_z = 0.155
    sample_center_world_z = 0.785
    table_min_x = -2.4
    table_max_x = -1.6
    table_front_y = 1.2
    table_mid_y = 1.5
    max_reachable_forward = 0.82
    minimum_nominal_front_margin = 0.07

    map_x, map_y = _base_to_map(DEFAULT_PLACE_POSE[:2], station_b)

    assert DEFAULT_PLACE_POSE[0] <= max_reachable_forward
    assert table_min_x <= map_x <= table_max_x
    assert table_front_y <= map_y <= table_mid_y
    assert map_y - table_front_y >= minimum_nominal_front_margin
    assert DEFAULT_PLACE_POSE[2] + base_link_world_z == pytest.approx(
        sample_center_world_z,
        abs=0.02,
    )
