"""Mission placement geometry regression tests."""

import math

import pytest

from lab_cobot_bringup.mission_node import (
    DEFAULT_PLACE_POSE,
    PLACE_BASE_TARGET_POSE,
    STATION_B_SAFE_DROP_BACK_Y,
    STATION_B_SAFE_DROP_FRONT_Y,
    STATION_B_SAFE_DROP_MAX_X,
    STATION_B_SAFE_DROP_MIN_X,
    _base_target_to_map,
)
from lab_cobot_manipulation.pick_place_node import (
    TACTILE_PLACE_RELEASE_CLEARANCE,
    TACTILE_PLACE_TCP_Z_COMPENSATION,
)


def _base_to_map(base_xy, station):
    x, y = base_xy
    yaw = station["yaw"]
    return (
        station["x"] + x * math.cos(yaw) - y * math.sin(yaw),
        station["y"] + x * math.sin(yaw) + y * math.cos(yaw),
    )


def test_default_place_pose_targets_reachable_station_b_table_front():
    station_b = {
        "x": PLACE_BASE_TARGET_POSE[0],
        "y": PLACE_BASE_TARGET_POSE[1],
        "yaw": PLACE_BASE_TARGET_POSE[2],
    }
    base_link_world_z = 0.155
    # 历史标定为 -0.065m(非早期误估的 -0.027m);2026-07-12 DG-2 深抓探针
    # 两轮实测为 -0.013055m/-0.017907m,这里取实测中点 -0.015481m。
    held_sample_center_from_tcp_z = -0.015481
    vision_z_error_band = 0.015
    table_top_world_z = 0.75
    sample_half_height = 0.035
    table_min_x = -0.25
    table_max_x = 0.55
    table_mid_y = -0.85
    max_reachable_forward = 0.82
    minimum_nominal_front_margin = 0.05

    map_x, map_y = _base_to_map(DEFAULT_PLACE_POSE[:2], station_b)

    assert DEFAULT_PLACE_POSE[0] == pytest.approx(0.78)
    assert map_y == pytest.approx(-0.945)
    carried_offset_y = 0.0455
    assert map_y - carried_offset_y - 0.035 >= -1.15 + 0.03
    sample_half_extent = 0.035
    assert map_y - sample_half_extent > -1.15
    assert map_y + sample_half_extent < -0.55
    assert DEFAULT_PLACE_POSE[0] <= max_reachable_forward
    assert table_min_x <= map_x <= table_max_x
    assert STATION_B_SAFE_DROP_FRONT_Y <= map_y <= table_mid_y
    assert map_y - STATION_B_SAFE_DROP_FRONT_Y >= minimum_nominal_front_margin
    assert DEFAULT_PLACE_POSE[2] == pytest.approx(0.725)
    # 悬空释放几何:释放瞬间(descend 到 target+clearance)物块底面必须
    # 高出台面一个视觉误差带,确保带焊物块永不压入台面(约束爆炸根因);
    # 同时落差不超过 8cm,避免 0.05kg 样件弹跳出安全落区。
    release_bottom_world_z = (
        DEFAULT_PLACE_POSE[2]
        + TACTILE_PLACE_RELEASE_CLEARANCE
        - TACTILE_PLACE_TCP_Z_COMPENSATION
        + base_link_world_z
        + held_sample_center_from_tcp_z
        - sample_half_height
    )
    drop_height = release_bottom_world_z - table_top_world_z
    assert drop_height >= vision_z_error_band
    assert drop_height <= 0.08


def test_place_base_target_projects_drop_point_into_safe_table_band():
    map_x, map_y = _base_target_to_map(
        PLACE_BASE_TARGET_POSE,
        DEFAULT_PLACE_POSE[:2],
    )

    assert STATION_B_SAFE_DROP_MIN_X <= map_x <= STATION_B_SAFE_DROP_MAX_X
    assert STATION_B_SAFE_DROP_FRONT_Y <= map_y <= STATION_B_SAFE_DROP_BACK_Y
    sample_half_extent = 0.035
    required_edge_margin = 0.03
    assert map_x - sample_half_extent >= -0.25 + required_edge_margin
    assert map_x + sample_half_extent <= 0.55 - required_edge_margin
    assert map_y - sample_half_extent >= -1.15 + required_edge_margin
    assert map_y + sample_half_extent <= -0.55 - required_edge_margin
