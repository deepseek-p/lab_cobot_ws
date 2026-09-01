"""Contracts for fixed tooling-zone grasp validation targets."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from lab_cobot_bringup.grasp_target_config import (
    GRASP_TARGETS,
    TOOLING_TARGET_NAMES,
    validation_base_pose_for_target,
)


GAZEBO = Path(__file__).resolve().parents[2] / "lab_cobot_gazebo"


REQUIRED_FIELDS = {
    "station",
    "entity_name",
    "model_uri",
    "link_name",
    "world_pose",
    "validation_base_pose",
    "grasp_offset",
    "grasp_yaw",
    "pre_grasp_height",
    "tcp_clearance",
    "descend_distance",
    "lift_distance",
    "collision_size",
    "carried_collision_size",
    "attach_envelope",
}


def _world_includes():
    root = ET.parse(GAZEBO / "worlds" / "lab.world").getroot()
    result = {}
    for include in root.findall(".//include"):
        name = include.findtext("name")
        result[name] = {
            "uri": include.findtext("uri"),
            "pose": [float(value) for value in include.findtext("pose").split()],
        }
    return result


def test_grasp_targets_are_exactly_the_five_tooling_objects():
    assert set(GRASP_TARGETS) == set(TOOLING_TARGET_NAMES)
    assert len(GRASP_TARGETS) == 5


def test_grasp_targets_have_required_fields_and_station():
    for target, config in GRASP_TARGETS.items():
        assert REQUIRED_FIELDS <= set(config), target
        assert config["station"] == "tooling_zone"
        assert config["entity_name"] == target
        assert config["link_name"] == "link"
        assert len(config["collision_size"]) == 3
        assert len(config["carried_collision_size"]) == 3
        assert all(value > 0.0 for value in config["collision_size"])
        assert 0.08 <= config["pre_grasp_height"] <= 0.13
        if target != "high_voltage_probe_kit":
            assert 0.010 <= config["tcp_clearance"] <= 0.030
        assert 0.09 <= config["lift_distance"] <= 0.15


def test_grasp_target_entities_match_lab_world_includes():
    includes = _world_includes()
    for target, config in GRASP_TARGETS.items():
        include = includes[target]
        world_pose = config["world_pose"]
        assert include["uri"] == config["model_uri"]
        assert include["pose"][:2] == pytest.approx([
            world_pose["x"],
            world_pose["y"],
        ])
        if target == "high_voltage_probe_kit":
            assert include["pose"][2] == pytest.approx(0.75)
            assert world_pose["z"] == pytest.approx(0.7489984341574721)
        else:
            assert include["pose"][2] == pytest.approx(world_pose["z"])
        assert include["pose"][5] == pytest.approx(world_pose["yaw"])


def test_validation_base_pose_for_unknown_target_fails_loudly():
    with pytest.raises(ValueError, match="unknown validation_target"):
        validation_base_pose_for_target("not_a_tool")


def test_validation_base_poses_keep_table_clearance_estimates():
    # Tooling table spans y=-2.90..-1.70.  Chassis half length is 0.275m.
    # Validation-only estimates may approach from the south or north, but the
    # chassis front should remain outside the table footprint.
    table_south_edge_y = -2.90
    table_north_edge_y = -1.70
    chassis_half_length = 0.275
    for target, config in GRASP_TARGETS.items():
        pose = config["validation_base_pose"]
        if pose["yaw"] > 0.0:
            clearance = table_south_edge_y - (pose["y"] + chassis_half_length)
        else:
            clearance = (pose["y"] - chassis_half_length) - table_north_edge_y
        assert clearance >= 0.03, target


def test_high_voltage_validation_base_keeps_arm_closer_without_entering_table():
    config = GRASP_TARGETS["high_voltage_probe_kit"]
    pose = config["validation_base_pose"]
    target = config["world_pose"]
    table_south_edge_y = -2.90
    chassis_half_length = 0.275

    forward_distance = target["y"] - pose["y"]
    clearance = table_south_edge_y - (pose["y"] + chassis_half_length)

    assert pose["x"] == pytest.approx(target["x"])
    assert forward_distance == pytest.approx(0.61)
    assert clearance == pytest.approx(0.035)


def test_board_validation_base_puts_candidate_one_in_comfortable_workspace():
    config = GRASP_TARGETS["board_test_fixture"]
    pose = config["validation_base_pose"]
    table_south_edge_y = -2.90
    chassis_half_length = 0.275

    assert pose["x"] == pytest.approx(-4.596977732335393)
    assert pose["y"] == pytest.approx(-3.212689253512364)
    assert pose["yaw"] == pytest.approx(math.pi / 2.0)
    assert table_south_edge_y - (pose["y"] + chassis_half_length) == pytest.approx(
        0.037689253512364
    )


def test_high_voltage_probe_kit_collision_z_uses_gazebo_measured_pose():
    config = GRASP_TARGETS["high_voltage_probe_kit"]
    collision_center_z = (
        config["world_pose"]["z"] + config["collision_local_pose"]["z"]
    )
    collision_half_height = 0.5 * config["collision_size"][2]

    assert collision_center_z == pytest.approx(0.7889984341574722)
    assert collision_center_z + collision_half_height == pytest.approx(
        0.8289984341574722
    )
    assert collision_center_z - collision_half_height == pytest.approx(
        0.7489984341574721
    )


def test_high_voltage_probe_kit_grasps_red_handle_center_xy_only():
    config = GRASP_TARGETS["high_voltage_probe_kit"]
    handle = config["handle_grasp_local_point"]
    offset = config["grasp_offset"]

    # Red painted handle mesh bounds after model.sdf scale 2:
    # x=0.004260658..0.15999996, y=-0.02671526..0.02671526.
    assert handle["x"] == pytest.approx(0.082130309)
    assert handle["y"] == pytest.approx(0.0)
    assert handle["z"] == pytest.approx(0.02671527)
    assert config["handle_long_axis_yaw"] == pytest.approx(0.0)
    assert offset["x"] == pytest.approx(handle["x"])
    assert offset["y"] == pytest.approx(handle["y"])
    assert offset["z"] == pytest.approx(0.040)


def test_board_test_fixture_grasps_bottom_wide_base_block():
    config = GRASP_TARGETS["board_test_fixture"]
    center = config["base_block_local_center"]
    size = config["base_block_size"]
    offset = config["grasp_offset"]
    candidate_1 = config["grasp_candidate_1"]

    assert center["x"] == pytest.approx(0.082585)
    assert center["y"] == pytest.approx(-0.0729625)
    assert center["z"] == pytest.approx(-0.0000085)
    assert size == pytest.approx((0.128356, 0.054419, 0.055183))
    assert config["base_block_local_rpy"] == pytest.approx((0.0, 0.0, 0.0))
    assert config["base_block_long_axis_yaw"] == pytest.approx(0.0)
    assert config["grasp_yaw"] == pytest.approx(math.pi / 2.0)
    assert offset["x"] == pytest.approx(center["x"])
    assert offset["y"] == pytest.approx(center["y"])
    assert offset["z"] == pytest.approx(0.006)
    assert config["grasp_z_adjust"] == pytest.approx(0.005)
    assert candidate_1["label"] == "bottom_light_beige_coarse_rectangular_block"
    assert candidate_1["local_center"] == center
    assert candidate_1["size"] == size
    assert candidate_1["grasp_yaw"] == pytest.approx(config["grasp_yaw"])


def test_board_test_fixture_records_yellow_cylinder_backup_candidate():
    config = GRASP_TARGETS["board_test_fixture"]
    candidate_2 = config["grasp_candidate_2"]
    center = candidate_2["local_center"]
    size = candidate_2["size"]

    assert candidate_2["label"] == "right_yellow_thick_cylindrical_handle"
    assert center["x"] == pytest.approx(0.09249197)
    assert center["y"] == pytest.approx(0.07339262)
    assert center["z"] == pytest.approx(-0.00000844)
    assert size == pytest.approx((0.11501598, 0.05355908, 0.05355885))
    assert candidate_2["long_axis_yaw"] == pytest.approx(0.0)
    assert candidate_2["grasp_yaw"] == pytest.approx(0.0)


def test_remaining_tool_targets_use_geometry_based_grasp_regions():
    expected = {
        "tooling_fixture_box": {
            "label": "central_adjustable_wrench_handle_shank",
            # The 0.06 m local-Y offset is intentional and corresponds to the
            # validated Fixture grasp region used by the frozen baseline.
            "point": (0.0, 0.06000000, 0.00007236),
            "size": (0.10411212, 0.31999992, 0.01614453),
            "yaw": math.pi / 2.0,
        },
        "material_spare_igbt": {
            "label": "main_ruler_mid_body",
            "point": (0.01580000, -0.05290000, 0.00017000),
            "size": (0.02378844, 0.06000000, 0.00475767),
            "yaw": math.pi / 2.0,
        },
    }
    for target, values in expected.items():
        config = GRASP_TARGETS[target]
        point = config["grasp_point_local"]
        assert config["grasp_region_label"] == values["label"]
        assert (point["x"], point["y"], point["z"]) == pytest.approx(
            values["point"]
        )
        assert config["grasp_offset"] == point
        assert config["grasp_region_size"] == pytest.approx(values["size"])
        if target == "material_spare_igbt":
            assert config["grasp_long_axis_yaw"] == pytest.approx(math.pi / 2.0)
            assert config["gripper_closing_axis_local"] == "model_local_x"
            assert config["expected_grasp_width_mm"] == pytest.approx(23.79)
            assert config["pre_grasp_position"] == pytest.approx(0.03010)
            assert config["tactile_start_position"] == pytest.approx(0.03010)
            assert config["tactile_max_position"] == pytest.approx(0.03480)
            assert config["pre_grasp_height"] == pytest.approx(0.080)
            assert config["descend_distance"] == pytest.approx(0.080)
        else:
            assert config["grasp_long_axis_yaw"] == pytest.approx(values["yaw"])
        assert config["grasp_yaw"] == pytest.approx(values["yaw"])

    hand_tools = GRASP_TARGETS["tooling_hand_tools"]
    hand_tools_point = hand_tools["grasp_point_local"]
    assert hand_tools["grasp_region_label"] == "closed_pliers_white_handle_body"
    assert len(hand_tools_point) == 3
    assert all(math.isfinite(float(hand_tools_point[axis])) for axis in ("x", "y", "z"))
    assert hand_tools["grasp_offset"] == hand_tools_point
    assert hand_tools["grasp_region_size"] == pytest.approx(
        (0.10398028, 0.20561742, 0.02166162)
    )
    assert hand_tools["grasp_long_axis_yaw"] == pytest.approx(math.pi / 2.0)
    assert hand_tools["grasp_yaw"] == pytest.approx(math.pi / 2.0)
    assert abs(float(hand_tools_point["x"])) <= 0.06
    assert abs(float(hand_tools_point["y"])) <= 0.12
    assert abs(float(hand_tools_point["z"])) <= 0.03
    assert hand_tools["entity_name"] == "tooling_hand_tools"
    assert hand_tools["model_uri"] == "model://tooling_hand_tools"
    assert hand_tools["link_name"] == "link"
