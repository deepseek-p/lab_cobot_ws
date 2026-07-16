"""Environment model contracts for the five-zone lab scene."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import pytest


GAZEBO = Path(__file__).resolve().parents[1]


def _model_root(name):
    return ET.parse(GAZEBO / "models" / name / "model.sdf").getroot()


def _world_root():
    return ET.parse(GAZEBO / "worlds" / "lab.world").getroot()


def _diffuse(model_name):
    return [
        float(value)
        for value in _model_root(model_name)
        .findtext(".//visual/material/diffuse")
        .split()
    ]


def _include_pose(entity_name):
    for include in _world_root().findall(".//include"):
        if include.findtext("name") == entity_name:
            return [float(value) for value in include.findtext("pose").split()]
    raise AssertionError(f"missing include for {entity_name}")


def test_plain_igbt_is_static_flat_square():
    model = _model_root("igbt_module_plain").find("model")

    assert model.findtext("static") == "true"
    assert model.findtext(".//collision/geometry/box/size") == "0.09 0.09 0.06"
    assert _diffuse("igbt_module_plain") == pytest.approx([0.28, 0.31, 0.34, 1.0])


def test_fixture_box_is_static_unmarked_tooling_obstacle():
    model = _model_root("fixture_box_plain").find("model")

    assert model.findtext("static") == "true"
    assert model.findtext(".//collision/geometry/box/size") == "0.16 0.12 0.10"
    assert _diffuse("fixture_box_plain") == pytest.approx([0.56, 0.46, 0.15, 1.0])


def test_aging_rack_has_three_visual_slots_and_status_panel():
    root = _model_root("aging_rack")
    assert root.find(".//visual[@name='slot_left']") is not None
    assert root.find(".//visual[@name='slot_mid']") is not None
    assert root.find(".//visual[@name='slot_right']") is not None
    assert root.find(".//visual[@name='status_green']") is not None
    assert root.find(".//visual[@name='status_yellow']") is not None
    assert root.find(".//visual[@name='status_red']") is not None


def test_new_tabletop_props_exist_for_tooling_board_test_and_high_voltage_identity():
    assert _model_root("thermal_grease_can").find(".//visual[@name='cap']") is not None
    assert _model_root("tooling_hand_tools").find(".//visual[@name='screwdriver_handle_red']") is not None
    assert _model_root("pcb_test_fixture").find(".//visual[@name='indicator_led_green']") is not None
    assert _model_root("safety_probe_kit").find(".//visual[@name='probe_red_handle']") is not None


def test_world_uses_four_worktables_plus_separate_high_voltage_and_home_zones():
    world = _world_root()
    names = {model.get("name") for model in world.findall(".//model")}
    assert {
        "station_a_table",
        "tooling_zone_table",
        "aging_zone_table",
        "station_b_table",
        "home_zone_pad",
    } <= names


def test_world_places_new_objects_in_the_expected_five_zone_layout():
    aruco = _include_pose("aruco_sample")
    spare_igbt = _include_pose("material_spare_igbt")
    grease = _include_pose("material_grease_can")
    fixture = _include_pose("tooling_fixture_box")
    hand_tools = _include_pose("tooling_hand_tools")
    rack = _include_pose("aging_rack")
    board_fixture = _include_pose("board_test_fixture")
    probe_kit = _include_pose("high_voltage_probe_kit")
    high_voltage = _include_pose("high_voltage_zone")

    assert aruco[:3] == pytest.approx([-2.08, 1.73, 0.78])
    assert spare_igbt[:3] == pytest.approx([-2.31, 1.96, 0.78])
    assert grease[:3] == pytest.approx([-1.95, 1.98, 0.805])
    assert fixture[:3] == pytest.approx([-1.94, -1.02, 0.80])
    assert hand_tools[:3] == pytest.approx([-2.18, -0.98, 0.80])
    assert rack[:3] == pytest.approx([0.10, 2.13, 0.80])
    assert board_fixture[:3] == pytest.approx([0.01, -0.72, 0.781])
    assert probe_kit[:3] == pytest.approx([2.02, 1.22, 0.0])
    assert high_voltage[:3] == pytest.approx([2.18, 1.45, 0.0])

    sample_half_extent = 0.045
    assert 1.60 < aruco[1] - sample_half_extent < 1.90
    assert 1.60 < spare_igbt[1] + sample_half_extent < 2.20
    assert hand_tools[0] < fixture[0] < -1.75
    assert rack[1] > aruco[1]
    assert probe_kit[0] < high_voltage[0]
    assert math.dist(aruco[:2], rack[:2]) > 1.8


def test_station_b_remains_clear_for_existing_place_task():
    world = _world_root()
    blockers = []
    for include in world.findall(".//include"):
        name = include.findtext("name")
        if name in {
            "aruco_sample",
            "material_spare_igbt",
            "material_grease_can",
            "tooling_fixture_box",
            "tooling_hand_tools",
            "aging_rack",
            "high_voltage_probe_kit",
            "high_voltage_zone",
        }:
            pose = [float(value) for value in include.findtext("pose").split()]
            if -0.25 <= pose[0] <= 0.55 and -1.15 <= pose[1] <= -0.55:
                blockers.append((name, pose))
    assert not blockers
