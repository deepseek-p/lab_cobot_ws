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


def _include_pose(entity_name):
    for include in _world_root().findall(".//include"):
        if include.findtext("name") == entity_name:
            return [float(value) for value in include.findtext("pose").split()]
    raise AssertionError(f"missing include for {entity_name}")


def _include_uri(entity_name):
    for include in _world_root().findall(".//include"):
        if include.findtext("name") == entity_name:
            return include.findtext("uri")
    raise AssertionError(f"missing include for {entity_name}")


def test_plain_igbt_asset_has_dynamic_collision_contract():
    model = _model_root("igbt_module_plain").find("model")

    assert model.findtext("static") is None
    assert model.findtext(".//link/inertial/mass") is not None
    assert model.findtext(".//collision/geometry/box/size") == "0.140 0.380 0.120"
    assert model.find(".//collision/surface/friction") is not None
    assert model.findtext(".//visual/geometry/mesh/uri") == (
        "model://igbt_module_plain/meshes/digital_caliper.dae"
    )


def test_fixture_box_asset_has_dynamic_collision_contract():
    model = _model_root("fixture_box_plain").find("model")

    assert model.findtext("static") is None
    assert model.findtext(".//link/inertial/mass") is not None
    assert model.findtext(".//collision/geometry/box/size") == "0.120 0.340 0.200"
    assert model.find(".//collision/surface/friction") is not None
    assert model.findtext(".//visual/geometry/mesh/uri") == (
        "model://fixture_box_plain/meshes/adjustable_wrench.dae"
    )


def test_aging_rack_has_three_visual_slots_and_status_panel():
    root = _model_root("aging_rack")
    assert root.find(".//visual[@name='slot_left']") is not None
    assert root.find(".//visual[@name='slot_mid']") is not None
    assert root.find(".//visual[@name='slot_right']") is not None
    assert root.find(".//visual[@name='status_green']") is not None
    assert root.find(".//visual[@name='status_yellow']") is not None
    assert root.find(".//visual[@name='status_red']") is not None


def test_new_tabletop_props_use_expected_imported_meshes():
    expected_meshes = {
        "thermal_grease_can": "meter_closed.dae",
        "tooling_hand_tools": "pliers_closed.dae",
        "pcb_test_fixture": "drill.dae",
        "safety_probe_kit": "screwdriver.dae",
    }
    for model_name, mesh_name in expected_meshes.items():
        assert _model_root(model_name).findtext(
            ".//visual/geometry/mesh/uri"
        ) == f"model://{model_name}/meshes/{mesh_name}"


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

    assert _include_uri("aruco_sample") == "model://aruco_sample"
    assert aruco[:3] == pytest.approx([-4.16, 3.46, 0.785])
    assert spare_igbt[:3] == pytest.approx([-4.62, 3.92, 0.78])
    assert grease[:3] == pytest.approx([-3.90, 3.96, 0.75])
    assert fixture[:3] == pytest.approx([-3.88, -2.04, 0.80])
    assert hand_tools[:3] == pytest.approx([-4.36, -1.96, 0.75])
    assert rack[:3] == pytest.approx([0.20, 4.26, 0.80])
    assert board_fixture[:3] == pytest.approx([0.02, -1.44, 0.75])
    assert probe_kit[:3] == pytest.approx([4.04, 2.44, 0.0])
    assert high_voltage[:3] == pytest.approx([4.36, 2.90, 0.0])

    sample_half_extent = 0.035
    assert 3.20 < aruco[1] - sample_half_extent < 3.80
    assert 3.20 < spare_igbt[1] + sample_half_extent < 4.40
    assert hand_tools[0] < fixture[0] < -3.50
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
