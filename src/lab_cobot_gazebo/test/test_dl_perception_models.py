"""DL perception object model contracts."""
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


def _include_pose(model_name):
    for include in _world_root().findall(".//include"):
        if include.findtext("uri") == f"model://{model_name}":
            return [float(value) for value in include.findtext("pose").split()]
    raise AssertionError(f"missing include for {model_name}")


def test_reagent_bottle_is_static_blue_cylinder():
    model = _model_root("reagent_bottle").find("model")

    assert model.findtext("static") == "true"
    cylinder = model.find(".//collision/geometry/cylinder")
    assert float(cylinder.findtext("radius")) == pytest.approx(0.035)
    assert float(cylinder.findtext("length")) == pytest.approx(0.16)
    assert _diffuse("reagent_bottle") == pytest.approx([0.1, 0.2, 0.8, 1.0])


def test_toolbox_yellow_is_static_yellow_box():
    model = _model_root("toolbox_yellow").find("model")

    assert model.findtext("static") == "true"
    assert model.findtext(".//collision/geometry/box/size") == "0.12 0.12 0.10"
    assert _diffuse("toolbox_yellow") == pytest.approx([0.9, 0.8, 0.1, 1.0])


def test_world_places_new_objects_by_goal_s_constraints():
    aruco = _include_pose("aruco_sample")
    reagent = _include_pose("reagent_bottle")
    toolbox = _include_pose("toolbox_yellow")

    assert aruco[:3] == pytest.approx([2.0, 1.32, 0.785])
    sample_half_extent = 0.035
    table_front_y = 1.20
    table_back_y = 1.80
    assert aruco[1] - sample_half_extent > table_front_y
    assert aruco[1] + sample_half_extent < table_back_y
    assert reagent[:3] == pytest.approx([2.28, 1.62, 0.83])
    assert toolbox[:3] == pytest.approx([1.72, 1.62, 0.80])
    assert abs(reagent[0] - 2.0) >= 0.25
    assert abs(toolbox[0] - 2.0) >= 0.25
    assert reagent[1] >= 1.5
    assert toolbox[1] >= 1.5
    assert math.dist(aruco[:2], reagent[:2]) >= 0.2
    assert math.dist(aruco[:2], toolbox[:2]) >= 0.2
