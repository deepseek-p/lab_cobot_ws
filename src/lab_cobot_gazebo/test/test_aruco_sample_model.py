"""ArUco sample model contracts."""
from pathlib import Path
import xml.etree.ElementTree as ET


def _model_root():
    model = (
        Path(__file__).resolve().parents[1]
        / "models"
        / "aruco_sample"
        / "model.sdf"
    )
    return ET.parse(model).getroot()


def _visuals():
    return _model_root().findall(".//link[@name='link']/visual")


def test_aruco_texture_is_on_single_robot_facing_patch_not_entire_cube():
    visuals = _visuals()
    marker_visuals = [
        visual
        for visual in visuals
        if visual.findtext("./material/script/name") == "ArucoSample/Marker"
    ]

    assert len(marker_visuals) == 1
    marker = marker_visuals[0]
    assert marker.find("./geometry/box") is not None
    assert marker.findtext("./geometry/box/size") == "0.07 0.001 0.07"
    assert marker.findtext("pose").split()[1] == "-0.0355"


def test_cube_body_visual_does_not_repeat_the_aruco_texture_on_all_faces():
    box_visuals = [
        visual
        for visual in _visuals()
        if visual.find("./geometry/box") is not None
        and visual.findtext("./material/script/name") != "ArucoSample/Marker"
    ]

    assert len(box_visuals) == 1
    cube = box_visuals[0]
    assert cube.findtext("./geometry/box/size") == "0.07 0.07 0.07"
    assert cube.findtext("./material/script/name") != "ArucoSample/Marker"


def test_collision_contact_surface_uses_tactile_safe_deadband():
    """Keep sample contact settings compatible with tactile grasp phase one."""
    root = _model_root()
    collision = root.find(".//link[@name='link']/collision[@name='collision']")
    ode = collision.find("./surface/contact/ode")

    assert ode is not None
    assert float(ode.findtext("kp")) == 1.0e6
    assert float(ode.findtext("kd")) == 1.0
    assert float(ode.findtext("min_depth")) == 0.001
    assert float(ode.findtext("max_vel")) == 0.01
