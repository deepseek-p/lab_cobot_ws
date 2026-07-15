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


def test_front_aruco_patch_contract_is_unchanged():
    marker = _model_root().find(
        ".//link[@name='link']/visual[@name='aruco_marker_front']"
    )

    assert marker is not None
    assert marker.findtext("pose") == "0 -0.0355 0 0 0 0"
    assert marker.findtext("./geometry/box/size") == "0.07 0.001 0.07"
    assert marker.findtext("./material/script/name") == "ArucoSample/Marker"


def test_top_aruco_patch_uses_marker_id_one_material():
    marker = _model_root().find(
        ".//link[@name='link']/visual[@name='aruco_marker_top']"
    )

    assert marker is not None
    assert marker.findtext("pose") == "0 0 0.0355 0 0 0"
    assert marker.findtext("./geometry/box/size") == "0.07 0.07 0.001"
    assert marker.findtext("./material/script/name") == "ArucoSample/MarkerTop"


def test_cube_body_visual_does_not_repeat_the_aruco_texture_on_all_faces():
    cube = _model_root().find(".//link[@name='link']/visual[@name='body_visual']")

    assert cube is not None
    assert cube.findtext("./geometry/box/size") == "0.07 0.07 0.07"
    assert cube.find("./material/script") is None


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
