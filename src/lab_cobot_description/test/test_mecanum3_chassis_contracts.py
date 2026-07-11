"""Contracts for the imported mecanum3 chassis."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml
PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE / "urdf" / "lab_cobot.urdf.xacro"
WHEEL_JOINTS = ["front_left_joint", "front_right_joint", "back_left_joint", "back_right_joint"]
WHEEL_LINKS = ["front_left_wheel_1", "front_right_wheel_1", "back_left_wheel_1", "back_right_wheel_1"]

def _generated():
    text = subprocess.run(["xacro", str(URDF)], check=True, stdout=subprocess.PIPE, text=True).stdout
    return text, ET.fromstring(text)

def test_mecanum3_structure_and_joint_contracts():
    _, root = _generated()
    links = {n.attrib["name"] for n in root.findall("link")}
    joints = {n.attrib["name"]: n for n in root.findall("joint")}
    assert {"base_footprint", "base_link", "chassis_mount_link"} <= links
    assert {"front_left_arm", "front_right_arm", "back_left_arm", "back_right_arm"} <= links
    assert set(WHEEL_LINKS) <= links
    suspension = ["front_left_suspension_joint", "front_right_suspension_joint", "back_left_suspension_joint", "back_right_suspension_joint"]
    assert set(suspension) <= joints.keys()
    assert [n.attrib["name"] for n in root.findall("joint") if n.attrib["name"] in WHEEL_JOINTS] == WHEEL_JOINTS
    assert all(joints[name].attrib["type"] == "continuous" for name in WHEEL_JOINTS)

def test_all_sixty_rollers_are_preserved():
    _, root = _generated()
    links = {n.attrib["name"] for n in root.findall("link")}
    joints = {n.attrib["name"] for n in root.findall("joint")}
    expected = {f"{wheel}_barrel_{i}_link" for wheel in ("front_left", "back_left", "front_right", "back_right") for i in range(15)}
    assert expected <= links
    assert {name[:-5] + "_joint" for name in expected} <= joints

def test_meshes_are_packaged_and_uris_and_base_scale_are_correct():
    text, root = _generated()
    names = ["base_link.stl", "arms.stl", "mecanum_wheel.stl", "mecanum_wheel_rev.stl", "mecanum_barrel.stl"]
    for name in names:
        assert (PACKAGE / "meshes" / "mecanum3" / name).is_file()
        assert f"package://lab_cobot_description/meshes/mecanum3/{name}" in text
    mesh = root.find("./link[@name='base_link']/visual/geometry/mesh")
    assert mesh is not None and mesh.attrib["scale"] == "0.001 0.001 0.001"

def test_mount_and_removed_legacy_drive_contracts():
    text, root = _generated()
    mount = root.find("./joint[@name='chassis_mount_joint']")
    assert mount is not None
    assert mount.find("parent").attrib["link"] == "base_link"
    assert mount.find("origin").attrib["xyz"] == "0 0 0.165"
    assert root.find("./joint[@name='pillar_joint']/parent").attrib["link"] == "chassis_mount_link"
    assert "liblab_cobot_mecanum_drive.so" not in text
    assert all(old not in text for old in ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"))

def test_ros2_control_and_controller_yaml_use_source_wheel_order():
    _, root = _generated()
    exported = [joint.attrib["name"] for control in root.findall("ros2_control") for joint in control.findall("joint") if joint.attrib["name"] in WHEEL_JOINTS]
    assert exported == WHEEL_JOINTS
    config = yaml.safe_load((PACKAGE / "config" / "lab_cobot_controllers.yaml").read_text())
    assert config["wheel_velocity_controller"]["ros__parameters"]["joints"] == WHEEL_JOINTS

