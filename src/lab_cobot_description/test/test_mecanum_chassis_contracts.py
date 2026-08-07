"""Contracts for the GitHub main box mecanum chassis."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE / "urdf" / "lab_cobot.urdf.xacro"
WHEEL_JOINTS = [
    "wheel_fl_joint",
    "wheel_fr_joint",
    "wheel_rl_joint",
    "wheel_rr_joint",
]
WHEEL_LINKS = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]


def _generated():
    text = subprocess.run(
        ["xacro", str(URDF)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    return text, ET.fromstring(text)


def test_main_chassis_structure_and_dimensions():
    text, root = _generated()
    links = {node.attrib["name"] for node in root.findall("link")}
    joints = {node.attrib["name"]: node for node in root.findall("joint")}

    assert {"base_footprint", "base_link", *WHEEL_LINKS} <= links
    assert "chassis_mount_link" not in links
    assert not any(name.endswith("_arm") for name in links)
    assert set(WHEEL_JOINTS) <= joints.keys()
    assert all(joints[name].attrib["type"] == "continuous" for name in WHEEL_JOINTS)
    assert "package://lab_cobot_description/meshes/mecanum3" not in text

    collision = root.find("./link[@name='base_link']/collision/geometry/box")
    assert collision is not None
    assert [float(value) for value in collision.attrib["size"].split()] == [
        0.55,
        0.50,
        0.15,
    ]
    mass = root.find("./link[@name='base_link']/inertial/mass")
    assert mass is not None
    assert float(mass.attrib["value"]) == 180.0


def test_main_chassis_ground_and_mount_contracts():
    _, root = _generated()

    footprint_joint = root.find("./joint[@name='base_footprint_joint']")
    assert footprint_joint is not None
    assert footprint_joint.find("parent").attrib["link"] == "base_footprint"
    assert footprint_joint.find("child").attrib["link"] == "base_link"
    assert float(footprint_joint.find("origin").attrib["xyz"].split()[2]) == 0.155

    pillar = root.find("./joint[@name='pillar_joint']")
    assert pillar is not None
    assert pillar.find("parent").attrib["link"] == "base_link"
    assert float(pillar.find("origin").attrib["xyz"].split()[2]) == 0.075


def test_ros2_control_and_controller_use_main_wheel_order():
    _, root = _generated()
    exported = [
        joint.attrib["name"]
        for control in root.findall("ros2_control")
        for joint in control.findall("joint")
        if joint.attrib["name"] in WHEEL_JOINTS
    ]
    assert exported == WHEEL_JOINTS

    config = yaml.safe_load(
        (PACKAGE / "config" / "lab_cobot_controllers.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert config["wheel_velocity_controller"]["ros__parameters"]["joints"] == (
        WHEEL_JOINTS
    )


def test_srdf_references_only_generated_chassis_links_and_main_wheels():
    _, root = _generated()
    urdf_links = {node.attrib["name"] for node in root.findall("link")}
    srdf = ET.fromstring(
        (PACKAGE / "srdf" / "lab_cobot.srdf").read_text(encoding="utf-8")
    )
    referenced = {
        collision.attrib[field]
        for collision in srdf.findall("disable_collisions")
        for field in ("link1", "link2")
    }
    assert referenced <= urdf_links
    assert [node.attrib["name"] for node in srdf.findall("passive_joint")] == (
        WHEEL_JOINTS
    )


def test_description_no_longer_installs_mecanum3_meshes():
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    install_block = cmake.split("install(DIRECTORY", 1)[1].split(
        "DESTINATION", 1
    )[0]
    assert "meshes" not in install_block.split()
