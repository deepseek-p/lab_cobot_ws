"""Regression checks for Stage 5 simulation contracts."""
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


def test_parallel_gripper_replaces_vacuum_in_generated_urdf():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = {joint.attrib["name"] for joint in root.findall("joint")}

    assert {
        "gripper_base",
        "gripper_left_finger",
        "gripper_right_finger",
        "gripper_tcp",
    } <= links
    assert {
        "gripper_left_finger_joint",
        "gripper_right_finger_joint",
        "gripper_tcp_joint",
    } <= joints
    assert "vacuum_gripper" not in urdf
    assert "/suction/switch" not in urdf


def test_gripper_controller_commands_both_finger_joints():
    config_file = (
        Path(__file__).resolve().parents[1] / "config" / "lab_cobot_controllers.yaml"
    )
    params = yaml.safe_load(config_file.read_text(encoding="utf-8"))
    manager = params["controller_manager"]["ros__parameters"]
    gripper = params["gripper_position_controller"]["ros__parameters"]

    assert (
        manager["gripper_position_controller"]["type"]
        == "position_controllers/JointGroupPositionController"
    )
    assert gripper["joints"] == [
        "gripper_left_finger_joint",
        "gripper_right_finger_joint",
    ]


def test_generated_urdf_exports_gripper_joints_to_ros2_control():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)
    ros2_control_joints = {
        joint.attrib["name"]
        for control in root.findall("ros2_control")
        for joint in control.findall("joint")
    }

    assert {
        "gripper_left_finger_joint",
        "gripper_right_finger_joint",
    } <= ros2_control_joints


def test_parallel_gripper_is_visual_only_for_soft_attach_simulation():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)

    for link_name in (
        "gripper_base",
        "gripper_left_finger",
        "gripper_right_finger",
    ):
        link = root.find(f"./link[@name='{link_name}']")
        assert link is not None
        assert link.find("collision") is None


def test_moveit_marks_uncontrolled_wheel_joints_passive():
    srdf_file = Path(__file__).resolve().parents[1] / "srdf" / "lab_cobot.srdf"
    text = srdf_file.read_text(encoding="utf-8")

    for joint in (
        "wheel_fl_joint",
        "wheel_fr_joint",
        "wheel_rl_joint",
        "wheel_rr_joint",
    ):
        assert f'<passive_joint name="{joint}"/>' in text


def test_generated_urdf_renders_mecanum_roller_visuals():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)

    for wheel_name in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
        wheel = root.find(f"./link[@name='{wheel_name}']")
        assert wheel is not None
        roller_visuals = [
            visual
            for visual in wheel.findall("visual")
            if "roller" in visual.attrib.get("name", "")
        ]

        assert len(roller_visuals) >= 10


def test_generated_urdf_exports_wheel_velocity_interfaces_to_ros2_control():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)
    ros2_control_joints = {
        joint.attrib["name"]: joint
        for control in root.findall("ros2_control")
        for joint in control.findall("joint")
    }

    for joint_name in (
        "wheel_fl_joint",
        "wheel_fr_joint",
        "wheel_rl_joint",
        "wheel_rr_joint",
    ):
        joint = ros2_control_joints[joint_name]
        command_interfaces = [
            interface.attrib["name"] for interface in joint.findall("command_interface")
        ]
        state_interfaces = [
            interface.attrib["name"] for interface in joint.findall("state_interface")
        ]

        assert command_interfaces == ["velocity"]
        assert {"position", "velocity"} <= set(state_interfaces)


def test_wheel_velocity_controller_commands_mecanum_wheel_joints():
    config_file = (
        Path(__file__).resolve().parents[1] / "config" / "lab_cobot_controllers.yaml"
    )
    params = yaml.safe_load(config_file.read_text(encoding="utf-8"))
    manager = params["controller_manager"]["ros__parameters"]
    controller = params["wheel_velocity_controller"]["ros__parameters"]

    assert (
        manager["wheel_velocity_controller"]["type"]
        == "velocity_controllers/JointGroupVelocityController"
    )
    assert controller["joints"] == [
        "wheel_fl_joint",
        "wheel_fr_joint",
        "wheel_rl_joint",
        "wheel_rr_joint",
    ]


def test_srdf_declares_gripper_group_and_collision_pairs():
    srdf_file = Path(__file__).resolve().parents[1] / "srdf" / "lab_cobot.srdf"
    root = ET.fromstring(srdf_file.read_text(encoding="utf-8"))
    assert root.find("./group[@name='gripper']") is not None
    assert root.find("./end_effector[@name='gripper']") is not None
    disabled_pairs = {
        frozenset((elem.attrib["link1"], elem.attrib["link2"]))
        for elem in root.findall("disable_collisions")
    }

    assert frozenset(("gripper_left_finger", "gripper_right_finger")) in disabled_pairs
    assert frozenset(("gripper_base", "ur_wrist_3_link")) in disabled_pairs
    assert frozenset(("gripper_base", "ur_forearm_link")) in disabled_pairs


def _origin_xyz_rpy(urdf: str, joint_name: str):
    root = ET.fromstring(urdf)
    joint = root.find(f"./joint[@name='{joint_name}']")
    assert joint is not None, f"missing joint {joint_name}"
    origin = joint.find("origin")
    assert origin is not None, f"missing origin for {joint_name}"
    xyz = tuple(float(v) for v in origin.attrib["xyz"].split())
    rpy = tuple(float(v) for v in origin.attrib.get("rpy", "0 0 0").split())
    return xyz, rpy


def test_station_a_sample_projects_into_camera_image():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout

    (arm_x, arm_y, arm_z), _ = _origin_xyz_rpy(urdf, "arm_mount_joint")
    (cam_x, cam_y, cam_z), (_, cam_pitch, _) = _origin_xyz_rpy(
        urdf, "camera_joint"
    )

    base_length = 0.55
    base_width = 0.50
    assert abs(arm_x) <= base_length / 2.0
    assert abs(arm_y) <= base_width / 2.0

    # Nominal station_a stop: base_link at map (2.0, 0.75, base_z), yaw +90deg.
    # Sample top center is at map (2.0, 1.5, 0.82). Expressed in base_link:
    sample_top_x = 1.5 - 0.75
    sample_top_y = 0.0
    sample_top_z = 0.82 - (0.08 + 0.15 / 2.0)

    camera_x = arm_x + cam_x
    camera_y = arm_y + cam_y
    camera_z = arm_z + cam_z
    target_x = sample_top_x - camera_x
    target_y = sample_top_y - camera_y
    target_z = sample_top_z - camera_z

    optical_x = -target_y
    optical_y = -math.sin(cam_pitch) * target_x - math.cos(cam_pitch) * target_z
    optical_z = math.cos(cam_pitch) * target_x - math.sin(cam_pitch) * target_z

    assert optical_z > 0.05
    assert abs(optical_x / optical_z) < math.tan(1.047 / 2.0)
    assert abs(optical_y / optical_z) < math.tan(1.047 / 2.0) * (480 / 640)
