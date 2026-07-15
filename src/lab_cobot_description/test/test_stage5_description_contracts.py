"""Regression checks for Stage 5 simulation contracts."""
import ast
import importlib.util
import math
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
import yaml


def _src_dir() -> Path:
    return Path(__file__).resolve().parents[2]


def _load_python_module(module_path: Path, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _literal_python_constant(module_path: Path, constant_name: str):
    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        else:
            continue

        if any(
            isinstance(target, ast.Name) and target.id == constant_name
            for target in targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(f"missing constant {constant_name} in {module_path}")


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


def test_parallel_gripper_fingers_have_collision_for_physical_grasp():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)

    for link_name in (
        "gripper_left_finger",
        "gripper_right_finger",
    ):
        link = root.find(f"./link[@name='{link_name}']")
        assert link is not None
        collisions = link.findall("collision")
        assert len(collisions) == 1
        main_box = collisions[0].find("./geometry/box")
        assert main_box is not None
        assert main_box.attrib["size"] == "0.045 0.012 0.075"


def test_tactile_probe_collisions_are_gazebo_only():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)

    for side in ("left", "right"):
        link = root.find(f"./link[@name='gripper_{side}_finger']")
        assert link is not None
        assert not link.findall("collision[@name]")


def test_gazebo_tactile_probe_collision_names_match_bumper_sensors():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file), "gazebo_tactile_probe:=true"],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as urdf_file_obj:
        urdf_file_obj.write(urdf)
        urdf_file_obj.flush()
        sdf = subprocess.run(
            ["gz", "sdf", "-p", urdf_file_obj.name],
            check=True,
            stdout=subprocess.PIPE,
            text=True,
        ).stdout
    root = ET.fromstring(sdf)

    for side in ("left", "right"):
        link = root.find(f".//link[@name='gripper_{side}_finger']")
        assert link is not None
        assert link.find(
            f"./collision[@name='gripper_{side}_finger_tactile_probe_collision_1']"
        ) is not None

        sensor = link.find(f"./sensor[@name='gripper_{side}_finger_bumper']")
        assert sensor is not None
        assert sensor.findtext("./contact/collision") == (
            f"gripper_{side}_finger_tactile_probe_collision_1"
        )


def test_generated_urdf_uses_wheel_command_pose_drive_instead_of_planar_move_plugin():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)

    assert "gazebo_ros_planar_move" not in urdf
    assert "libgazebo_ros_planar_move.so" not in urdf
    for wheel_name in ("front_left_wheel_1", "front_right_wheel_1", "back_left_wheel_1", "back_right_wheel_1"):
        wheel = root.find(f"./link[@name='{wheel_name}']")
        assert wheel is not None
        assert wheel.find("collision") is not None


def test_camera_is_offset_from_arm_centerline_to_avoid_self_occlusion():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)
    origin = root.find("./joint[@name='camera_joint']/origin")

    assert origin is not None
    x, y, z = [float(value) for value in origin.attrib["xyz"].split()]
    assert x >= 0.15
    assert abs(y) >= 0.18
    assert z >= 0.50


def test_generated_urdf_omits_legacy_mecanum_traction_plugin():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(["xacro", str(urdf_file)], check=True, stdout=subprocess.PIPE, text=True).stdout
    assert "liblab_cobot_mecanum_drive.so" not in urdf


def test_default_mecanum_drive_uses_physical_roller_contact():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(["xacro", str(urdf_file)], check=True, stdout=subprocess.PIPE, text=True).stdout
    root = ET.fromstring(urdf)
    assert root.find("./gazebo[@reference='front_left_barrel_0_link']/mu1").text == "1.0"


def test_commanded_velocity_drive_uses_low_front_right_wheel_1iction():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(["xacro", str(urdf_file)], check=True, stdout=subprocess.PIPE, text=True).stdout
    root = ET.fromstring(urdf)
    for wheel in ("front_left_wheel_1", "front_right_wheel_1", "back_left_wheel_1", "back_right_wheel_1"):
        assert root.find(f"./gazebo[@reference='{wheel}']/mu1").text == "0.05"


def test_generated_urdf_loads_grasp_fix_for_contact_based_holding():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)
    plugin = root.find(".//plugin[@name='lab_cobot_grasp_fix']")

    assert plugin is not None
    assert plugin.attrib["filename"] == "liblab_cobot_grasp_fix.so"
    assert plugin.findtext("object_model") == "aruco_sample"
    assert plugin.findtext("object_link") == "link"
    assert plugin.findtext("tcp_link") == "gripper_tcp"
    assert plugin.findtext("left_joint") == "gripper_left_finger_joint"
    assert plugin.findtext("right_joint") == "gripper_right_finger_joint"
    assert float(plugin.findtext("close_threshold")) <= 0.003
    assert plugin.findtext("grip_count_threshold") == "1"
    assert plugin.findtext("grasp_center_offset") == "-0.037 0.0 0.030"
    assert float(plugin.findtext("max_center_distance")) >= 0.090
    assert plugin.findtext("max_abs_x") == "0.065"
    assert plugin.findtext("max_abs_y") == "0.055"
    assert float(plugin.findtext("max_z")) >= 0.075


def test_generated_urdf_adds_contact_sensors_to_both_fingers():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)

    for side in ("left", "right"):
        gazebo = root.find(f"./gazebo[@reference='gripper_{side}_finger']")
        sensor = gazebo.find(f"./sensor[@name='gripper_{side}_finger_bumper']")
        plugin = sensor.find(f"./plugin[@name='gripper_{side}_finger_bumper_plugin']")

        assert sensor.attrib["type"] == "contact"
        assert sensor.findtext("./contact/collision") == (
            f"gripper_{side}_finger_tactile_probe_collision_1"
        )
        assert sensor.findtext("always_on") == "true"
        assert sensor.findtext("update_rate") == "50.0"
        assert gazebo.findtext("maxVel") == "0.01"
        assert plugin.attrib["filename"] == "libgazebo_ros_bumper.so"
        assert plugin.findtext("./ros/namespace") == "/gripper"
        assert plugin.findtext("./ros/remapping") == (
            f"bumper_states:={side}_finger_contacts"
        )
        assert plugin.findtext("frame_name") == f"gripper_{side}_finger"


def test_moveit_marks_uncontrolled_wheel_joints_passive():
    srdf_file = Path(__file__).resolve().parents[1] / "srdf" / "lab_cobot.srdf"
    text = srdf_file.read_text(encoding="utf-8")

    for joint in (
        "front_left_joint",
        "front_right_joint",
        "back_left_joint",
        "back_right_joint",
    ):
        assert f'<passive_joint name="{joint}"/>' in text


def test_generated_urdf_renders_mecanum_roller_visuals():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(["xacro", str(urdf_file)], check=True, stdout=subprocess.PIPE, text=True).stdout
    root = ET.fromstring(urdf)
    assert len([link for link in root.findall("link") if "_barrel_" in link.attrib["name"]]) == 60


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
        "front_left_joint",
        "front_right_joint",
        "back_left_joint",
        "back_right_joint",
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
        "front_left_joint",
        "front_right_joint",
        "back_left_joint",
        "back_right_joint",
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
    for finger in ("gripper_left_finger", "gripper_right_finger"):
        assert frozenset((finger, "ur_wrist_1_link")) in disabled_pairs
        assert frozenset((finger, "ur_forearm_link")) in disabled_pairs


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
    waypoints = _load_python_module(
        _src_dir() / "lab_cobot_navigation" / "lab_cobot_navigation" / "waypoints.py",
        "_lab_cobot_waypoints_contract",
    )
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
    base_height = 0.15
    wheel_radius = 0.08
    assert abs(arm_x) <= base_length / 2.0
    assert abs(arm_y) <= base_width / 2.0

    station_a = waypoints.get_waypoint("station_a")
    sample_map_x = 2.0
    sample_map_y = 1.32
    station_table_front_y = 1.20
    inflation_radius = 0.55
    sample_top_world_z = 0.82

    assert station_a["x"] == sample_map_x
    assert math.isclose(station_a["yaw"], math.pi / 2.0)
    assert station_table_front_y - station_a["y"] > inflation_radius

    # Sample top center at station_a, expressed in base_link at the waypoint.
    sample_top_x = sample_map_y - station_a["y"]
    sample_top_y = 0.0
    sample_top_z = sample_top_world_z - (wheel_radius + base_height / 2.0)

    camera_x = arm_x + cam_x
    camera_y = arm_y + cam_y
    camera_z = base_height / 2.0 + arm_z + cam_z
    target_x = sample_top_x - camera_x
    target_y = sample_top_y - camera_y
    target_z = sample_top_z - camera_z

    optical_x = -target_y
    optical_y = -math.sin(cam_pitch) * target_x - math.cos(cam_pitch) * target_z
    optical_z = math.cos(cam_pitch) * target_x - math.sin(cam_pitch) * target_z

    assert optical_z > 0.05
    assert abs(optical_x / optical_z) < math.tan(1.047 / 2.0)
    assert abs(optical_y / optical_z) < math.tan(1.047 / 2.0) * (480 / 640)


def test_wrist_refine_camera_is_absent_by_default():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)

    assert root.find("./link[@name='wrist_camera_link']") is None
    assert root.find(".//sensor[@name='wrist_camera']") is None


def test_wrist_refine_camera_contract_when_enabled():
    urdf_file = Path(__file__).resolve().parents[1] / "urdf" / "lab_cobot.urdf.xacro"
    urdf = subprocess.run(
        ["xacro", str(urdf_file), "wrist_refine_camera:=true"],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout
    root = ET.fromstring(urdf)

    assert root.find("./link[@name='wrist_camera_link']") is not None
    assert root.find("./link[@name='wrist_camera_optical_frame']") is not None
    camera_joint = root.find("./joint[@name='wrist_camera_joint']")
    assert camera_joint is not None
    assert camera_joint.find("parent").attrib["link"] == "gripper_base"
    assert camera_joint.find("child").attrib["link"] == "wrist_camera_link"
    camera_xyz = [
        float(value)
        for value in camera_joint.find("origin").attrib["xyz"].split()
    ]
    camera_rpy = [
        float(value)
        for value in camera_joint.find("origin").attrib["rpy"].split()
    ]
    # EIH uses the top marker with a known small lateral camera offset and a
    # parallel downward optical axis. The nearest-distance probe keeps the
    # complete marker visible without wrist self-occlusion.
    assert camera_xyz[0] > 0.02
    assert camera_rpy[1] == pytest.approx(-math.pi / 2.0, abs=1.0e-6)
    optical_joint = root.find("./joint[@name='wrist_camera_optical_joint']")
    assert optical_joint is not None
    sensor = root.find(".//sensor[@name='wrist_camera']")
    assert sensor is not None
    assert sensor.attrib["type"] == "depth"
    assert float(sensor.findtext("./camera/horizontal_fov")) == pytest.approx(
        math.radians(65.0), abs=1.0e-6
    )
    assert sensor.findtext("update_rate") == "30"
    assert float(sensor.findtext("./camera/clip/near")) == pytest.approx(0.01)
    assert float(sensor.findtext("./plugin/min_depth")) == pytest.approx(0.01)
    gazebo = root.find("./gazebo[@reference='wrist_camera_link']")
    assert gazebo is not None
    assert gazebo.findtext("material") == "Gazebo/DarkGrey"


def test_arm_trajectory_controller_reports_success_only_after_settling():
    config_path = (
        _src_dir()
        / "lab_cobot_description"
        / "config"
        / "lab_cobot_controllers.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    params = config["joint_trajectory_controller"]["ros__parameters"]
    constraints = params["constraints"]

    assert 0.0 < constraints["stopped_velocity_tolerance"] <= 0.02
    assert 2.0 <= constraints["goal_time"] <= 8.0
    for joint in params["joints"]:
        assert constraints[joint]["trajectory"] >= 0.05
        assert 0.0 < constraints[joint]["goal"] <= 0.005
    assert constraints["ur_wrist_3_joint"]["trajectory"] == 0.12
    for joint in params["joints"]:
        if joint == "ur_wrist_3_joint":
            continue
        assert constraints[joint]["trajectory"] == 0.10
        assert constraints[joint]["goal"] == 0.005
