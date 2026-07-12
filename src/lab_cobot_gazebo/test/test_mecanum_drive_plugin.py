"""Contracts for the synchronous planar mecanum Gazebo plugin."""

import subprocess
from pathlib import Path
from xml.etree import ElementTree


GAZEBO = Path(__file__).resolve().parents[1]
DESCRIPTION = GAZEBO.parent / "lab_cobot_description"
PLANAR_PLUGIN = "liblab_cobot_planar_drive.so"


def _robot_xml():
    xacro_file = DESCRIPTION / "urdf" / "lab_cobot.urdf.xacro"
    result = subprocess.run(
        ["xacro", str(xacro_file)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ElementTree.fromstring(result.stdout)


def test_generated_robot_uses_synchronous_planar_drive_plugin():
    plugin = next(
        node for node in _robot_xml().iter("plugin")
        if node.get("filename") == PLANAR_PLUGIN
    )
    assert plugin.findtext("wheel_command_topic") == "/wheel_velocity_controller/commands"
    assert plugin.findtext("wheel_radius") == "0.07"
    assert plugin.findtext("wheel_separation_width") == "0.24"
    assert plugin.findtext("wheel_separation_length") == "0.175"


def test_generated_robot_keeps_ros2_control_for_visual_wheel_motion():
    """Wheel joints remain animated independently of whole-model pose drive."""
    plugin_filenames = {
        plugin.get("filename") for plugin in _robot_xml().iter("plugin")
    }
    assert "libgazebo_ros2_control.so" in plugin_filenames


def test_planar_plugin_updates_inside_gazebo_and_disables_model_gravity():
    source = (GAZEBO / "src" / "lab_cobot_planar_drive.cpp").read_text()
    assert "ConnectWorldUpdateBegin" in source
    assert "model_->SetGravityMode(false)" in source
    assert "wheelSpeedsToTwist" in source


def test_planar_plugin_does_not_override_model_velocity():
    """Whole-model velocity writes must not suppress joint visual rotation."""
    source = (GAZEBO / "src" / "lab_cobot_planar_drive.cpp").read_text()
    assert "model_->SetLinearVel" not in source
    assert "model_->SetAngularVel" not in source
