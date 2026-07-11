"""Contracts preventing the retired traction plugin from returning."""

import subprocess
from pathlib import Path
from xml.etree import ElementTree


GAZEBO = Path(__file__).resolve().parents[1]
DESCRIPTION = GAZEBO.parent / "lab_cobot_description"
RETIRED_PLUGIN = "liblab_cobot_mecanum_drive.so"


def _robot_xml():
    xacro_file = DESCRIPTION / "urdf" / "lab_cobot.urdf.xacro"
    result = subprocess.run(
        ["xacro", str(xacro_file)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ElementTree.fromstring(result.stdout)


def test_generated_robot_omits_retired_traction_plugin():
    """The pose runtime is a node; the old in-model drive must stay detached."""
    plugin_filenames = {
        plugin.get("filename") for plugin in _robot_xml().iter("plugin")
    }
    assert RETIRED_PLUGIN not in plugin_filenames


def test_generated_robot_keeps_ros2_control_for_visual_wheel_motion():
    """Wheel joints remain animated independently of whole-model pose drive."""
    plugin_filenames = {
        plugin.get("filename") for plugin in _robot_xml().iter("plugin")
    }
    assert "libgazebo_ros2_control.so" in plugin_filenames


def test_description_source_does_not_reference_retired_plugin():
    """Guard every Xacro, not just the default expansion path."""
    urdf_dir = DESCRIPTION / "urdf"
    references = [
        path
        for path in urdf_dir.rglob("*.xacro")
        if RETIRED_PLUGIN in path.read_text(encoding="utf-8")
    ]
    assert references == []
