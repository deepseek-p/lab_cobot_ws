"""Cross-file configuration contracts for the mecanum drive plugin."""
# 运动学行为由 test_mecanum_kinematics.cpp (gtest) 覆盖;本文件只保留
# URDF 挂接配置、构建产物和可视化器参数的跨文件一致性合同。
import subprocess
from pathlib import Path
from xml.etree import ElementTree


GAZEBO = Path(__file__).resolve().parents[1]
DESCRIPTION = GAZEBO.parent / "lab_cobot_description"
VALID_CONTROL_MODES = {
    "pose_from_wheel_commands",
    "pose_from_wheel_joints",
    "velocity_from_wheel_joints",
    "force_from_wheel_joints",
}


def _robot_xml():
    xacro_file = DESCRIPTION / "urdf" / "lab_cobot.urdf.xacro"
    result = subprocess.run(
        ["xacro", str(xacro_file)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ElementTree.fromstring(result.stdout)


def _plugin(root):
    for plugin in root.iter("plugin"):
        if plugin.get("name") == "lab_cobot_mecanum_traction":
            return plugin
    raise AssertionError("URDF 中未找到插件 lab_cobot_mecanum_traction")


def test_urdf_references_this_package_mecanum_plugin_library():
    plugin = _plugin(_robot_xml())
    assert plugin.get("filename") == "liblab_cobot_mecanum_drive.so"
    cmake = (GAZEBO / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "add_library(lab_cobot_mecanum_drive SHARED" in cmake


def test_urdf_control_mode_is_valid_and_honestly_named_pose_drive():
    """Default control mode must be the honestly named pose drive."""
    plugin = _plugin(_robot_xml())
    mode = plugin.findtext("control_mode")
    assert mode in VALID_CONTROL_MODES
    assert mode == "pose_from_wheel_commands"
    source = (GAZEBO / "src" / "lab_cobot_mecanum_drive.cpp").read_text(
        encoding="utf-8"
    )
    assert "velocity_from_wheel_commands" not in source


def test_urdf_kinematics_params_match_visualizer_inverse_kinematics():
    """Plugin FK params must match the visualizer IK params."""
    plugin = _plugin(_robot_xml())
    wheel_radius = float(plugin.findtext("wheel_radius"))
    wheelbase_radius = float(plugin.findtext("wheelbase_radius"))

    import importlib.util

    visualizer = (
        GAZEBO.parent
        / "lab_cobot_bringup"
        / "lab_cobot_bringup"
        / "mecanum_wheel_visualizer.py"
    )
    spec = importlib.util.spec_from_file_location("mwv_contract", visualizer)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert wheel_radius == module.WHEEL_RADIUS
    assert wheelbase_radius == module.WHEELBASE_RADIUS


def test_urdf_wheel_joint_order_matches_visualizer_convention():
    """Wheel joint order must follow the [fl, fr, rl, rr] convention."""
    plugin = _plugin(_robot_xml())
    joints = [element.text for element in plugin.findall("wheel_joint")]
    assert joints == [
        "wheel_fl_joint",
        "wheel_fr_joint",
        "wheel_rl_joint",
        "wheel_rr_joint",
    ]


def test_drive_plugin_is_the_single_odom_and_tf_source():
    """The drive plugin must publish matching odometry and base TF."""
    plugin = _plugin(_robot_xml())
    assert plugin.findtext("publish_odom") == "true"
    assert plugin.findtext("publish_odom_tf") == "true"
    assert plugin.findtext("odom_frame") == "odom"
    assert plugin.findtext("base_frame") == "base_footprint"

    source = (GAZEBO / "src" / "lab_cobot_mecanum_drive.cpp").read_text(
        encoding="utf-8"
    )
    assert "tf_broadcaster_->sendTransform(transform)" in source
