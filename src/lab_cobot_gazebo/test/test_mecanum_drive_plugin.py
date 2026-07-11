"""Cross-file configuration contracts for the mecanum drive plugin."""
# 运动学行为由 test_mecanum_kinematics.cpp (gtest) 覆盖;原源码文本断言已退役:
# - 正解公式/NaN 清洗/限幅/看门狗 -> ament_add_gtest(test_mecanum_kinematics)
#   直测头文件实现(含与 mecanum_wheel_visualizer.py 逆解的互逆性对拍)
# - 默认模式运行时行为 -> 诚实 E2E(机器人真被导航驱动)
# 本文件只保留跨文件一致性合同:URDF 挂接配置与构建产物、可视化器参数一致。
import subprocess
from pathlib import Path
from xml.etree import ElementTree

GAZEBO = Path(__file__).resolve().parents[1]
DESCRIPTION = GAZEBO.parent / "lab_cobot_description"

# 与插件源码 OnUpdate 分发和 UsesWheelCommandWatchdog 一致的模式全集;
# URDF 配置值必须落在其中,否则运行时 gzerr unknown control_mode 且底盘不动。
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
    # 不得回退到已除名的 velocity_from_wheel_commands 等名不副实别名
    plugin = _plugin(_robot_xml())
    mode = plugin.findtext("control_mode")
    assert mode in VALID_CONTROL_MODES
    assert mode == "pose_from_wheel_commands"
    # 插件源码不得再包含旧别名(防止别名复活让命名再度失实)
    source = (GAZEBO / "src" / "lab_cobot_mecanum_drive.cpp").read_text(
        encoding="utf-8"
    )
    assert "velocity_from_wheel_commands" not in source


def test_urdf_kinematics_params_match_visualizer_inverse_kinematics():
    """Plugin FK params must match the visualizer IK params."""
    # 参数不一致则 FK/IK 不再互逆,cmd_vel 与实际车体速度比例失真
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
