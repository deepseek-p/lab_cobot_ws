"""Cross-file configuration contracts for the grasp plugin."""
# 行为逻辑由 test_grasp_envelope.cpp (gtest) 覆盖;原源码文本断言已退役:
# - 封套判定/坐标变换数学 -> ament_add_gtest(test_grasp_envelope) 直测头文件实现
# - attach/detach 运行时行为 -> 诚实 E2E(test_honest_e2e_launch.py)
# 本文件只保留跨文件一致性合同:URDF 挂接的插件库名必须与本包构建产物一致,
# 防止改名后运行时静默加载失败。
import subprocess
from pathlib import Path
from xml.etree import ElementTree

GAZEBO = Path(__file__).resolve().parents[1]
DESCRIPTION = GAZEBO.parent / "lab_cobot_description"


def _robot_xml():
    xacro_file = DESCRIPTION / "urdf" / "lab_cobot.urdf.xacro"
    result = subprocess.run(
        ["xacro", str(xacro_file)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ElementTree.fromstring(result.stdout)


def _plugin(root, name):
    for plugin in root.iter("plugin"):
        if plugin.get("name") == name:
            return plugin
    raise AssertionError(f"URDF 中未找到插件 {name}")


def test_urdf_references_this_package_grasp_plugin_library():
    plugin = _plugin(_robot_xml(), "lab_cobot_grasp_fix")
    assert plugin.get("filename") == "liblab_cobot_grasp_fix.so"
    # 库目标名与 CMakeLists add_library 一致(改名必须双侧同步)
    cmake = (GAZEBO / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "add_library(lab_cobot_grasp_fix SHARED" in cmake


def test_urdf_grasp_envelope_params_match_gtest_contract():
    """Keep URDF envelope limits in sync with the gtest contract."""
    # 与 test_grasp_envelope.cpp 的 UrdfLimits() 逐值对拍
    plugin = _plugin(_robot_xml(), "lab_cobot_grasp_fix")
    assert float(plugin.findtext("max_center_distance")) == 0.220
    assert float(plugin.findtext("max_abs_x")) == 0.125
    assert float(plugin.findtext("max_abs_y")) == 0.105
    assert float(plugin.findtext("min_z")) == -0.060
    assert float(plugin.findtext("max_z")) == 0.220


def test_urdf_grasp_envelope_accepts_observed_stable_pick_offset():
    """Accept the first stable E2E pick offset before fingers push the sample."""
    plugin = _plugin(_robot_xml(), "lab_cobot_grasp_fix")
    limits = {
        "max_center_distance": float(plugin.findtext("max_center_distance")),
        "max_abs_x": float(plugin.findtext("max_abs_x")),
        "max_abs_y": float(plugin.findtext("max_abs_y")),
        "min_z": float(plugin.findtext("min_z")),
        "max_z": float(plugin.findtext("max_z")),
    }
    # 2026-07-10 honest E2E run_3 first refusal while the sample was still
    # on station A: offset=(0.061,0.052,0.021), distance ~= 0.083 m.
    x, y, z = (0.061, 0.052, 0.021)

    assert (x * x + y * y + z * z) ** 0.5 <= limits["max_center_distance"]
    assert abs(x) <= limits["max_abs_x"]
    assert abs(y) <= limits["max_abs_y"]
    assert limits["min_z"] <= z <= limits["max_z"]


def test_urdf_grasp_envelope_accepts_wrist_reference_pick_offset():
    """Accept the measured TCP-to-wrist reference offset used by the plugin."""
    plugin = _plugin(_robot_xml(), "lab_cobot_grasp_fix")
    limits = {
        "max_center_distance": float(plugin.findtext("max_center_distance")),
        "max_abs_x": float(plugin.findtext("max_abs_x")),
        "max_abs_y": float(plugin.findtext("max_abs_y")),
        "min_z": float(plugin.findtext("min_z")),
        "max_z": float(plugin.findtext("max_z")),
    }
    x, y, z = (0.031, 0.010, 0.135)
    assert (x * x + y * y + z * z) ** 0.5 <= limits["max_center_distance"]
    assert abs(x) <= limits["max_abs_x"]
    assert abs(y) <= limits["max_abs_y"]
    assert limits["min_z"] <= z <= limits["max_z"]


def test_urdf_keeps_breakaway_fuse_disabled_by_default():
    """Keep the breakaway force fuse disabled by default."""
    # 位姿驱动底盘下正常搬运的 fixed-joint ERP 校正力与异常力同量级,
    # 启用会把物块半路丢掉(实测),故必须保持禁用。
    plugin = _plugin(_robot_xml(), "lab_cobot_grasp_fix")
    breakaway = plugin.findtext("breakaway_force")
    assert breakaway is None or float(breakaway) == 0.0


def test_urdf_enables_noninvasive_virtual_force_sensor_for_stable_g4():
    """Stable G4 should expose force curves without physical probe response."""
    plugin = _plugin(_robot_xml(), "lab_cobot_grasp_fix")

    assert plugin.findtext("virtual_force_sensor") == "true"
    assert float(plugin.findtext("virtual_force_stiffness")) > 0.0
    assert float(plugin.findtext("virtual_force_baseline")) >= 0.0
    assert float(plugin.findtext("virtual_force_max")) > 0.0


def test_urdf_grasp_candidate_list_is_exactly_the_e2e_sample():
    """Keep the default grasp candidate list limited to the E2E sample."""
    plugin = _plugin(_robot_xml(), "lab_cobot_grasp_fix")

    assert [elem.text for elem in plugin.findall("object_model")] == ["aruco_sample"]


def test_grasp_candidates_are_dynamic_models_with_link_named_link():
    """Require every grasp candidate to be dynamic with a link named link."""
    plugin = _plugin(_robot_xml(), "lab_cobot_grasp_fix")
    candidates = [elem.text for elem in plugin.findall("object_model")]

    for name in candidates:
        model_file = GAZEBO / "models" / name / "model.sdf"
        root = ElementTree.parse(model_file).getroot()
        model = root.find(f".//model[@name='{name}']")
        assert model is not None
        static = model.findtext("static")
        assert static is None or static.strip().lower() == "false"
        assert model.find("./link[@name='link']") is not None


def test_urdf_keeps_require_finger_contact_disabled_by_default():
    """Keep the bare-xacro default conservative; launch layer flips it on."""
    # T-5 已在 launch 层翻默认(lab_cobot/world launch 传 true);
    # xacro 单独展开(如 MoveIt 加载)保持保守 false,本测试锁定该分层。
    plugin = _plugin(_robot_xml(), "lab_cobot_grasp_fix")
    require_contact = plugin.findtext("require_finger_contact")

    assert require_contact is None or require_contact.strip().lower() == "false"


def test_urdf_tactile_contact_gate_attaches_on_first_dual_contact_frame():
    """Attach immediately once both tactile fingers contact the target."""
    plugin = _plugin(_robot_xml(), "lab_cobot_grasp_fix")

    assert int(plugin.findtext("contact_count_threshold")) == 1


def test_urdf_tactile_attach_uses_gazebo_stable_wrist_link():
    """Attach tactile grasps to a stable Gazebo link."""
    plugin = _plugin(_robot_xml(), "lab_cobot_grasp_fix")

    assert plugin.findtext("stable_attach_link") == "ur_wrist_3_link"
