"""Environment model contracts for the five-zone lab scene."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


GAZEBO = Path(__file__).resolve().parents[1]


def _model_root(name):
    return ET.parse(GAZEBO / "models" / name / "model.sdf").getroot()


def _world_root():
    return ET.parse(GAZEBO / "worlds" / "lab.world").getroot()


def _include_pose(entity_name):
    for include in _world_root().findall(".//include"):
        if include.findtext("name") == entity_name:
            return [float(value) for value in include.findtext("pose").split()]
    raise AssertionError(f"missing include for {entity_name}")


def _include_uri(entity_name):
    for include in _world_root().findall(".//include"):
        if include.findtext("name") == entity_name:
            return include.findtext("uri")
    raise AssertionError(f"missing include for {entity_name}")


def test_plain_igbt_asset_has_dynamic_collision_contract():
    model = _model_root("igbt_module_plain").find("model")

    assert model.findtext("static") is None
    assert model.findtext(".//link/inertial/mass") is not None
    assert model.findtext(
        ".//collision/geometry/box/size"
    ) == "0.140 0.380 0.120"
    assert model.find(".//collision/surface/friction") is not None
    assert model.findtext(".//visual/geometry/mesh/uri") == (
        "model://igbt_module_plain/meshes/digital_caliper.dae"
    )


def test_fixture_box_asset_has_dynamic_collision_contract():
    model = _model_root("fixture_box_plain").find("model")

    assert model.findtext("static") is None
    assert model.findtext(".//link/inertial/mass") is not None
    assert model.findtext(
        ".//collision/geometry/box/size"
    ) == "0.120 0.340 0.200"
    assert model.find(".//collision/surface/friction") is not None
    assert model.findtext(".//visual/geometry/mesh/uri") == (
        "model://fixture_box_plain/meshes/adjustable_wrench.dae"
    )


def test_aging_rack_has_three_visual_slots_and_status_panel():
    root = _model_root("aging_rack")
    assert root.find(".//visual[@name='slot_left']") is not None
    assert root.find(".//visual[@name='slot_mid']") is not None
    assert root.find(".//visual[@name='slot_right']") is not None
    assert root.find(".//visual[@name='status_green']") is not None
    assert root.find(".//visual[@name='status_yellow']") is not None
    assert root.find(".//visual[@name='status_red']") is not None


def test_new_tabletop_props_use_expected_imported_meshes():
    expected_meshes = {
        "thermal_grease_can": "meter_closed.dae",
        "tooling_hand_tools": "pliers_closed.dae",
        "pcb_test_fixture": "drill.dae",
        "safety_probe_kit": "screwdriver.dae",
    }
    for model_name, mesh_name in expected_meshes.items():
        assert _model_root(model_name).findtext(
            ".//visual/geometry/mesh/uri"
        ) == f"model://{model_name}/meshes/{mesh_name}"


def test_world_uses_four_worktables_plus_hv_and_home_zones():
    world = _world_root()
    names = {model.get("name") for model in world.findall(".//model")}
    assert {
        "station_a_table",
        "tooling_zone_table",
        "aging_zone_table",
        "station_b_table",
        "home_zone_pad",
    } <= names


def test_world_places_new_objects_in_the_expected_five_zone_layout():
    aruco = _include_pose("aruco_sample")
    cubes = {
        color: _include_pose(f"material_cube_{color}")
        for color in ("red", "green", "blue", "yellow")
    }
    fixture = _include_pose("tooling_fixture_box")
    hand_tools = _include_pose("tooling_hand_tools")
    drill = _include_pose("board_test_fixture")
    probe = _include_pose("high_voltage_probe_kit")
    caliper = _include_pose("material_spare_igbt")
    rack = _include_pose("aging_rack")
    board = _include_pose("pcb_board")
    reagent_rack = _include_pose("reagent_rack")
    bottles = [_include_pose(f"reagent_bottle_{i}") for i in range(1, 5)]
    high_voltage = _include_pose("high_voltage_zone")

    assert _include_uri("aruco_sample") == "model://aruco_sample"
    assert aruco[:3] == pytest.approx([-4.16, 3.46, 0.785])

    # 物料区:主样件 + 4 个彩色物料方块(删除了硅脂罐)
    for color, cube in cubes.items():
        assert (
            _include_uri(f"material_cube_{color}")
            == f"model://material_cube_{color}"
        )
        assert -5.10 <= cube[0] <= -3.50, f"{color} 方块不在物料区 x 范围"
        assert 3.20 <= cube[1] <= 4.40, f"{color} 方块不在物料区 y 范围"
        assert cube[2] == pytest.approx(0.785)
    assert _include_named("material_grease_can") is None, "硅脂罐已删除"

    # 工装工具区:扳手/钳子/钻头/螺丝刀/卡尺 5 件工具集中在一张桌
    tool_x_min, tool_x_max = -4.90, -3.30
    tool_y_min, tool_y_max = -2.90, -1.70
    for tool in (fixture, hand_tools, drill, probe, caliper):
        assert tool_x_min <= tool[0] <= tool_x_max, f"工具 x={tool[0]} 超出工装区"
        assert tool_y_min <= tool[1] <= tool_y_max, f"工具 y={tool[1]} 超出工装区"
    assert probe[2] > 0.70, "螺丝刀应从地面抬到桌面"

    # 板卡测试台(原老化桌):aging_rack 保留 + 新建板卡
    assert rack[:3] == pytest.approx([0.20, 4.26, 0.80])
    assert -0.60 <= board[0] <= 1.00 and 3.60 <= board[1] <= 4.80
    assert board[2] == pytest.approx(0.75)

    # 老化实验台(原板卡桌):试剂架 + 4 个试剂瓶,放在桌后缘
    for obj in [reagent_rack] + bottles:
        assert -0.50 <= obj[0] <= 1.10, f"老化实验台道具 x={obj[0]} 越界"
        assert -2.30 <= obj[1] <= -1.10, f"老化实验台道具 y={obj[1]} 越界"
    for bottle in bottles:
        assert bottle[2] == pytest.approx(0.845)

    # 高压区围栏与地面警示不变
    assert high_voltage[:3] == pytest.approx([4.36, 2.90, 0.0])


def _include_named(entity_name):
    for include in _world_root().findall(".//include"):
        if include.findtext("name") == entity_name:
            return include
    return None


def test_station_b_remains_clear_for_existing_place_task():
    world = _world_root()
    blockers = []
    for include in world.findall(".//include"):
        name = include.findtext("name")
        if name in {
            "aruco_sample",
            "material_cube_red",
            "material_cube_green",
            "material_cube_blue",
            "material_cube_yellow",
            "tooling_fixture_box",
            "tooling_hand_tools",
            "board_test_fixture",
            "high_voltage_probe_kit",
            "material_spare_igbt",
            "aging_rack",
            "pcb_board",
            "reagent_rack",
            "reagent_bottle_1",
            "reagent_bottle_2",
            "reagent_bottle_3",
            "reagent_bottle_4",
            "beaker",
            "erlenmeyer_flask",
            "graduated_cylinder",
            "high_voltage_zone",
        }:
            pose = [float(value) for value in include.findtext("pose").split()]
            if -0.25 <= pose[0] <= 0.55 and -1.15 <= pose[1] <= -0.55:
                blockers.append((name, pose))
    assert not blockers


# ── 新增模型物理可抓取契约 ────────────────────────────────────

def test_material_cubes_are_graspable_with_unique_aruco_ids():
    """彩色物料方块:动态、0.07³、接触面=主样件,ArUco ID 2/3/4/5 且不复用 0/1."""
    ids = {"red": 2, "green": 3, "blue": 4, "yellow": 5}
    cv2 = pytest.importorskip("cv2")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    seen = set()
    for color, marker_id in ids.items():
        model = _model_root(f"material_cube_{color}").find("model")
        assert model.findtext("static") is None, f"{color} 方块必须是动态"
        assert model.findtext(".//link/inertial/mass") is not None
        assert model.findtext(
            ".//collision/geometry/box/size"
        ) == "0.07 0.07 0.07"
        assert model.find(".//collision/surface/contact/ode") is not None
        assert (
            model.find(".//visual[@name='aruco_marker_front']/material/script")
            is not None
        )
        assert (
            model.find(".//visual[@name='aruco_marker_top']/material/script")
            is not None
        )

        tex = (
            GAZEBO / "models" / f"material_cube_{color}"
            / "materials" / "textures" / f"marker_{marker_id}.png"
        )
        assert tex.exists(), f"{color} 缺少 marker_{marker_id}.png"
        img = cv2.imread(str(tex), cv2.IMREAD_GRAYSCALE)
        _, detected_ids, _ = cv2.aruco.detectMarkers(img, dictionary)
        got = None if detected_ids is None else detected_ids.flatten().tolist()
        assert got == [marker_id], f"{color}: 期望 id {marker_id},实际 {got}"
        seen.add(marker_id)
    assert seen == {2, 3, 4, 5}
    assert seen.isdisjoint({0, 1}), "彩色方块不得复用主样件 id 0/1"


def test_reagent_bottle_is_dynamic_and_graspable():
    model = _model_root("reagent_bottle").find("model")
    assert model.findtext("static") is None, "试剂瓶必须是动态"
    assert model.findtext(".//link/inertial/mass") is not None
    assert model.find(".//collision/surface/contact/ode") is not None
    radius = model.findtext(".//collision/geometry/cylinder/radius")
    assert float(radius) <= 0.05, "瓶径必须小于夹爪开度(0.16m)"


def test_reagent_rack_is_static_with_four_cells():
    root = _model_root("reagent_rack")
    assert root.find(".//model").findtext("static") == "true"
    dividers = [
        c for c in root.findall(".//collision")
        if "divider" in (c.attrib.get("name") or "")
    ]
    assert len(dividers) == 3, "试剂架应有三道隔板形成 4 格"
    assert root.find(".//collision[@name='collision_base']") is not None


def test_pcb_board_is_dynamic_and_graspable():
    model = _model_root("pcb_board").find("model")
    assert model.findtext("static") is None, "板卡必须是动态"
    assert model.findtext(".//link/inertial/mass") is not None
    assert model.find(".//collision/surface/contact/ode") is not None
    board = model.find(
        ".//collision[@name='collision_board']/geometry/box/size"
    )
    thickness = float(board.text.split()[1])
    assert thickness <= 0.02, "板厚必须小于夹爪开度(0.16m)"


@pytest.mark.parametrize(
    "model_name",
    ["beaker", "erlenmeyer_flask", "graduated_cylinder"],
)
def test_lab_glassware_props_are_dynamic_and_graspable(model_name):
    """老化实验台玻璃器皿:动态、圆柱碰撞、直径 < 夹爪开度(0.16m)."""
    model = _model_root(model_name).find("model")
    assert model.findtext("static") is None, f"{model_name} 必须是动态"
    assert model.findtext(".//link/inertial/mass") is not None
    assert model.find(".//collision/surface/contact/ode") is not None
    radius = model.findtext(".//collision/geometry/cylinder/radius")
    assert radius is not None, f"{model_name} 缺圆柱碰撞"
    assert float(radius) * 2.0 < 0.16, f"{model_name} 直径须小于夹爪开度"
