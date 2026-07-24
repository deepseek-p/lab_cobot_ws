"""Unit tests for planning scene obstacle geometry helpers."""
# 中文说明:台面碰撞盒/持物样件附着盒的几何推导回归测试。
# 场景事实来源 lab.world:台面 0.8x0.6x0.75、样件 0.07 立方、
# 停靠点距台前缘 0.58m(waypoints station_a y=0.62,台前缘 y=1.2)。
import math

from lab_cobot_manipulation.scene_obstacles import (
    ATTACHED_SAMPLE_BOTTOM_TRIM,
    CARRIED_SAMPLE_BOX_ID,
    DYNAMIC_ARM_OBSTACLE_BOX_ID,
    GRIPPER_ATTACH_LINK,
    GRIPPER_TOUCH_LINKS,
    HELD_SAMPLE_CENTER_FROM_TCP_Z,
    SAMPLE_HALF_HEIGHT,
    STATION_SURFACE_BOX_ID,
    SURFACE_BOX_XY,
    TABLE_HEIGHT,
    carried_sample_box,
    dynamic_obstacle_box,
    make_attach_scene,
    make_detach_scene,
    make_dynamic_obstacle_scene,
    make_remove_dynamic_obstacle_scene,
    make_world_box_scene,
    station_surface_box,
)

# world 事实:样件中心世界 z=0.785,base_link 离地 0.155(实测标定,
# 与 test_mission_place_pose.py 同源),故 base_link 系检测 z=0.63。
DETECTED_POS = [0.88, 0.0, 0.63]
BASE_LINK_WORLD_Z = 0.155
TABLE_TOP_WORLD_Z = 0.75
# 停靠几何:车头半长 0.28(nav2 footprint),名义车心-样件距离 0.88。
FOOTPRINT_HALF_LENGTH = 0.28
DOCK_XY_TOLERANCE = 0.12


def test_surface_box_top_matches_world_table_height():
    box = station_surface_box(DETECTED_POS)
    top_z = box["center"][2] + box["size"][2] / 2.0
    expected_top = TABLE_TOP_WORLD_Z - BASE_LINK_WORLD_Z
    assert math.isclose(top_z, expected_top, abs_tol=1e-9)
    assert math.isclose(box["size"][2], TABLE_HEIGHT, abs_tol=1e-9)


def test_surface_box_centered_on_detection_xy():
    box = station_surface_box(DETECTED_POS)
    assert math.isclose(box["center"][0], DETECTED_POS[0], abs_tol=1e-9)
    assert math.isclose(box["center"][1], DETECTED_POS[1], abs_tol=1e-9)


def test_surface_box_ignores_input_z():
    # 台顶是常量(两站同高,base_link 离地恒定);检测 z 有误差,
    # 放置点 z 又是 TCP 语义——盒 z 一律不得从输入推。
    lifted = station_surface_box([0.88, 0.0, 0.99])
    nominal = station_surface_box(DETECTED_POS)
    assert lifted == nominal


def test_surface_box_covers_rotated_table_envelope():
    # 停靠 yaw 容差 0.25rad 下,0.8x0.6 台面的轴对齐包络需求:
    # w*cos(yaw)+h*sin(yaw);方盒边长必须覆盖再留检测误差余量。
    yaw_tol = 0.25
    envelope = 0.8 * math.cos(yaw_tol) + 0.6 * math.sin(yaw_tol)
    assert SURFACE_BOX_XY >= envelope + 0.01


def test_surface_box_front_edge_clears_docked_footprint():
    # 预算锁:盒前缘(朝车侧)必须留出车头半长+停靠容差,
    # 否则起始位形陷入碰撞盒,所有规划直接失败。
    box = station_surface_box(DETECTED_POS)
    front_edge_x = box["center"][0] - box["size"][0] / 2.0
    assert front_edge_x > FOOTPRINT_HALF_LENGTH + DOCK_XY_TOLERANCE


def test_carried_sample_box_bottom_trimmed():
    # 附着盒底部上收:lift 起点样件底与台面零距,不收则首个
    # 笛卡尔路径点即报碰撞;收量与悬空释放余量同源(0.02)。
    box = carried_sample_box()
    true_center_z = HELD_SAMPLE_CENTER_FROM_TCP_Z
    true_bottom = true_center_z - SAMPLE_HALF_HEIGHT
    box_bottom = box["center"][2] - box["size"][2] / 2.0
    assert math.isclose(
        box_bottom - true_bottom, ATTACHED_SAMPLE_BOTTOM_TRIM, abs_tol=1e-9
    )
    # 顶部不动:仍与样件真实顶面一致。
    true_top = true_center_z + SAMPLE_HALF_HEIGHT
    box_top = box["center"][2] + box["size"][2] / 2.0
    assert math.isclose(box_top, true_top, abs_tol=1e-9)


def test_touch_links_cover_both_fingers():
    assert "gripper_left_finger" in GRIPPER_TOUCH_LINKS
    assert "gripper_right_finger" in GRIPPER_TOUCH_LINKS
    assert GRIPPER_ATTACH_LINK == "gripper_tcp"


def test_make_world_box_scene_is_add_diff():
    box = station_surface_box(DETECTED_POS)
    scene = make_world_box_scene(
        STATION_SURFACE_BOX_ID, box, frame_id="base_link"
    )
    assert scene.is_diff is True
    assert len(scene.world.collision_objects) == 1
    obj = scene.world.collision_objects[0]
    assert obj.id == STATION_SURFACE_BOX_ID
    assert obj.operation == obj.ADD
    assert obj.header.frame_id == "base_link"
    assert list(obj.primitives[0].dimensions) == list(box["size"])
    assert obj.primitive_poses[0].position.z == box["center"][2]


def test_dynamic_obstacle_box_preserves_center_and_size():
    box = dynamic_obstacle_box([0.35, 0.12, 0.5], [0.12, 0.12, 0.2])

    assert box["center"] == [0.35, 0.12, 0.5]
    assert box["size"] == [0.12, 0.12, 0.2]


def test_dynamic_obstacle_box_rejects_non_positive_size():
    try:
        dynamic_obstacle_box([0.35, 0.12, 0.5], [0.12, 0.0, 0.2])
    except ValueError as exc:
        assert "positive" in str(exc)
    else:
        raise AssertionError("dynamic_obstacle_box should reject zero size")


def test_make_dynamic_obstacle_scene_is_add_or_update_diff():
    scene = make_dynamic_obstacle_scene(
        [0.35, 0.12, 0.5],
        [0.12, 0.12, 0.2],
        frame_id="base_link",
    )

    assert scene.is_diff is True
    obj = scene.world.collision_objects[0]
    assert obj.id == DYNAMIC_ARM_OBSTACLE_BOX_ID
    assert obj.operation == obj.ADD
    assert obj.header.frame_id == "base_link"
    assert list(obj.primitives[0].dimensions) == [0.12, 0.12, 0.2]
    assert obj.primitive_poses[0].position.x == 0.35


def test_make_remove_dynamic_obstacle_scene_is_remove_diff():
    scene = make_remove_dynamic_obstacle_scene()

    assert scene.is_diff is True
    obj = scene.world.collision_objects[0]
    assert obj.id == DYNAMIC_ARM_OBSTACLE_BOX_ID
    assert obj.operation == obj.REMOVE


def test_make_attach_scene_binds_to_tcp_with_touch_links():
    scene = make_attach_scene(CARRIED_SAMPLE_BOX_ID)
    assert scene.is_diff is True
    assert scene.robot_state.is_diff is True
    attached = scene.robot_state.attached_collision_objects
    assert len(attached) == 1
    aco = attached[0]
    assert aco.link_name == GRIPPER_ATTACH_LINK
    assert aco.object.header.frame_id == GRIPPER_ATTACH_LINK
    assert aco.object.id == CARRIED_SAMPLE_BOX_ID
    assert aco.object.operation == aco.object.ADD
    assert set(GRIPPER_TOUCH_LINKS).issubset(set(aco.touch_links))


def test_make_detach_scene_removes_attached_and_world_copy():
    # detach 语义:attached REMOVE 会把物体放回 world,
    # 必须在同一 diff 里同时 REMOVE world 副本才算干净移除。
    scene = make_detach_scene(CARRIED_SAMPLE_BOX_ID)
    assert scene.is_diff is True
    attached = scene.robot_state.attached_collision_objects
    assert len(attached) == 1
    assert attached[0].object.id == CARRIED_SAMPLE_BOX_ID
    assert attached[0].object.operation == attached[0].object.REMOVE
    world_objs = scene.world.collision_objects
    assert len(world_objs) == 1
    assert world_objs[0].id == CARRIED_SAMPLE_BOX_ID
    assert world_objs[0].operation == world_objs[0].REMOVE
