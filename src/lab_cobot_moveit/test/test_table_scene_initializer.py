"""Tests for the static worktable PlanningScene description."""
import importlib.util
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE / "lab_cobot_moveit" / "table_scene_initializer.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("table_scene_initializer", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_table_collision_objects_match_gazebo_world():
    module = _load_module()

    objects = module.build_table_collision_objects("map")

    assert [obj.id for obj in objects] == [
        "station_a_table",
        "tooling_zone_table",
        "aging_zone_table",
        "station_b_table",
    ]
    assert [obj.header.frame_id for obj in objects] == ["map"] * 4
    assert [list(obj.primitives[0].dimensions) for obj in objects] == [[0.8, 0.6, 0.75]] * 4
    assert [obj.primitive_poses[0].position.x for obj in objects] == [-2.15, -2.05, 0.1, 0.15]
    assert [obj.primitive_poses[0].position.y for obj in objects] == [1.9, -1.15, 2.1, -0.85]
    assert [obj.primitive_poses[0].position.z for obj in objects] == [0.375] * 4
    assert all(obj.primitive_poses[0].orientation.w == 1.0 for obj in objects)
    assert all(obj.operation == obj.ADD for obj in objects)


def test_build_planning_scene_is_an_idempotent_world_diff():
    module = _load_module()

    scene = module.build_table_planning_scene("map")

    assert scene.is_diff is True
    assert scene.robot_state.is_diff is True
    assert [obj.id for obj in scene.world.collision_objects] == [
        "station_a_table",
        "tooling_zone_table",
        "aging_zone_table",
        "station_b_table",
    ]
