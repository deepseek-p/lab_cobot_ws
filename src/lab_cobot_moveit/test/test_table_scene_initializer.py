"""Tests for the static worktable PlanningScene description."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


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


def test_validation_tooling_table_uses_base_footprint_frame():
    module = _load_module()

    scene = module.build_validation_tooling_table_planning_scene(
        base_x=-3.62,
        base_y=-3.35,
        base_yaw=1.57079632679,
    )

    objects = scene.world.collision_objects
    assert len(objects) == 1
    table = objects[0]
    assert table.id == "tooling_zone_table"
    assert table.header.frame_id == "base_footprint"
    assert list(table.primitives[0].dimensions) == [1.6, 1.2, 0.75]


def test_validation_tooling_table_transform_includes_base_yaw():
    module = _load_module()

    x_base, y_base, yaw_base = module.world_pose_to_base_footprint(
        world_x=-4.10,
        world_y=-2.30,
        world_yaw=0.0,
        base_x=-3.62,
        base_y=-3.35,
        base_yaw=1.57079632679,
    )

    assert x_base == pytest.approx(1.05, abs=1e-6)
    assert y_base == pytest.approx(0.48, abs=1e-6)
    assert yaw_base == pytest.approx(-1.57079632679, abs=1e-6)


def test_aging_validation_registers_real_station_a_and_aging_tables_in_map_frame():
    module = _load_module()

    scene = module.build_aging_rack_insert_validation_planning_scene("map")

    assert scene.is_diff is True
    assert scene.robot_state.is_diff is True
    objects = scene.world.collision_objects
    assert [obj.id for obj in objects] == ["station_a_table", "aging_zone_table"]
    table = objects[0]
    pose = table.primitive_poses[0]
    assert table.id == "station_a_table"
    assert table.header.frame_id == "map"
    assert table.pose.orientation.w == pytest.approx(1.0)
    assert list(table.primitives[0].dimensions) == [1.6, 1.2, 0.75]
    assert pose.position.x == pytest.approx(-4.30)
    assert pose.position.y == pytest.approx(3.80)
    assert pose.position.z == pytest.approx(0.375)
    assert (pose.position.x, pose.position.y, pose.position.z) != (0.0, 0.0, 0.0)
    assert pose.position.z + table.primitives[0].dimensions[2] / 2.0 == pytest.approx(0.75)
    aging_table = objects[1]
    aging_pose = aging_table.primitive_poses[0]
    assert aging_table.id == "aging_zone_table"
    assert aging_table.header.frame_id == "map"
    assert list(aging_table.primitives[0].dimensions) == [1.6, 1.2, 0.75]
    assert aging_pose.position.x == pytest.approx(0.20)
    assert aging_pose.position.y == pytest.approx(4.20)
    assert aging_pose.position.z == pytest.approx(0.375)
    assert aging_pose.position.z + aging_table.primitives[0].dimensions[2] / 2.0 == pytest.approx(0.75)


def test_aging_station_a_table_can_be_transformed_to_base_footprint():
    module = _load_module()

    table = module.build_validation_station_a_table_collision_object(
        frame_id="base_footprint",
        base_x=-4.30,
        base_y=2.745,
        base_yaw=1.57079632679,
    )
    pose = table.primitive_poses[0]

    assert table.id == "station_a_table"
    assert table.header.frame_id == "base_footprint"
    assert table.pose.orientation.w == pytest.approx(1.0)
    assert list(table.primitives[0].dimensions) == [1.6, 1.2, 0.75]
    assert pose.position.x == pytest.approx(1.055, abs=1e-6)
    assert pose.position.y == pytest.approx(0.0, abs=1e-6)
    assert pose.position.z == pytest.approx(0.375)
    assert pose.orientation.z == pytest.approx(-0.70710678, abs=1e-6)
    assert pose.orientation.w == pytest.approx(0.70710678, abs=1e-6)


def test_material_spare_validation_spawn_positions_tooling_table_correctly():
    module = _load_module()

    table = module.build_validation_tooling_table_collision_object(
        base_x=-3.62,
        base_y=-3.35,
        base_yaw=1.57079632679,
    )
    pose = table.primitive_poses[0]

    assert pose.position.x == pytest.approx(1.05, abs=1e-6)
    assert pose.position.y == pytest.approx(0.48, abs=1e-6)
    assert pose.position.z == pytest.approx(0.375)
    assert pose.orientation.z == pytest.approx(-0.70710678, abs=1e-6)
    assert pose.orientation.w == pytest.approx(0.70710678, abs=1e-6)


def test_apply_backs_off_after_a_failed_planning_scene_response(monkeypatch):
    module = _load_module()
    initializer = module.TableSceneInitializer.__new__(module.TableSceneInitializer)
    attempts = []
    sleeps = []

    initializer.get_parameter = lambda name: SimpleNamespace(
        value={
            "world_frame": "map",
            "validation_mode": False,
            "validation_scene_kind": "tooling",
            "robot_spawn_x": 0.0,
            "robot_spawn_y": 0.0,
            "robot_spawn_yaw": 0.0,
            "max_attempts": 3,
            "retry_delay": 0.5,
        }[name]
    )
    initializer.get_logger = lambda: SimpleNamespace(
        error=lambda _message: None,
        info=lambda _message: None,
        warning=lambda _message: None,
    )

    class FakeClient:
        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, _request):
            attempts.append(True)
            return SimpleNamespace(
                done=lambda: True,
                exception=lambda: None,
                result=lambda: SimpleNamespace(success=len(attempts) == 3),
            )

    initializer._client = FakeClient()
    monkeypatch.setattr(module.rclpy, "spin_until_future_complete", lambda *args, **kwargs: None)
    monkeypatch.setattr(module.time, "sleep", sleeps.append)

    assert initializer.apply()
    assert len(attempts) == 3
    assert sleeps == [0.5, 0.5]
