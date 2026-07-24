"""Unit tests for the dynamic arm obstacle PlanningScene bridge."""

import pytest

from lab_cobot_manipulation import dynamic_arm_obstacle_node as node_mod


class FakeLogger:
    def __init__(self, events):
        self._events = events

    def info(self, message):
        self._events.append(f"info:{message}")

    def warn(self, message):
        self._events.append(f"warn:{message}")


class FakeSceneClient:
    def __init__(self, events, ok=True):
        self.events = events
        self.ok = ok

    def apply(self, scene, **_kwargs):
        if scene.world.collision_objects:
            obj = scene.world.collision_objects[0]
            if obj.operation == obj.ADD:
                self.events.append(
                    (
                        "update",
                        obj.id,
                        obj.header.frame_id,
                        list(obj.primitives[0].dimensions),
                    )
                )
            else:
                self.events.append(("remove", obj.id))
        return self.ok


class FakeMsg:
    def __init__(self, data):
        self.data = data


def make_node(events, ok=True):
    node = node_mod.DynamicArmObstacleNode.__new__(
        node_mod.DynamicArmObstacleNode
    )
    node.scene_client = FakeSceneClient(events, ok=ok)
    node.frame_id = "base_link"
    node.object_id = node_mod.DYNAMIC_ARM_OBSTACLE_BOX_ID
    node.service_timeout_sec = 5.0
    node.get_logger = lambda: FakeLogger(events)
    return node


def test_parse_obstacle_box_splits_center_and_size():
    center, size = node_mod.parse_obstacle_box([
        0.35,
        0.12,
        0.5,
        0.12,
        0.12,
        0.2,
    ])

    assert center == [0.35, 0.12, 0.5]
    assert size == [0.12, 0.12, 0.2]


def test_parse_obstacle_box_rejects_wrong_length():
    with pytest.raises(ValueError, match="6 values"):
        node_mod.parse_obstacle_box([0.35, 0.12, 0.5])


def test_parse_obstacle_box_rejects_non_positive_size():
    with pytest.raises(ValueError, match="positive"):
        node_mod.parse_obstacle_box([0.35, 0.12, 0.5, 0.12, 0.0, 0.2])


def test_obstacle_callback_updates_scene():
    events = []
    node = make_node(events)

    node._on_obstacle_box(FakeMsg([0.35, 0.12, 0.5, 0.12, 0.12, 0.2]))

    assert events[0] == (
        "update",
        node_mod.DYNAMIC_ARM_OBSTACLE_BOX_ID,
        "base_link",
        [0.12, 0.12, 0.2],
    )
    assert any("dynamic arm obstacle updated" in event for event in events)


def test_empty_obstacle_callback_removes_scene_object():
    events = []
    node = make_node(events)

    node._on_obstacle_box(FakeMsg([]))

    assert events[0] == ("remove", node_mod.DYNAMIC_ARM_OBSTACLE_BOX_ID)
    assert any("dynamic arm obstacle removed" in event for event in events)


def test_invalid_obstacle_callback_warns_without_apply():
    events = []
    node = make_node(events)

    node._on_obstacle_box(FakeMsg([0.35, 0.12, 0.5]))

    assert events == ["warn:dynamic obstacle command must contain 6 values"]
