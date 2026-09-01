"""PlanningScene geometry for the station_b tube insertion validation task."""
from __future__ import annotations

import math

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from shape_msgs.msg import SolidPrimitive


BASE_LINK_WORLD_Z = 0.155
STATION_B_TABLE_WORLD = {
    "id": "station_b_table",
    "x": 0.30,
    "y": -1.70,
    "z": 0.375,
    "yaw": 0.0,
    "size": (1.6, 1.2, 0.75),
}
TEST_TUBE_RACK_1_WORLD = {
    "id": "test_tube_rack_1",
    "x": 0.48,
    "y": -1.95,
    "z": 0.75,
    "yaw": 0.0,
}
TEST_TUBE_RACK_2_WORLD = {
    "id": "test_tube_rack_2",
    "x": 0.12,
    "y": -1.95,
    "z": 0.75,
    "yaw": 0.0,
}
TEST_TUBE_WORLD_OBJECTS = (
    ("test_tube_1", 0.36, -1.95, 0.762),
    ("test_tube_2", 0.42, -1.95, 0.762),
    ("test_tube_3", 0.48, -1.95, 0.762),
    ("test_tube_4", 0.54, -1.95, 0.762),
    ("test_tube_5", 0.60, -1.95, 0.762),
    ("test_tube_6", 0.00, -1.95, 0.762),
    ("test_tube_7", 0.06, -1.95, 0.762),
    # rack2 middle slot intentionally empty
    ("test_tube_8", 0.18, -1.95, 0.762),
    ("test_tube_9", 0.24, -1.95, 0.762),
)
TEST_TUBE_RADIUS = 0.011
TEST_TUBE_LENGTH = 0.125
TEST_TUBE_GRASP_HEIGHT = 0.083
TEST_TUBE_ATTACHED_ID = "test_tube_1"
TEST_TUBE_ATTACH_LINK = "gripper_tcp"
TEST_TUBE_TOUCH_LINKS = (
    "gripper_left_finger",
    "gripper_right_finger",
    "gripper_base",
)

# Direct transcription of src/lab_cobot_gazebo/models/test_tube_rack/model.sdf
# collision boxes.  There is no solid block over the center slot; the insertion
# channel is the space left between front/back walls and divider boxes.
RACK_COLLISION_BOXES = (
    ("base", (0.0, 0.0, 0.006), (0.31, 0.10, 0.012)),
    ("wall_front", (0.0, 0.036, 0.037), (0.31, 0.028, 0.050)),
    ("wall_back", (0.0, -0.036, 0.037), (0.31, 0.028, 0.050)),
    ("divider_1", (-0.09, 0.0, 0.037), (0.020, 0.044, 0.050)),
    ("divider_2", (-0.03, 0.0, 0.037), (0.020, 0.044, 0.050)),
    ("divider_3", (0.03, 0.0, 0.037), (0.020, 0.044, 0.050)),
    ("divider_4", (0.09, 0.0, 0.037), (0.020, 0.044, 0.050)),
    ("endcap_1", (-0.145, 0.0, 0.037), (0.020, 0.044, 0.050)),
    ("endcap_2", (0.145, 0.0, 0.037), (0.020, 0.044, 0.050)),
)


def yaw_to_quat(yaw):
    half = float(yaw) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def rotate_xy(x, y, yaw):
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    return c * float(x) - s * float(y), s * float(x) + c * float(y)


def world_pose_to_base_link(world_x, world_y, world_z, world_yaw, base_x, base_y, base_yaw):
    dx = float(world_x) - float(base_x)
    dy = float(world_y) - float(base_y)
    yaw = float(base_yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_base = cos_yaw * dx + sin_yaw * dy
    y_base = -sin_yaw * dx + cos_yaw * dy
    yaw_base = float(world_yaw) - yaw
    yaw_base = math.atan2(math.sin(yaw_base), math.cos(yaw_base))
    return x_base, y_base, float(world_z) - BASE_LINK_WORLD_Z, yaw_base


def build_box_collision_object(object_id, frame_id, center, size, yaw=0.0):
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(value) for value in size]

    pose = Pose()
    pose.position.x = float(center[0])
    pose.position.y = float(center[1])
    pose.position.z = float(center[2])
    qx, qy, qz, qw = yaw_to_quat(yaw)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

    obj = CollisionObject()
    obj.header.frame_id = str(frame_id)
    obj.id = str(object_id)
    obj.primitives = [primitive]
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


def build_cylinder_collision_object(object_id, frame_id, center, radius, length):
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.CYLINDER
    primitive.dimensions = [float(length), float(radius)]

    pose = Pose()
    pose.position.x = float(center[0])
    pose.position.y = float(center[1])
    pose.position.z = float(center[2])
    pose.orientation.w = 1.0

    obj = CollisionObject()
    obj.header.frame_id = str(frame_id)
    obj.id = str(object_id)
    obj.primitives = [primitive]
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


def remove_collision_object(object_id):
    obj = CollisionObject()
    obj.id = str(object_id)
    obj.operation = CollisionObject.REMOVE
    return obj


def station_b_table_object(base_x, base_y, base_yaw, frame_id="base_link"):
    x, y, z, yaw = world_pose_to_base_link(
        STATION_B_TABLE_WORLD["x"],
        STATION_B_TABLE_WORLD["y"],
        STATION_B_TABLE_WORLD["z"],
        STATION_B_TABLE_WORLD["yaw"],
        base_x,
        base_y,
        base_yaw,
    )
    return build_box_collision_object(
        STATION_B_TABLE_WORLD["id"],
        frame_id,
        (x, y, z),
        STATION_B_TABLE_WORLD["size"],
        yaw=yaw,
    )


def rack_collision_objects(rack, base_x, base_y, base_yaw, frame_id="base_link"):
    objects = []
    for suffix, local_center, size in RACK_COLLISION_BOXES:
        lx, ly = rotate_xy(local_center[0], local_center[1], rack["yaw"])
        x, y, z, yaw = world_pose_to_base_link(
            rack["x"] + lx,
            rack["y"] + ly,
            rack["z"] + local_center[2],
            rack["yaw"],
            base_x,
            base_y,
            base_yaw,
        )
        objects.append(
            build_box_collision_object(
                "%s_%s" % (rack["id"], suffix),
                frame_id,
                (x, y, z),
                size,
                yaw=yaw,
            )
        )
    return objects


def test_tube_world_objects(base_x, base_y, base_yaw, frame_id="base_link"):
    objects = []
    for object_id, world_x, world_y, bottom_z in TEST_TUBE_WORLD_OBJECTS:
        x, y, z, _yaw = world_pose_to_base_link(
            world_x,
            world_y,
            bottom_z + TEST_TUBE_LENGTH / 2.0,
            0.0,
            base_x,
            base_y,
            base_yaw,
        )
        objects.append(
            build_cylinder_collision_object(
                object_id,
                frame_id,
                (x, y, z),
                TEST_TUBE_RADIUS,
                TEST_TUBE_LENGTH,
            )
        )
    return objects


def build_tube_insert_collision_objects(
    base_x,
    base_y,
    base_yaw,
    frame_id="base_link",
    include_tubes=True,
):
    objects = [station_b_table_object(base_x, base_y, base_yaw, frame_id)]
    objects.extend(
        rack_collision_objects(
            TEST_TUBE_RACK_1_WORLD,
            base_x,
            base_y,
            base_yaw,
            frame_id,
        )
    )
    objects.extend(
        rack_collision_objects(
            TEST_TUBE_RACK_2_WORLD,
            base_x,
            base_y,
            base_yaw,
            frame_id,
        )
    )
    if include_tubes:
        objects.extend(test_tube_world_objects(base_x, base_y, base_yaw, frame_id))
    return objects


def build_tube_insert_planning_scene(
    base_x,
    base_y,
    base_yaw,
    frame_id="base_link",
    include_tubes=True,
):
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.collision_objects = build_tube_insert_collision_objects(
        base_x,
        base_y,
        base_yaw,
        frame_id=frame_id,
        include_tubes=include_tubes,
    )
    return scene


def make_attach_test_tube_scene(
    object_id=TEST_TUBE_ATTACHED_ID,
    attach_link=TEST_TUBE_ATTACH_LINK,
    remove_world_object=True,
):
    attached = AttachedCollisionObject()
    attached.link_name = attach_link
    attached.touch_links = list(TEST_TUBE_TOUCH_LINKS)
    attached.object = build_cylinder_collision_object(
        object_id,
        attach_link,
        (0.0, 0.0, TEST_TUBE_GRASP_HEIGHT - TEST_TUBE_LENGTH / 2.0),
        TEST_TUBE_RADIUS,
        TEST_TUBE_LENGTH,
    )
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [attached]
    if remove_world_object:
        scene.world.collision_objects = [remove_collision_object(object_id)]
    return scene


def make_detach_test_tube_scene(
    object_id=TEST_TUBE_ATTACHED_ID,
    remove_world_object=True,
):
    attached = AttachedCollisionObject()
    attached.link_name = TEST_TUBE_ATTACH_LINK
    attached.object = remove_collision_object(object_id)
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [attached]
    if remove_world_object:
        scene.world.collision_objects = [remove_collision_object(object_id)]
    return scene
