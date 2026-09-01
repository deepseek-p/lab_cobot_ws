#!/usr/bin/env python3
"""Register frozen Gazebo worktables in MoveIt's PlanningScene."""

import math
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from lab_cobot_moveit.tube_insert_scene import build_tube_insert_planning_scene


TABLES = (
    ("station_a_table", -2.15, 1.9),
    ("tooling_zone_table", -2.05, -1.15),
    ("aging_zone_table", 0.1, 2.1),
    ("station_b_table", 0.15, -0.85),
)
TABLE_SIZE = (0.8, 0.6, 0.75)
TABLE_CENTER_Z = 0.375
TOOLING_ZONE_TABLE_WORLD = {
    "id": "tooling_zone_table",
    "x": -4.10,
    "y": -2.30,
    "z": 0.375,
    "yaw": 0.0,
    "size": (1.6, 1.2, 0.75),
}
STATION_A_TABLE_WORLD = {
    "id": "station_a_table",
    "x": -4.30,
    "y": 3.80,
    "z": 0.375,
    "yaw": 0.0,
    "size": (1.6, 1.2, 0.75),
}
AGING_ZONE_TABLE_WORLD = {
    "id": "aging_zone_table",
    "x": 0.20,
    "y": 4.20,
    "z": 0.375,
    "yaw": 0.0,
    "size": (1.6, 1.2, 0.75),
}


def yaw_to_quat(yaw):
    half = float(yaw) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def world_pose_to_base_footprint(world_x, world_y, world_yaw, base_x, base_y, base_yaw):
    """Transform a world/map planar pose into base_footprint coordinates."""
    dx = float(world_x) - float(base_x)
    dy = float(world_y) - float(base_y)
    yaw = float(base_yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_base = cos_yaw * dx + sin_yaw * dy
    y_base = -sin_yaw * dx + cos_yaw * dy
    yaw_base = float(world_yaw) - yaw
    yaw_base = math.atan2(math.sin(yaw_base), math.cos(yaw_base))
    return x_base, y_base, yaw_base


def build_table_collision_objects(frame_id):
    """Build idempotent ADD messages matching the Gazebo table geometry."""
    objects = []
    for object_id, x, y in TABLES:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(TABLE_SIZE)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = TABLE_CENTER_Z
        pose.orientation.w = 1.0

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = object_id
        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [pose]
        collision_object.operation = CollisionObject.ADD
        objects.append(collision_object)
    return objects


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

    collision_object = CollisionObject()
    collision_object.header.frame_id = str(frame_id)
    collision_object.id = str(object_id)
    collision_object.pose.orientation.w = 1.0
    collision_object.primitives = [primitive]
    collision_object.primitive_poses = [pose]
    collision_object.operation = CollisionObject.ADD
    return collision_object


def build_validation_tooling_table_collision_object(base_x, base_y, base_yaw):
    x_base, y_base, yaw_base = world_pose_to_base_footprint(
        TOOLING_ZONE_TABLE_WORLD["x"],
        TOOLING_ZONE_TABLE_WORLD["y"],
        TOOLING_ZONE_TABLE_WORLD["yaw"],
        base_x,
        base_y,
        base_yaw,
    )
    return build_box_collision_object(
        TOOLING_ZONE_TABLE_WORLD["id"],
        "base_footprint",
        (x_base, y_base, TOOLING_ZONE_TABLE_WORLD["z"]),
        TOOLING_ZONE_TABLE_WORLD["size"],
        yaw=yaw_base,
    )


def build_validation_station_a_table_collision_object(
    frame_id="map",
    base_x=None,
    base_y=None,
    base_yaw=None,
):
    if frame_id in ("base_footprint", "base_link"):
        if base_x is None or base_y is None or base_yaw is None:
            raise ValueError("base pose is required for base-frame station_a_table")
        x, y, yaw = world_pose_to_base_footprint(
            STATION_A_TABLE_WORLD["x"],
            STATION_A_TABLE_WORLD["y"],
            STATION_A_TABLE_WORLD["yaw"],
            base_x,
            base_y,
            base_yaw,
        )
    else:
        x = STATION_A_TABLE_WORLD["x"]
        y = STATION_A_TABLE_WORLD["y"]
        yaw = STATION_A_TABLE_WORLD["yaw"]
    return build_box_collision_object(
        STATION_A_TABLE_WORLD["id"],
        frame_id,
        (x, y, STATION_A_TABLE_WORLD["z"]),
        STATION_A_TABLE_WORLD["size"],
        yaw=yaw,
    )


def build_validation_aging_zone_table_collision_object(frame_id="map"):
    return build_box_collision_object(
        AGING_ZONE_TABLE_WORLD["id"],
        frame_id,
        (
            AGING_ZONE_TABLE_WORLD["x"],
            AGING_ZONE_TABLE_WORLD["y"],
            AGING_ZONE_TABLE_WORLD["z"],
        ),
        AGING_ZONE_TABLE_WORLD["size"],
        yaw=AGING_ZONE_TABLE_WORLD["yaw"],
    )


def build_table_planning_scene(frame_id):
    """Build a PlanningScene diff; repeating it replaces objects with the same IDs."""
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.collision_objects = build_table_collision_objects(frame_id)
    return scene


def build_validation_tooling_table_planning_scene(base_x, base_y, base_yaw):
    """Build a validation-only scene with tooling table in base_footprint."""
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.collision_objects = [
        build_validation_tooling_table_collision_object(base_x, base_y, base_yaw)
    ]
    return scene


def build_tube_insert_validation_planning_scene(base_x, base_y, base_yaw):
    """Build station_b + tube racks + tubes scene for tube insert validation."""
    return build_tube_insert_planning_scene(base_x, base_y, base_yaw)


def build_aging_rack_insert_validation_planning_scene(
    frame_id="map",
    base_x=None,
    base_y=None,
    base_yaw=None,
):
    """Build aging validation scene with the real station_a and aging tables."""
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.collision_objects = [
        build_validation_station_a_table_collision_object(
            frame_id=frame_id,
            base_x=base_x,
            base_y=base_y,
            base_yaw=base_yaw,
        ),
        build_validation_aging_zone_table_collision_object(frame_id=frame_id),
    ]
    return scene


class TableSceneInitializer(Node):
    """Apply static worktable geometry once move_group exposes its service."""

    def __init__(self):
        super().__init__("table_scene_initializer")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("validation_mode", False)
        self.declare_parameter("validation_scene_kind", "tooling")
        self.declare_parameter("robot_spawn_x", 0.0)
        self.declare_parameter("robot_spawn_y", 0.0)
        self.declare_parameter("robot_spawn_yaw", 0.0)
        self.declare_parameter("max_attempts", 120)
        self.declare_parameter("retry_delay", 0.5)
        self._client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

    def apply(self):
        frame_id = self.get_parameter("world_frame").value
        validation_mode = bool(self.get_parameter("validation_mode").value)
        validation_scene_kind = str(
            self.get_parameter("validation_scene_kind").value
        )
        max_attempts = int(self.get_parameter("max_attempts").value)
        retry_delay = float(self.get_parameter("retry_delay").value)
        if not frame_id or max_attempts <= 0 or retry_delay <= 0.0:
            self.get_logger().error("Invalid PlanningScene initializer parameters")
            return False

        request = ApplyPlanningScene.Request()
        if validation_mode:
            base_x = float(self.get_parameter("robot_spawn_x").value)
            base_y = float(self.get_parameter("robot_spawn_y").value)
            base_yaw = float(self.get_parameter("robot_spawn_yaw").value)
            if validation_scene_kind == "tube_insert":
                request.scene = build_tube_insert_validation_planning_scene(
                    base_x,
                    base_y,
                    base_yaw,
                )
                object_ids = [
                    obj.id for obj in request.scene.world.collision_objects
                ]
                success_message = (
                    "tube insert station_b scene registered in PlanningScene "
                    "frame base_link objects=%s"
                    % ",".join(object_ids)
                )
            elif validation_scene_kind == "aging_rack_insert":
                request.scene = build_aging_rack_insert_validation_planning_scene(
                    frame_id=frame_id,
                    base_x=base_x,
                    base_y=base_y,
                    base_yaw=base_yaw,
                )
                objects = request.scene.world.collision_objects
                object_ids = [obj.id for obj in objects]
                success_message = (
                    "aging rack validation tables registered in PlanningScene "
                    "frame %s objects=%s"
                    % (
                        frame_id,
                        ",".join(object_ids),
                    )
                )
            else:
                request.scene = build_validation_tooling_table_planning_scene(
                    base_x,
                    base_y,
                    base_yaw,
                )
                success_message = (
                    "tooling_zone table registered in PlanningScene frame "
                    "base_footprint"
                )
        else:
            request.scene = build_table_planning_scene(frame_id)
            success_message = (
                "Registered %d frozen worktables in PlanningScene frame %s"
                % (len(TABLES), frame_id)
            )
        for attempt in range(1, max_attempts + 1):
            if not self._client.wait_for_service(timeout_sec=retry_delay):
                self.get_logger().info(
                    f"Waiting for /apply_planning_scene ({attempt}/{max_attempts})"
                )
                continue
            future = self._client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=retry_delay)
            if future.done() and future.exception() is None:
                response = future.result()
                if response is not None and response.success:
                    self.get_logger().info(success_message)
                    return True
            self.get_logger().warning(
                f"PlanningScene apply attempt failed ({attempt}/{max_attempts})"
            )
            time.sleep(retry_delay)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = TableSceneInitializer()
    try:
        if not node.apply():
            raise RuntimeError("Failed to register worktables in PlanningScene")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
