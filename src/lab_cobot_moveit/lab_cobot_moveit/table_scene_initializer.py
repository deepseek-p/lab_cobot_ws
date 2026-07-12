#!/usr/bin/env python3
"""Register the two fixed Gazebo worktables in MoveIt's PlanningScene."""

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


TABLES = (
    ("station_a_table", 2.0, 1.5),
    ("station_b_table", -2.0, 1.5),
)
TABLE_SIZE = (0.8, 0.6, 0.75)
TABLE_CENTER_Z = 0.375


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


def build_table_planning_scene(frame_id):
    """Build a PlanningScene diff; repeating it replaces objects with the same IDs."""
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.collision_objects = build_table_collision_objects(frame_id)
    return scene


class TableSceneInitializer(Node):
    """Apply static worktable geometry once move_group exposes its service."""

    def __init__(self):
        super().__init__("table_scene_initializer")
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("max_attempts", 30)
        self.declare_parameter("retry_delay", 1.0)
        self._client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

    def apply(self):
        frame_id = self.get_parameter("world_frame").value
        max_attempts = int(self.get_parameter("max_attempts").value)
        retry_delay = float(self.get_parameter("retry_delay").value)
        if not frame_id or max_attempts <= 0 or retry_delay <= 0.0:
            self.get_logger().error("Invalid PlanningScene initializer parameters")
            return False

        request = ApplyPlanningScene.Request()
        request.scene = build_table_planning_scene(frame_id)
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
                    self.get_logger().info(
                        "Registered station_a_table and station_b_table in "
                        f"PlanningScene frame {frame_id}"
                    )
                    return True
            self.get_logger().warning(
                f"PlanningScene apply attempt failed ({attempt}/{max_attempts})"
            )
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
