"""G5 MoveIt dynamic obstacle injection probe.

This standalone tool applies or removes a box-shaped collision object through
MoveIt's /apply_planning_scene service. It is intentionally outside the main
pick/place path so G5 can validate arm-level obstacle handling without changing
mission behavior.
"""
import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class BoxSpec:
    object_id: str
    frame_id: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]


def validate_box_spec(spec: BoxSpec) -> None:
    if not spec.object_id:
        raise ValueError("object id must not be empty")
    if not spec.frame_id:
        raise ValueError("frame id must not be empty")
    if len(spec.center) != 3:
        raise ValueError("center must have exactly 3 values")
    if len(spec.size) != 3:
        raise ValueError("size must have exactly 3 values")
    if not all(math.isfinite(value) for value in (*spec.center, *spec.size)):
        raise ValueError("center and size must be finite")
    if not all(value > 0.0 for value in spec.size):
        raise ValueError("box size values must be positive")


def ros_time_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def make_box_scene(spec: BoxSpec):
    """Build a PlanningScene diff that adds one box collision object."""
    validate_box_spec(spec)
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(value) for value in spec.size]

    pose = Pose()
    pose.position.x = float(spec.center[0])
    pose.position.y = float(spec.center[1])
    pose.position.z = float(spec.center[2])
    pose.orientation.w = 1.0

    obj = CollisionObject()
    obj.header.frame_id = spec.frame_id
    obj.id = spec.object_id
    obj.operation = CollisionObject.ADD
    obj.primitives = [primitive]
    obj.primitive_poses = [pose]

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [obj]
    return scene


def make_remove_scene(object_id: str):
    """Build a PlanningScene diff that removes one collision object."""
    if not object_id:
        raise ValueError("object id must not be empty")
    from moveit_msgs.msg import CollisionObject, PlanningScene

    obj = CollisionObject()
    obj.id = object_id
    obj.operation = CollisionObject.REMOVE

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [obj]
    return scene


def apply_scene(scene, *, node_name: str = "g5_arm_dynamic_obstacle_probe", timeout_s: float = 5.0) -> bool:
    try:
        import rclpy
        from moveit_msgs.srv import ApplyPlanningScene
    except ImportError as exc:
        raise RuntimeError(
            "Applying a planning scene requires a sourced ROS 2/MoveIt environment"
        ) from exc

    rclpy.init(args=None)
    node = rclpy.create_node(node_name)
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    try:
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            future.cancel()
            return False
        response = future.result()
        return response is not None and bool(response.success)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _parse_triplet(values: list[str], name: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise argparse.ArgumentTypeError(f"{name} requires exactly 3 numbers")
    try:
        return tuple(float(value) for value in values)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} values must be numbers") from exc


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="G5 MoveIt 动态障碍盒注入探针")
    subparsers = parser.add_subparsers(dest="action", required=True)

    add_parser = subparsers.add_parser("add", help="add or update a box collision object")
    add_parser.add_argument("--id", required=True, dest="object_id")
    add_parser.add_argument("--frame", default="base_link", dest="frame_id")
    add_parser.add_argument("--center", nargs=3, required=True, metavar=("X", "Y", "Z"))
    add_parser.add_argument("--size", nargs=3, required=True, metavar=("X", "Y", "Z"))
    add_parser.add_argument("--timeout", type=float, default=5.0)

    remove_parser = subparsers.add_parser("remove", help="remove a collision object")
    remove_parser.add_argument("--id", required=True, dest="object_id")
    remove_parser.add_argument("--timeout", type=float, default=5.0)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.action == "add":
        spec = BoxSpec(
            object_id=args.object_id,
            frame_id=args.frame_id,
            center=_parse_triplet(args.center, "center"),
            size=_parse_triplet(args.size, "size"),
        )
        scene = make_box_scene(spec)
        label = f"添加动态障碍盒 {spec.object_id}"
    else:
        scene = make_remove_scene(args.object_id)
        label = f"移除动态障碍盒 {args.object_id}"

    ok = apply_scene(scene, timeout_s=args.timeout)
    print(f"{label}: {'OK' if ok else 'FAILED'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
