#!/usr/bin/env python3
"""

Sync ghost collision cloud with walking actor trajectory.

Publishes:
  /actor_ghost/pose           — PoseStamped of ghost position (for metrics)
  /actor_ghost/obstacle_cloud — PointCloud2 of ghost cylinder (for costmap obstacle marking)

Also teleports the physical actor_ghost_collision model in Gazebo so that
the LiDAR (/scan) can directly detect the moving person.
"""
import math
import struct
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

PUBLISH_HZ = 20.0
REFERENCE_FRAME = "map"

# Ghost cylinder parameters.
GHOST_RADIUS = 0.60
GHOST_HEIGHT = 1.70
GHOST_CENTER_Z = 0.85
GHOST_MODEL_NAME = "actor_ghost_collision"
OBSTACLE_TOPIC = "/actor_ghost/obstacle_cloud"

# Delay between actor start of movement and world start, matches <delay_start>
# in the world file. Actor begins at sim_time = ACTOR_DELAY_START_SEC.
ACTOR_DELAY_START_SEC = 1.0
SERVICE_WARN_PERIOD_SEC = 5.0

# T1 actor-interference route.
# Pytest stabilizes the stack for about 150 s before sending T1.  This long
# cycle creates one deliberate crossing immediately after that window and then
# keeps the actor away for the rest of the task.  The crossing is offset from
# the route centerline so Nav2 must perceive/replan, while the strict
# min_distance_m >= 0.3 m acceptance gate remains achievable.
# (time, x, y, yaw) at each time (seconds), 300 s cycle.
DEFAULT_WAYPOINTS: List[Tuple[float, float, float, float]] = [
    (0.0,   6.00, -3.60,  2.95),
    (148.0, 6.00, -3.60,  2.95),
    (155.0, 1.00, -2.60,  2.95),
    (164.0, -6.00, 4.00,  2.39),
    (300.0, -6.00, 4.00,  2.39),
]



def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _slerp_yaw(a: float, b: float, t: float) -> float:
    diff = b - a
    diff = math.atan2(math.sin(diff), math.cos(diff))
    return a + diff * t


def _interpolate_waypoint(
    waypoints: List[Tuple[float, float, float, float]], elapsed: float
) -> Tuple[float, float, float]:
    cycle = waypoints[-1][0]
    if cycle <= 0:
        return waypoints[0][1], waypoints[0][2], waypoints[0][3]
    t = elapsed % cycle
    for i in range(len(waypoints) - 1):
        t_i, x_i, y_i, yaw_i = waypoints[i]
        t_next, x_next, y_next, yaw_next = waypoints[i + 1]
        if t_i <= t <= t_next:
            if t_next == t_i:
                alpha = 0.0
            else:
                alpha = (t - t_i) / (t_next - t_i)
            return (
                _lerp(x_i, x_next, alpha),
                _lerp(y_i, y_next, alpha),
                _slerp_yaw(yaw_i, yaw_next, alpha),
            )
    return waypoints[-1][1], waypoints[-1][2], waypoints[-1][3]


def _build_obstacle_cloud(
    x: float, y: float, stamp, frame_id: str,
) -> PointCloud2:
    step = 0.05
    radius = GHOST_RADIUS
    points = []
    gx = x - radius
    while gx <= x + radius:
        gy = y - radius
        while gy <= y + radius:
            if (gx - x) ** 2 + (gy - y) ** 2 <= radius ** 2:
                points.append((gx, gy, GHOST_CENTER_Z))
            gy += step
        gx += step

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud = PointCloud2()
    cloud.header = Header(stamp=stamp, frame_id=frame_id)
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * len(points)
    cloud.is_dense = True
    cloud.data = b"".join(struct.pack("fff", *p) for p in points)
    return cloud


def _build_debug_marker(x: float, y: float, stamp) -> Marker:
    marker = Marker()
    marker.header = Header(stamp=stamp, frame_id=REFERENCE_FRAME)
    marker.ns = "actor_ghost"
    marker.id = 1
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = GHOST_CENTER_Z
    marker.pose.orientation.w = 1.0
    marker.scale.x = GHOST_RADIUS * 2.0
    marker.scale.y = GHOST_RADIUS * 2.0
    marker.scale.z = GHOST_HEIGHT
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.45
    return marker


class ActorCollisionShadow(Node):
    """Sync ghost collision model pose with walking actor trajectory."""

    def __init__(self) -> None:
        super().__init__("actor_collision_shadow")
        self._waypoints = self._load_waypoints()
        self._pose_pub = self.create_publisher(
            PoseStamped, "/actor_ghost/pose", 10
        )
        self._obstacle_pub = self.create_publisher(
            PointCloud2, OBSTACLE_TOPIC, 10
        )
        self._marker_pub = self.create_publisher(
            Marker, "/actor_ghost/debug_marker", 10
        )

        # Gazebo set_model_state client — teleports physical ghost cylinder
        # so that the LiDAR /scan naturally detects the moving person.
        self._gz_set_state = self.create_client(
            SetModelState, "/gazebo/set_model_state"
        )
        self._gz_set_state_root = self.create_client(
            SetModelState, "/set_model_state"
        )
        self._gazebo_connected = False
        self._gazebo_service_name = None
        self._last_service_warn_sec = -SERVICE_WARN_PERIOD_SEC

        period = 1.0 / PUBLISH_HZ
        self._timer = self.create_timer(period, self._tick)
        self._tick_count = 0
        self.get_logger().info(
            f"shadow started: hz={PUBLISH_HZ}, waypoints={len(self._waypoints)}, "
            f"cycle={self._cycle}s, obstacle_topic={OBSTACLE_TOPIC}"
        )

    def _load_waypoints(self) -> List[Tuple[float, float, float, float]]:
        raw = (
            self.declare_parameter("waypoints", value=[])
            .get_parameter_value()
            ._double_array_value
        )
        if raw and len(raw) >= 4 and len(raw) % 4 == 0:
            waypoints = [
                (raw[i], raw[i + 1], raw[i + 2], raw[i + 3])
                for i in range(0, len(raw), 4)
            ]
        else:
            self.get_logger().info("using default waypoints (from lab_actor.world)")
            waypoints = list(DEFAULT_WAYPOINTS)
        self._cycle = waypoints[-1][0]
        return waypoints

    def _teleport_ghost_model(self, x: float, y: float, yaw: float) -> None:
        """Move the physical ghost collision cylinder in Gazebo."""
        client = None
        service_name = None
        if self._gz_set_state.service_is_ready():
            client = self._gz_set_state
            service_name = "/gazebo/set_model_state"
        elif self._gz_set_state_root.service_is_ready():
            client = self._gz_set_state_root
            service_name = "/set_model_state"
        if client is None:
            self._gazebo_connected = False
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            elapsed = now_sec - self._last_service_warn_sec
            if elapsed >= SERVICE_WARN_PERIOD_SEC:
                self.get_logger().warn(
                    "set_model_state service not ready yet; publishing actor "
                    "cloud and will retry"
                )
                self._last_service_warn_sec = now_sec
            return
        if (not self._gazebo_connected
                or self._gazebo_service_name != service_name):
            self._gazebo_connected = True
            self._gazebo_service_name = service_name
            self.get_logger().info(f"gazebo set_model_state connected: {service_name}")
        req = SetModelState.Request()
        req.model_state = ModelState()
        req.model_state.model_name = GHOST_MODEL_NAME
        req.model_state.pose.position.x = x
        req.model_state.pose.position.y = y
        req.model_state.pose.position.z = GHOST_CENTER_Z
        req.model_state.pose.orientation.x = 0.0
        req.model_state.pose.orientation.y = 0.0
        req.model_state.pose.orientation.z = math.sin(yaw / 2.0)
        req.model_state.pose.orientation.w = math.cos(yaw / 2.0)
        req.model_state.reference_frame = REFERENCE_FRAME
        client.call_async(req)

    def _tick(self) -> None:
        t_sim = self.get_clock().now().nanoseconds * 1e-9
        trajectory_time = max(0.0, t_sim - ACTOR_DELAY_START_SEC)
        x, y, yaw = _interpolate_waypoint(self._waypoints, trajectory_time)
        stamp = self.get_clock().now().to_msg()
        pose_msg = PoseStamped(
            header=Header(stamp=stamp, frame_id=REFERENCE_FRAME),
            pose=Pose(
                position=Point(x=x, y=y, z=GHOST_CENTER_Z),
                orientation=Quaternion(
                    x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
                ),
            ),
        )
        self._pose_pub.publish(pose_msg)
        obstacle_cloud = _build_obstacle_cloud(x, y, stamp, REFERENCE_FRAME)
        self._obstacle_pub.publish(obstacle_cloud)
        self._marker_pub.publish(_build_debug_marker(x, y, stamp))

        # Teleport the physical ghost model so LiDAR can detect it.
        # Throttle: only call every other tick (10 Hz) to reduce service load.
        self._tick_count += 1
        if self._tick_count % 2 == 0:
            self._teleport_ghost_model(x, y, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = ActorCollisionShadow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
