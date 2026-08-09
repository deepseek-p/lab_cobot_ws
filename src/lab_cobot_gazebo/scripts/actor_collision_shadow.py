#!/usr/bin/env python3
"""

Synchronize the collision cloud with the live Gazebo actor pose.

The actor pose comes from Gazebo model states, never from a copied trajectory.

Publishes:
  /actor_ghost/pose           — PoseStamped of ghost position (for metrics)
  /actor_ghost/obstacle_cloud — PointCloud2 of ghost cylinder (for costmap obstacle marking)

Also teleports the physical actor_ghost_collision model in Gazebo so that
the LiDAR (/scan) can directly detect the moving person.
"""
import math
import struct

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

REFERENCE_FRAME = "map"

# Ghost cylinder parameters.
GHOST_RADIUS = 0.60
GHOST_HEIGHT = 1.70
GHOST_CENTER_Z = 0.85
GHOST_MODEL_NAME = "actor_ghost_collision"
GHOST_ENTITY_NAME = "actor_ghost_collision::body"
OBSTACLE_TOPIC = "/actor_ghost/obstacle_cloud"

ACTOR_MODEL_NAME = "test_engineer_actor"
MODEL_STATES_TOPIC = "/gazebo/model_states"
SERVICE_WARN_PERIOD_SEC = 5.0


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
    """Mirror the live actor into a LiDAR-visible collision and cloud source."""

    def __init__(self) -> None:
        super().__init__("actor_collision_shadow")
        self._actor_pose = None
        self._model_states_sub = self.create_subscription(
            ModelStates, MODEL_STATES_TOPIC, self._on_model_states, 10
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, "/actor_ghost/pose", 10
        )
        self._obstacle_pub = self.create_publisher(
            PointCloud2, OBSTACLE_TOPIC, 10
        )
        self._marker_pub = self.create_publisher(
            Marker, "/actor_ghost/debug_marker", 10
        )

        # Gazebo state client moves the collision cylinder into the LiDAR scan.
        self._gz_set_state = self.create_client(
            SetEntityState, "/gazebo/set_entity_state"
        )
        self._gazebo_connected = False
        self._gazebo_service_name = None
        self._last_service_warn_sec = -SERVICE_WARN_PERIOD_SEC
        self._set_state_request_in_flight = False

        self.get_logger().info(
            f"shadow started: actor={ACTOR_MODEL_NAME}, "
            f"model_states={MODEL_STATES_TOPIC}, obstacle_topic={OBSTACLE_TOPIC}"
        )

    def _on_model_states(self, msg: ModelStates) -> None:
        try:
            index = msg.name.index(ACTOR_MODEL_NAME)
        except ValueError:
            return
        self._actor_pose = msg.pose[index]
        self._publish_actor_state(self._actor_pose)

    def _teleport_ghost_model(self, x: float, y: float, yaw: float) -> None:
        """Move the physical ghost collision cylinder in Gazebo."""
        if self._set_state_request_in_flight:
            return
        client = self._gz_set_state
        service_name = "/gazebo/set_entity_state"
        if not client.service_is_ready():
            self._gazebo_connected = False
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            elapsed = now_sec - self._last_service_warn_sec
            if elapsed >= SERVICE_WARN_PERIOD_SEC:
                self.get_logger().warn(
                    "set_entity_state service not ready yet; publishing actor "
                    "cloud and will retry"
                )
                self._last_service_warn_sec = now_sec
            return
        if (not self._gazebo_connected
                or self._gazebo_service_name != service_name):
            self._gazebo_connected = True
            self._gazebo_service_name = service_name
            self.get_logger().info(f"gazebo set_entity_state connected: {service_name}")
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = GHOST_ENTITY_NAME
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = GHOST_CENTER_Z
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = "world"
        request = client.call_async(req)
        self._set_state_request_in_flight = True
        request.add_done_callback(self._on_set_state_complete)

    def _on_set_state_complete(self, _future) -> None:
        self._set_state_request_in_flight = False

    def _publish_actor_state(self, actor_pose: Pose) -> None:
        """Publish and mirror one Gazebo actor sample without trajectory delay."""
        x = actor_pose.position.x
        y = actor_pose.position.y
        orientation = actor_pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
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

        # The cylinder is updated for every actor sample so /scan sees the same pose.
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
