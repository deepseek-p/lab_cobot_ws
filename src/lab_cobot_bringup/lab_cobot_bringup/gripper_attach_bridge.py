#!/usr/bin/env python3
"""Gazebo Classic attach bridge for the simulated parallel gripper."""
from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import GetLinkProperties, SetEntityState, SetLinkProperties
from geometry_msgs.msg import Pose, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

from lab_cobot_bringup.grasp_validator import (
    GraspValidationConfig,
    validate_tcp_object_grasp,
)


OBJECT_NAME = "aruco_sample"
ATTACH_TOPIC = "/gripper/attach/aruco_sample"
DETACH_TOPIC = "/gripper/detach/aruco_sample"
ATTACH_STATUS_TOPIC = "/gripper/attach/status"
MODEL_STATES_TOPIC = "/gazebo/model_states"
SET_ENTITY_STATE_SERVICE = "/gazebo/set_entity_state"
GET_LINK_PROPERTIES_SERVICE = "/gazebo/get_link_properties"
SET_LINK_PROPERTIES_SERVICE = "/gazebo/set_link_properties"
TF_REFERENCE_FRAME = "odom"
GAZEBO_REFERENCE_FRAME = "world"
TCP_FRAME = "gripper_tcp"
OBJECT_LINK_NAME = f"{OBJECT_NAME}::link"
VISUAL_HOLD_OFFSET_TCP = (0.0, 0.0, 0.0)


@dataclass
class Attachment:
    offset_tcp: tuple[float, float, float]
    relative_orientation: Quaternion


@dataclass
class LinkPhysicsProperties:
    com: Pose
    gravity_mode: bool
    mass: float
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float


def quat_tuple(q: Quaternion) -> tuple[float, float, float, float]:
    return (q.x, q.y, q.z, q.w)


def quat_msg(values: tuple[float, float, float, float]) -> Quaternion:
    q = Quaternion()
    q.x, q.y, q.z, q.w = values
    return q


def quat_normalize(
    q: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quat_conjugate(
    q: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply_raw(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return quat_normalize(quat_multiply_raw(a, b))


def rotate_vector(
    q: tuple[float, float, float, float],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    vx, vy, vz = v
    rotated = quat_multiply_raw(
        quat_multiply_raw(quat_normalize(q), (vx, vy, vz, 0.0)),
        quat_conjugate(quat_normalize(q)),
    )
    return (rotated[0], rotated[1], rotated[2])


def hold_offset_for_validated_grasp(
    _validated_offset_tcp: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return the simulated hold offset after a grasp has passed validation."""
    return VISUAL_HOLD_OFFSET_TCP


def make_attached_entity_state(
    name: str,
    pose: Pose,
    reference_frame: str,
) -> EntityState:
    state = EntityState()
    state.name = name
    state.pose = pose
    state.reference_frame = reference_frame
    return state


def capture_link_properties(
    response: GetLinkProperties.Response,
) -> LinkPhysicsProperties | None:
    if not response.success:
        return None
    return LinkPhysicsProperties(
        com=response.com,
        gravity_mode=bool(response.gravity_mode),
        mass=float(response.mass),
        ixx=float(response.ixx),
        ixy=float(response.ixy),
        ixz=float(response.ixz),
        iyy=float(response.iyy),
        iyz=float(response.iyz),
        izz=float(response.izz),
    )


def make_link_properties_request(
    link_name: str,
    properties: LinkPhysicsProperties,
    gravity_mode: bool,
) -> SetLinkProperties.Request:
    request = SetLinkProperties.Request()
    request.link_name = link_name
    request.com = properties.com
    request.gravity_mode = bool(gravity_mode)
    request.mass = properties.mass
    request.ixx = properties.ixx
    request.ixy = properties.ixy
    request.ixz = properties.ixz
    request.iyy = properties.iyy
    request.iyz = properties.iyz
    request.izz = properties.izz
    return request


def wait_until_gazebo_ready(
    node,
    ok=rclpy.ok,
    retry_timeout_sec: float = 1.0,
) -> bool:
    while ok():
        if node.wait_for_gazebo(timeout_sec=retry_timeout_sec):
            return True
        node.get_logger().warn(
            "waiting for Gazebo Classic state/properties services"
        )
    return False


class GripperAttachBridge(Node):
    def __init__(self) -> None:
        super().__init__("gripper_attach_bridge")
        self.declare_parameter("object_name", OBJECT_NAME)
        self.declare_parameter("tf_reference_frame", TF_REFERENCE_FRAME)
        self.declare_parameter("gazebo_reference_frame", GAZEBO_REFERENCE_FRAME)
        self.declare_parameter("tcp_frame", TCP_FRAME)
        self.declare_parameter("object_link_name", OBJECT_LINK_NAME)
        self.declare_parameter("model_states_topic", MODEL_STATES_TOPIC)
        self.declare_parameter("update_rate", 120.0)
        self.declare_parameter("grasp.max_center_distance_m", 0.080)
        self.declare_parameter("grasp.max_abs_x_m", 0.040)
        self.declare_parameter("grasp.max_abs_y_m", 0.018)
        self.declare_parameter("grasp.min_z_m", -0.060)
        self.declare_parameter("grasp.max_z_m", 0.025)

        self._object_name = str(self.get_parameter("object_name").value)
        self._tf_reference_frame = str(self.get_parameter("tf_reference_frame").value)
        self._gazebo_reference_frame = str(
            self.get_parameter("gazebo_reference_frame").value
        )
        self._tcp_frame = str(self.get_parameter("tcp_frame").value)
        self._object_link_name = str(self.get_parameter("object_link_name").value)
        self._model_pose: Pose | None = None
        self._attachment: Attachment | None = None
        self._attachment_generation = 0
        self._object_link_properties: LinkPhysicsProperties | None = None
        self._pending = None
        self._grasp_config = GraspValidationConfig(
            max_center_distance_m=float(
                self.get_parameter("grasp.max_center_distance_m").value
            ),
            max_abs_x_m=float(self.get_parameter("grasp.max_abs_x_m").value),
            max_abs_y_m=float(self.get_parameter("grasp.max_abs_y_m").value),
            min_z_m=float(self.get_parameter("grasp.min_z_m").value),
            max_z_m=float(self.get_parameter("grasp.max_z_m").value),
        )

        self._set_entity_state = self.create_client(
            SetEntityState,
            SET_ENTITY_STATE_SERVICE,
        )
        self._get_link_properties = self.create_client(
            GetLinkProperties,
            GET_LINK_PROPERTIES_SERVICE,
        )
        self._set_link_properties = self.create_client(
            SetLinkProperties,
            SET_LINK_PROPERTIES_SERVICE,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            ModelStates,
            str(self.get_parameter("model_states_topic").value),
            self._on_model_states,
            10,
        )
        self.create_subscription(Empty, ATTACH_TOPIC, self._attach, 10)
        self.create_subscription(Empty, DETACH_TOPIC, self._detach, 10)
        self._attach_status_pub = self.create_publisher(
            String,
            ATTACH_STATUS_TOPIC,
            10,
        )

        rate = float(self.get_parameter("update_rate").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            "gripper_attach_bridge up; object=%s tcp_frame=%s"
            % (self._object_name, self._tcp_frame)
        )

    def wait_for_gazebo(self, timeout_sec: float = 10.0) -> bool:
        return (
            self._set_entity_state.wait_for_service(timeout_sec=timeout_sec)
            and self._get_link_properties.wait_for_service(timeout_sec=timeout_sec)
            and self._set_link_properties.wait_for_service(timeout_sec=timeout_sec)
        )

    def _on_model_states(self, msg: ModelStates) -> None:
        try:
            index = msg.name.index(self._object_name)
        except ValueError:
            return
        self._model_pose = msg.pose[index]

    def _tcp_pose(self) -> Pose | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._tf_reference_frame,
                self._tcp_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as ex:
            self.get_logger().debug(f"TCP transform unavailable: {ex}")
            return None

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def _publish_attach_status(self, status: str, reason: str = "") -> None:
        msg = String()
        if reason:
            msg.data = f"{status} {self._object_name} {reason}"
        else:
            msg.data = f"{status} {self._object_name}"
        self._attach_status_pub.publish(msg)

    def _attach(self, _msg: Empty) -> None:
        tcp = self._tcp_pose()
        obj = self._model_pose
        if tcp is None or obj is None:
            self.get_logger().warn(
                "cannot attach %s; missing TCP transform or model pose"
                % self._object_name
            )
            self._publish_attach_status("refused", "missing_tcp_or_model_pose")
            return

        tcp_q = quat_tuple(tcp.orientation)
        obj_q = quat_tuple(obj.orientation)
        offset_world = (
            obj.position.x - tcp.position.x,
            obj.position.y - tcp.position.y,
            obj.position.z - tcp.position.z,
        )
        offset_tcp = rotate_vector(quat_conjugate(tcp_q), offset_world)
        validation = validate_tcp_object_grasp(offset_tcp, self._grasp_config)
        if not validation.accepted:
            self.get_logger().warn(
                "refusing attach %s: %s offset_tcp=(%.3f, %.3f, %.3f)"
                % (
                    self._object_name,
                    validation.reason,
                    validation.offset_tcp[0],
                    validation.offset_tcp[1],
                    validation.offset_tcp[2],
                )
            )
            self._publish_attach_status("refused", validation.reason)
            return

        relative_q = quat_msg(quat_multiply(quat_conjugate(tcp_q), obj_q))
        hold_offset_tcp = hold_offset_for_validated_grasp(offset_tcp)
        self._attachment_generation += 1
        self._attachment = Attachment(hold_offset_tcp, relative_q)
        self._request_object_gravity_disabled(self._attachment_generation)
        self.get_logger().info(
            "attached %s to %s offset_tcp=(%.3f, %.3f, %.3f) hold_offset_tcp=(%.3f, %.3f, %.3f)"
            % (
                self._object_name,
                self._tcp_frame,
                offset_tcp[0],
                offset_tcp[1],
                offset_tcp[2],
                hold_offset_tcp[0],
                hold_offset_tcp[1],
                hold_offset_tcp[2],
            )
        )
        self._publish_attach_status("attached")

    def _detach(self, _msg: Empty) -> None:
        if self._attachment is not None:
            self._attachment_generation += 1
            self._attachment = None
            self._restore_object_gravity()
            self.get_logger().info("detached %s" % self._object_name)
            self._publish_attach_status("detached")

    def _request_object_gravity_disabled(self, generation: int) -> None:
        if not self._get_link_properties.service_is_ready():
            self.get_logger().warn(
                "cannot disable gravity for %s; %s unavailable"
                % (self._object_link_name, GET_LINK_PROPERTIES_SERVICE)
            )
            return
        request = GetLinkProperties.Request()
        request.link_name = self._object_link_name
        future = self._get_link_properties.call_async(request)
        future.add_done_callback(
            lambda result: self._on_object_link_properties(
                result,
                generation,
            )
        )

    def _on_object_link_properties(self, future, generation: int) -> None:
        if generation != self._attachment_generation or self._attachment is None:
            return
        try:
            response = future.result()
        except Exception as ex:  # pragma: no cover - defensive ROS callback path
            self.get_logger().warn(
                "failed to read link properties for %s: %s"
                % (self._object_link_name, ex)
            )
            return

        properties = capture_link_properties(response)
        if properties is None:
            self.get_logger().warn(
                "failed to read link properties for %s: %s"
                % (
                    self._object_link_name,
                    getattr(response, "status_message", ""),
                )
            )
            return

        self._object_link_properties = properties
        self._set_object_gravity(properties, gravity_mode=False)

    def _restore_object_gravity(self) -> None:
        properties = self._object_link_properties
        self._object_link_properties = None
        if properties is None:
            return
        self._set_object_gravity(properties, gravity_mode=properties.gravity_mode)

    def _set_object_gravity(
        self,
        properties: LinkPhysicsProperties,
        gravity_mode: bool,
    ) -> None:
        if not self._set_link_properties.service_is_ready():
            self.get_logger().warn(
                "cannot set gravity for %s; %s unavailable"
                % (self._object_link_name, SET_LINK_PROPERTIES_SERVICE)
            )
            return

        request = make_link_properties_request(
            self._object_link_name,
            properties,
            gravity_mode=gravity_mode,
        )
        future = self._set_link_properties.call_async(request)
        future.add_done_callback(
            lambda result: self._on_set_object_gravity(
                result,
                gravity_mode,
            )
        )

    def _on_set_object_gravity(self, future, gravity_mode: bool) -> None:
        try:
            response = future.result()
        except Exception as ex:  # pragma: no cover - defensive ROS callback path
            self.get_logger().warn(
                "failed to set gravity=%s for %s: %s"
                % (gravity_mode, self._object_link_name, ex)
            )
            return
        if not response.success:
            self.get_logger().warn(
                "failed to set gravity=%s for %s: %s"
                % (
                    gravity_mode,
                    self._object_link_name,
                    getattr(response, "status_message", ""),
                )
            )
            return
        self.get_logger().info(
            "set %s gravity_mode=%s" % (self._object_link_name, gravity_mode)
        )

    def _tick(self) -> None:
        if self._attachment is None:
            return
        if self._pending is not None and not self._pending.done():
            return

        tcp = self._tcp_pose()
        if tcp is None:
            return

        tcp_q = quat_tuple(tcp.orientation)
        offset_world = rotate_vector(tcp_q, self._attachment.offset_tcp)
        pose = Pose()
        pose.position.x = tcp.position.x + offset_world[0]
        pose.position.y = tcp.position.y + offset_world[1]
        pose.position.z = tcp.position.z + offset_world[2]
        pose.orientation = quat_msg(
            quat_multiply(tcp_q, quat_tuple(self._attachment.relative_orientation))
        )

        request = SetEntityState.Request()
        request.state = make_attached_entity_state(
            name=self._object_name,
            pose=pose,
            reference_frame=self._gazebo_reference_frame,
        )
        self._pending = self._set_entity_state.call_async(request)


def main() -> None:
    rclpy.init()
    node = GripperAttachBridge()
    try:
        if wait_until_gazebo_ready(node):
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as ex:
        if rclpy.ok():
            node.get_logger().error(f"gripper_attach_bridge stopped unexpectedly: {ex}")
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
