"""Nav2 启动就绪守卫:轮询 /map 与 map->odom,失败时自动重试生命周期."""
from __future__ import annotations

import time

from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from lab_cobot_navigation.nav_startup_policy import (
    AMCL_NAME,
    MAP_SERVER_NAME,
    Readiness,
    RecoveryPlan,
    decide_recovery,
)


class NavStartupGuard(Node):
    """监控定位链就绪状态并对 map_server/amcl 做幂等生命周期补转."""

    def __init__(self) -> None:
        super().__init__("nav_startup_guard")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("recovery_interval_sec", 5.0)
        self.declare_parameter("max_recovery_attempts", 4)

        self._global_frame = str(self.get_parameter("global_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._max_attempts = int(self.get_parameter("max_recovery_attempts").value)
        self._map_received = False
        self._ready_published = False
        self._attempts = {}
        # rclpy Node already owns Node._clients (a list); keep service clients
        # in our own dict so create_client() does not mutate the wrong field.
        self._service_clients: dict[tuple[str, str], object] = {}
        self._node_states: dict[str, int] = {}
        self._pending_states: dict[str, object] = {}
        self._state_query_since: dict[str, float] = {}
        self._pending_transitions: dict[str, list[int]] = {}

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._ready_pub = self.create_publisher(Bool, "nav_startup_ready", 1)
        interval = float(self.get_parameter("recovery_interval_sec").value)
        self._timer = self.create_timer(interval, self._check)
        self.get_logger().info(
            "nav_startup_guard online: waiting for /map + map->odom"
        )

    def _on_map(self, _msg: OccupancyGrid) -> None:
        self._map_received = True

    def _check(self) -> None:
        map_to_odom_ready = False
        if self._map_received:
            map_to_odom_ready = self._tf_buffer.can_transform(
                self._global_frame,
                self._odom_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        if self._map_received and map_to_odom_ready:
            self._publish_ready()
            return
        if self._pending_transitions:
            return

        self._request_lifecycle_states()
        self._fallback_lifecycle_states()
        if not self._states_available():
            return

        lifecycle_states = {
            name: self._lifecycle_state(name)
            for name in (MAP_SERVER_NAME, AMCL_NAME)
        }
        readiness = Readiness(
            map_received=self._map_received,
            map_to_odom_ready=map_to_odom_ready,
            lifecycle_states=lifecycle_states,
        )
        plan = decide_recovery(readiness)
        if plan is None:
            self._publish_ready()
            return
        attempts = self._attempts.get(plan.node_name, 0)
        if attempts >= self._max_attempts:
            self.get_logger().error(
                f"nav startup still not ready: /map={self._map_received} "
                f"map->odom={map_to_odom_ready} "
                f"(retry limit reached for {plan.node_name})"
            )
            return
        self._attempts[plan.node_name] = attempts + 1
        self._recover(plan)

    def _recover(self, plan: RecoveryPlan) -> None:
        self.get_logger().warn(
            f"recovery attempt {self._attempts[plan.node_name]} for "
            f"{plan.node_name} transitions={list(plan.transitions)}"
        )
        self._pending_transitions[plan.node_name] = [
            int(t) for t in plan.transitions
        ]
        self._advance_transitions(plan.node_name)

    def _lifecycle_state(self, node_name: str) -> int:
        return self._node_states.get(node_name, 1)

    def _request_lifecycle_states(self) -> None:
        now = time.monotonic()
        for name in (MAP_SERVER_NAME, AMCL_NAME):
            if name in self._node_states or name in self._pending_states:
                continue
            self._state_query_since.setdefault(name, now)
            client = self._service_client(name, "get_state", GetState)
            if not client.service_is_ready():
                continue
            future = client.call_async(GetState.Request())
            self._pending_states[name] = future
            future.add_done_callback(
                lambda fut, node_name=name: self._on_state(node_name, fut)
            )

    def _fallback_lifecycle_states(self) -> None:
        """Fall back to unconfigured when lifecycle services are unavailable."""
        now = time.monotonic()
        for name in (MAP_SERVER_NAME, AMCL_NAME):
            if name in self._node_states or name in self._pending_states:
                continue
            if now - self._state_query_since.get(name, now) > 6.0:
                self.get_logger().warn(
                    f"{name} get_state unavailable; using unconfigured fallback"
                )
                self._node_states[name] = 1

    def _states_available(self) -> bool:
        return all(
            name in self._node_states
            for name in (MAP_SERVER_NAME, AMCL_NAME)
        )

    def _on_state(self, node_name: str, future) -> None:
        self._pending_states.pop(node_name, None)
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"{node_name} get_state failed: {exc}")
            return
        if result is None:
            self.get_logger().warn(f"{node_name} get_state returned no result")
            return
        self._node_states[node_name] = int(result.current_state.id)

    def _advance_transitions(self, node_name: str) -> None:
        queue = self._pending_transitions.get(node_name)
        if not queue:
            return
        client = self._service_client(node_name, "change_state", ChangeState)
        if not client.service_is_ready():
            self.get_logger().warn(
                f"{node_name} change_state service not ready"
            )
            return
        transition_id = queue[0]
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        future.add_done_callback(
            lambda fut, name=node_name, tid=transition_id:
            self._on_transition_done(fut, name, tid)
        )

    def _on_transition_done(
        self, future, node_name: str, transition_id: int
    ) -> None:
        queue = self._pending_transitions.get(node_name)
        if not queue or queue[0] != transition_id:
            return
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"{node_name} transition {transition_id} failed: {exc}"
            )
        else:
            if result is None or not result.success:
                self.get_logger().warn(
                    f"{node_name} transition {transition_id} rejected"
                )
            else:
                self.get_logger().info(
                    f"{node_name} transition {transition_id} accepted"
                )
        queue.pop(0)
        if queue:
            self._advance_transitions(node_name)
        else:
            self._node_states.pop(node_name, None)
            self._pending_transitions.pop(node_name, None)

    def _service_client(self, node_name: str, suffix: str, srv_type):
        key = (node_name, suffix)
        if key not in self._service_clients:
            self._service_clients[key] = self.create_client(
                srv_type, "/{}/{}".format(node_name, suffix)
            )
        return self._service_clients[key]

    def _publish_ready(self) -> None:
        if self._ready_published:
            return
        self._ready_published = True
        self._ready_pub.publish(Bool(data=True))
        self.get_logger().info(
            "nav localization ready: /map received and map->odom tf available"
        )
        if self._timer is not None:
            self._timer.cancel()
