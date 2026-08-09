#!/usr/bin/env python3
"""
Obstacle avoidance latency metrics collector + active safety evasion.

Monitors: actor ghost pose, local costmap, local plan, and robot pose
to produce a timeline of each avoidance event and publish summary metrics.

When the ghost person approaches a stationary robot, this node actively
commands evasion velocity to move the robot away from the person.

Metrics published as JSON on /obstacle_avoidance/metrics (std_msgs/String).
Evasion commands published on /cmd_vel_safety (geometry_msgs/Twist).
"""
import json
import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

LIDAR_MAX_RANGE = 3.0
GHOST_RADIUS = 0.60
ROBOT_SAFETY_RADIUS = 0.42
SAFETY_WARN_DIST = 1.00
SAFETY_CRITICAL_DIST = 0.70
SAFETY_EVADE_SPEED = 0.45
SAFETY_EVADE_CRITICAL_SPEED = 0.55
SAFETY_EVADE_HYSTERESIS = 0.25
NAV2_IDLE_TIMEOUT = 0.5             # seconds without non-zero cmd_vel_nav
COSTMAP_LETHAL = 100
COSTMAP_OCCUPIED = 90
DEVIATION_ANGLE_THRESH = 0.0873


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _plan_heading(path: Path, lookahead: int = 3) -> Optional[float]:
    if len(path.poses) < 2:
        return None
    idx = min(lookahead, len(path.poses) - 1)
    p0 = path.poses[0].pose.position
    p1 = path.poses[idx].pose.position
    dx = p1.x - p0.x
    dy = p1.y - p0.y
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return None
    return math.atan2(dy, dx)


def _is_nonzero_twist(msg: Twist) -> bool:
    return (abs(msg.linear.x) > 1e-4 or abs(msg.linear.y) > 1e-4
            or abs(msg.linear.z) > 1e-4 or abs(msg.angular.z) > 1e-4)


class AvoidanceMetrics(Node):
    """Collect metrics and actively evade approaching persons."""

    def __init__(self) -> None:
        super().__init__("obstacle_avoidance_metrics")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._events: List[Dict] = []
        self._ghost_pose: Optional[PoseStamped] = None
        self._costmap_data: Optional[OccupancyGrid] = None
        self._local_plan: Optional[Path] = None
        self._baseline_heading: Optional[float] = None
        self._avoidance_active = False
        self._event_start: Optional[float] = None
        self._t_detect: Optional[float] = None
        self._t_mark: Optional[float] = None
        self._t_deviate: Optional[float] = None
        self._min_distance = float("inf")
        self._last_nav2_cmd_time = 0.0
        self._evading = False
        self._task_status = ""
        # Ghost velocity tracking for predictive evasion
        self._last_ghost_xy: Optional[tuple] = None
        self._last_ghost_time: float = 0.0
        self._ghost_vx: float = 0.0
        self._ghost_vy: float = 0.0

        self._ghost_sub = self.create_subscription(
            PoseStamped, "/actor_ghost/pose", self._ghost_cb, 10
        )
        self._costmap_sub = self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._costmap_cb, 10
        )
        self._plan_sub = self.create_subscription(
            Path, "/local_plan", self._plan_cb, 10
        )
        self._nav2_cmd_sub = self.create_subscription(
            Twist, "/cmd_vel_nav", self._nav2_cmd_cb, 10
        )
        self._task_status_sub = self.create_subscription(
            String, "/task/status", self._task_status_cb, 10
        )
        self._metrics_pub = self.create_publisher(String, "/obstacle_avoidance/metrics", 10)
        self._safety_cmd_pub = self.create_publisher(Twist, "/cmd_vel_safety", 10)
        self._evaluate_timer = self.create_timer(0.1, self._evaluate)
        self.get_logger().info("metrics collector + safety evasion started")

    @property
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _ghost_cb(self, msg: PoseStamped) -> None:
        now = self._now
        gx, gy = msg.pose.position.x, msg.pose.position.y
        if self._last_ghost_xy is not None:
            dt = now - self._last_ghost_time
            if dt > 1e-6:
                self._ghost_vx = (gx - self._last_ghost_xy[0]) / dt
                self._ghost_vy = (gy - self._last_ghost_xy[1]) / dt
        self._last_ghost_xy = (gx, gy)
        self._last_ghost_time = now
        self._ghost_pose = msg

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        self._costmap_data = msg

    def _plan_cb(self, msg: Path) -> None:
        self._local_plan = msg

    def _nav2_cmd_cb(self, msg: Twist) -> None:
        if _is_nonzero_twist(msg):
            self._last_nav2_cmd_time = self._now

    def _task_status_cb(self, msg: String) -> None:
        self._task_status = str(msg.data)

    @property
    def _nav2_is_idle(self) -> bool:
        return (self._now - self._last_nav2_cmd_time) > NAV2_IDLE_TIMEOUT

    def _safety_override_allowed(self) -> bool:
        status = self._task_status
        return (
            status.startswith("NAV_TO_")
            or status.startswith("RETURN_HOME")
        ) and not self._nav2_is_idle

    def _get_robot_pose(self) -> Optional[tuple]:
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
                _yaw_from_quat(
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w,
                ),
            )
        except TransformException:
            return None

    def _distance_to_ghost(self) -> Optional[float]:
        robot = self._get_robot_pose()
        ghost = self._ghost_pose
        if robot is None or ghost is None:
            return None
        dx = robot[0] - ghost.pose.position.x
        dy = robot[1] - ghost.pose.position.y
        return math.hypot(dx, dy) - GHOST_RADIUS - ROBOT_SAFETY_RADIUS

    def _is_ghost_in_costmap(self) -> bool:
        if self._costmap_data is None or self._ghost_pose is None:
            return False
        data = self._costmap_data
        gx = self._ghost_pose.pose.position.x
        gy = self._ghost_pose.pose.position.y
        costmap_frame = data.header.frame_id or "map"
        if costmap_frame != "map":
            try:
                transform = self._tf_buffer.lookup_transform(
                    costmap_frame, "map", rclpy.time.Time()
                )
            except TransformException:
                return False
            yaw = _yaw_from_quat(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            gx, gy = (
                transform.transform.translation.x + math.cos(yaw) * gx - math.sin(yaw) * gy,
                transform.transform.translation.y + math.sin(yaw) * gx + math.cos(yaw) * gy,
            )
        ox = data.info.origin.position.x
        oy = data.info.origin.position.y
        res = data.info.resolution
        w, h = data.info.width, data.info.height
        col = int((gx - ox) / res)
        row = int((gy - oy) / res)
        if col < 0 or col >= w or row < 0 or row >= h:
            return False
        idx = row * w + col
        if idx < 0 or idx >= len(data.data):
            return False
        return data.data[idx] >= COSTMAP_OCCUPIED

    # ------------------------------------------------------------------
    # Active safety evasion
    # ------------------------------------------------------------------

    def _is_blocked_by_costmap(
        self, robot_xy: tuple, angle_rad: float, lookahead: float = 0.55
    ) -> bool:
        """Check whether moving *lookahead* metres along *angle_rad* hits an obstacle."""
        if self._costmap_data is None:
            return False
        data = self._costmap_data
        cx = robot_xy[0] + lookahead * math.cos(angle_rad)
        cy = robot_xy[1] + lookahead * math.sin(angle_rad)
        ox = data.info.origin.position.x
        oy = data.info.origin.position.y
        res = data.info.resolution
        w, h = data.info.width, data.info.height
        col = int((cx - ox) / res)
        row = int((cy - oy) / res)
        if col < 0 or col >= w or row < 0 or row >= h:
            return True
        idx = row * w + col
        if idx < 0 or idx >= len(data.data):
            return True
        return data.data[idx] >= COSTMAP_OCCUPIED

    def _publish_evasion(self, speed: float) -> None:
        """Evade perpendicular to ghost velocity — step aside, don't run away."""
        robot = self._get_robot_pose()
        ghost = self._ghost_pose
        if robot is None or ghost is None:
            return
        dx = robot[0] - ghost.pose.position.x
        dy = robot[1] - ghost.pose.position.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return

        robot_xy = (robot[0], robot[1])

        # Perpendicular to ghost's movement: ghost walks past, robot steps aside
        gspeed = math.hypot(self._ghost_vx, self._ghost_vy)
        if gspeed > 0.05:
            # Ghost walking direction unit vector
            gdx = self._ghost_vx / gspeed
            gdy = self._ghost_vy / gspeed
            # Two perpendicular directions (left / right of ghost's path)
            candidates = [
                math.atan2(gdx, -gdy),   # perpendicular: turn ghost-dir 90° right
                math.atan2(-gdx, gdy),   # perpendicular: turn ghost-dir 90° left
                math.atan2(dy, dx),      # fallback: away from ghost
                math.atan2(dy, dx) + math.pi / 4,
                math.atan2(dy, dx) - math.pi / 4,
            ]
        else:
            # Ghost stationary — just move away
            away_angle = math.atan2(dy, dx)
            candidates = [
                away_angle,
                away_angle + math.pi / 4,
                away_angle - math.pi / 4,
                away_angle + math.pi / 2,
                away_angle - math.pi / 2,
            ]

        chosen = None
        for angle in candidates:
            if not self._is_blocked_by_costmap(robot_xy, angle):
                chosen = angle
                break

        if chosen is None:
            self._publish_zero_velocity()
            return

        yaw = robot[2]
        vx = speed * math.cos(chosen)
        vy = speed * math.sin(chosen)
        msg = Twist()
        msg.linear.x = vx * math.cos(yaw) + vy * math.sin(yaw)
        msg.linear.y = -vx * math.sin(yaw) + vy * math.cos(yaw)
        self._safety_cmd_pub.publish(msg)

    def _publish_zero_velocity(self) -> None:
        self._safety_cmd_pub.publish(Twist())

    def _handle_safety_evasion(self, dist: Optional[float]) -> None:
        """Decide whether to actively evade and publish commands."""
        if dist is None:
            if self._evading:
                self._publish_zero_velocity()
                self._evading = False
            return

        need_evade = False
        evade_speed = SAFETY_EVADE_SPEED

        if dist < SAFETY_WARN_DIST:
            need_evade = True
            evade_speed = SAFETY_EVADE_SPEED
        if dist < SAFETY_CRITICAL_DIST:
            evade_speed = SAFETY_EVADE_CRITICAL_SPEED

        # Hysteresis: keep evading until ghost is well clear
        if self._evading and dist < SAFETY_WARN_DIST + SAFETY_EVADE_HYSTERESIS:
            need_evade = True

        if need_evade:
            self._publish_evasion(evade_speed)
            if not self._evading:
                self.get_logger().info(
                    f"safety evasion ON: dist={dist:.2f}m speed={evade_speed:.2f}m/s"
                )
            self._evading = True
        elif self._evading:
            self._publish_zero_velocity()
            self.get_logger().info("safety evasion OFF: ghost cleared")
            self._evading = False

    # ------------------------------------------------------------------
    # Metrics evaluation (original logic)
    # ------------------------------------------------------------------

    def _evaluate(self) -> None:
        now = self._now
        dist = self._distance_to_ghost()

        # Only take over base motion while Nav2 is actively navigating. An
        # idle robot must remain stationary even if the actor starts nearby.
        if self._safety_override_allowed():
            self._handle_safety_evasion(dist)
        elif self._evading:
            self._publish_zero_velocity()
            self._evading = False

        if not self._safety_override_allowed():
            return

        # --- metrics collection ---
        ghost_in_range = dist is not None and dist < LIDAR_MAX_RANGE
        ghost_marked = self._is_ghost_in_costmap()
        heading = _plan_heading(self._local_plan) if self._local_plan else None
        deviated = False
        if heading is not None and self._baseline_heading is not None:
            diff = abs(math.atan2(
                math.sin(heading - self._baseline_heading),
                math.cos(heading - self._baseline_heading),
            ))
            deviated = diff > DEVIATION_ANGLE_THRESH

        if dist is not None and dist < self._min_distance:
            self._min_distance = dist

        if dist is not None:
            if dist < SAFETY_CRITICAL_DIST:
                self.get_logger().error(f"CRITICAL: robot-ghost distance {dist:.2f}m")
            elif dist < SAFETY_WARN_DIST:
                self.get_logger().warn(f"WARN: robot-ghost distance {dist:.2f}m")

        if ghost_in_range and not self._avoidance_active:
            self._avoidance_active = True
            self._event_start = now
            self._t_detect = now
            self._t_mark = None
            self._t_deviate = None
            self._min_distance = dist if dist is not None else float("inf")
            if heading is not None:
                self._baseline_heading = heading
            self.get_logger().info(f"detection: ghost entered range at dist={dist:.2f}m")

        if self._avoidance_active and ghost_marked and self._t_mark is None:
            self._t_mark = now
            self.get_logger().info(f"costmap mark: +{self._t_mark - self._event_start:.3f}s")

        if self._avoidance_active and deviated and self._t_deviate is None:
            self._t_deviate = now
            self.get_logger().info(f"path deviate: +{self._t_deviate - self._event_start:.3f}s")

        if self._avoidance_active and not ghost_in_range:
            self._finalize_event(now)
            return

    def _finalize_event(self, now: float) -> None:
        if self._event_start is None:
            return
        event = {
            "t_detect": self._event_start,
            "t_mark": self._t_mark,
            "t_deviate": self._t_deviate,
            "t_clear": now,
            "perception_latency_ms": round(1000 * (self._t_mark - self._event_start), 1)
            if self._t_mark else None,
            "replan_latency_ms": round(1000 * (self._t_deviate - self._t_mark), 1)
            if self._t_deviate and self._t_mark else None,
            "total_response_ms": round(1000 * (self._t_deviate - self._event_start), 1)
            if self._t_deviate else None,
            "total_duration_ms": round(1000 * (now - self._event_start), 1),
            "min_distance_m": round(self._min_distance, 3),
        }
        self._events.append(event)
        msg = String(data=json.dumps(event))
        self._metrics_pub.publish(msg)
        self.get_logger().info(
            f"event #{len(self._events)}: response={event['total_response_ms']}ms "
            f"min_dist={event['min_distance_m']}m"
        )
        self._avoidance_active = False
        self._event_start = None
        self._t_detect = None
        self._t_mark = None
        self._t_deviate = None
        self._baseline_heading = None
        self._min_distance = float("inf")


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
