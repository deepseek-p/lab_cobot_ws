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
import os
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

LIDAR_MAX_RANGE = 4.5
GHOST_RADIUS = 0.60
ROBOT_SAFETY_RADIUS = 0.42
SAFETY_WARN_DIST = 2.00
SAFETY_CRITICAL_DIST = 1.20
SAFETY_EVADE_SPEED = 0.50
SAFETY_EVADE_CRITICAL_SPEED = 0.60
SAFETY_EVADE_HYSTERESIS = 0.40
EVASION_HEADING_HOLD_SEC = 0.50
NAV2_IDLE_TIMEOUT = 0.5             # seconds without non-zero cmd_vel_nav
COSTMAP_LETHAL = 100
COSTMAP_OCCUPIED = 90
DEVIATION_ANGLE_THRESH = 0.0873
COSTMAP_LOOKAHEAD_STEP = 0.20
COSTMAP_MAX_LOOKAHEAD = 1.60
EVASION_CANDIDATE_SPACING = 0.18
PLAN_ALIGN_BONUS = 1.00
PLAN_ALIGN_ACTIVE_FACTOR = 4.0
PLAN_SAFE_PREDICTED_SEPARATION = SAFETY_CRITICAL_DIST + 0.15
CLOSING_OVERRIDE_SPEED = 0.20
EVASION_TIMEOUT_SEC = 4.0
EVASION_SUPPRESS_SEC = 2.0
EVASION_PREDICT_HORIZON_SEC = 4.0
EVASION_PREDICT_STEP_SEC = 0.5
EVASION_PREDICT_SPEED = SAFETY_EVADE_SPEED
EVASION_GHOST_VEL_ALPHA = 0.6
EVASION_FUTURE_SEP_WEIGHT = 1.0
EVASION_COLLISION_PENALTY_FACTOR = 4.0
EVASION_LATERAL_BONUS = 2.0
EVASION_LATERAL_TOLERANCE = 0.35
# 候选避让方向所需的最小 costmap 净空(m)。低于该值视为“被挡”:
# 只允许机器人沿净空充足的方向让行,禁止朝静态障碍蠕动/硬推。
EVASION_MIN_CLEARANCE = 0.30


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
        self._t_evade: Optional[float] = None
        self._t_safety_cmd_pub: Optional[float] = None
        self._t_cmd_vel: Optional[float] = None
        self._t_warn_detected: Optional[float] = None
        self._t_warn_stamp: Optional[float] = None
        self._last_safety_cmd: Optional[tuple] = None
        self._evade_heading: Optional[float] = None
        self._evade_heading_set_at: Optional[float] = None
        self._evade_heading_dist: Optional[float] = None
        self._last_evasion_log_time: Optional[float] = None
        self._events_file = os.environ.get(
            "OBSTACLE_AVOIDANCE_EVENTS_FILE",
            "/tmp/obstacle_avoidance_events.jsonl",
        )
        self._min_distance = float("inf")
        self._last_nav2_cmd_time = 0.0
        self._evading = False
        self._t_evade_release: Optional[float] = None
        self._task_status = ""
        # Ghost velocity tracking for predictive evasion
        self._last_ghost_xy: Optional[tuple] = None
        self._last_ghost_time: float = 0.0
        self._ghost_vx: float = 0.0
        self._ghost_vy: float = 0.0
        self._closing_speed: float = 0.0
        self._last_eval_dist: Optional[float] = None
        self._last_eval_time: float = 0.0
        self._last_nav2_cmd: Optional[Twist] = None

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
        self._cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_cb, 10
        )
        self._task_status_sub = self.create_subscription(
            String, "/task/status", self._task_status_cb, 10
        )
        self._metrics_pub = self.create_publisher(String, "/obstacle_avoidance/metrics", 10)
        self._safety_cmd_pub = self.create_publisher(Twist, "/cmd_vel_safety", 10)
        self._evaluate_timer = self.create_timer(0.05, self._evaluate)
        self.get_logger().info("metrics collector + safety evasion started")

    @property
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _ghost_stamp_seconds(self) -> Optional[float]:
        if self._ghost_pose is None:
            return None
        st = self._ghost_pose.header.stamp
        return float(st.sec) + float(st.nanosec) * 1e-9

    def _ghost_cb(self, msg: PoseStamped) -> None:
        now = self._now
        gx, gy = msg.pose.position.x, msg.pose.position.y
        if self._last_ghost_xy is not None:
            dt = now - self._last_ghost_time
            if dt > 1e-6:
                inst_vx = (gx - self._last_ghost_xy[0]) / dt
                inst_vy = (gy - self._last_ghost_xy[1]) / dt
                # 平滑瞬时速度,避免短暂抖动让避让方向在侧向之间来回翻转.
                self._ghost_vx = (
                    EVASION_GHOST_VEL_ALPHA * inst_vx
                    + (1.0 - EVASION_GHOST_VEL_ALPHA) * self._ghost_vx
                )
                self._ghost_vy = (
                    EVASION_GHOST_VEL_ALPHA * inst_vy
                    + (1.0 - EVASION_GHOST_VEL_ALPHA) * self._ghost_vy
                )
        self._last_ghost_xy = (gx, gy)
        self._last_ghost_time = now
        self._ghost_pose = msg
        # 事件驱动响应:即使 20Hz 评估定时器被系统负载拖慢,
        # ghost 位姿回调到达时也立即判断是否需要让行。
        dist = self._distance_to_ghost()
        if self._safety_override_allowed():
            self._handle_safety_evasion(dist)

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        self._costmap_data = msg

    def _plan_cb(self, msg: Path) -> None:
        self._local_plan = msg

    def _nav2_cmd_cb(self, msg: Twist) -> None:
        if _is_nonzero_twist(msg):
            self._last_nav2_cmd_time = self._now
            self._last_nav2_cmd = msg

    def _cmd_vel_cb(self, msg: Twist) -> None:
        if not _is_nonzero_twist(msg):
            return
        if (self._evading and self._t_safety_cmd_pub is not None
                and self._t_cmd_vel is None and self._safety_matches(msg)):
            self._t_cmd_vel = self._now
            self.get_logger().info(
                "reactive cmd_vel observed at "
                f"+{1000 * (self._t_cmd_vel - self._t_safety_cmd_pub):.1f}ms"
            )

    def _safety_matches(self, msg: Twist) -> bool:
        s = self._last_safety_cmd
        if s is None:
            return False
        sx, sy, sz, st = s
        s_norm = math.hypot(sx, sy, sz, st)
        m_norm = math.hypot(
            msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z
        )
        if s_norm < 1e-6 or m_norm < 1e-6:
            return False
        # Only count a /cmd_vel that is essentially the safety command the
        # mux forwards, not a same-direction Nav2/manual retreat command.
        cos_angle = (
            msg.linear.x * sx + msg.linear.y * sy
            + msg.linear.z * sz + msg.angular.z * st
        ) / (s_norm * m_norm)
        speed_ratio = m_norm / s_norm
        return cos_angle >= 0.99 and 0.9 <= speed_ratio <= 1.15

    def _task_status_cb(self, msg: String) -> None:
        self._task_status = str(msg.data)

    @property
    def _nav2_is_idle(self) -> bool:
        return (self._now - self._last_nav2_cmd_time) > NAV2_IDLE_TIMEOUT

    @property
    def _nav2_active(self) -> bool:
        return (self._now - self._last_nav2_cmd_time) <= NAV2_IDLE_TIMEOUT

    def _closing_velocity(self) -> float:
        """Positive means robot and ghost are getting closer (m/s)."""
        robot = self._get_robot_pose()
        ghost = self._ghost_pose
        if robot is None or ghost is None:
            return 0.0
        rx, ry = robot[0], robot[1]
        gx = ghost.pose.position.x
        gy = ghost.pose.position.y
        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0
        if self._last_nav2_cmd is not None:
            yaw = robot[2]
            nav_vx = self._last_nav2_cmd.linear.x * math.cos(yaw) \
                - self._last_nav2_cmd.linear.y * math.sin(yaw)
            nav_vy = self._last_nav2_cmd.linear.x * math.sin(yaw) \
                + self._last_nav2_cmd.linear.y * math.cos(yaw)
        else:
            nav_vx = nav_vy = 0.0
        relative_approach = (
            - (dx * (nav_vx - self._ghost_vx) + dy * (nav_vy - self._ghost_vy)) / dist
        )
        return max(0.0, relative_approach)

    def _future_ghost_xy(
        self, ahead_sec: float, base_xy: Optional[tuple] = None
    ) -> tuple:
        if base_xy is None:
            ghost = self._ghost_pose
            if ghost is None:
                return (0.0, 0.0)
            base_xy = (ghost.pose.position.x, ghost.pose.position.y)
        return (
            base_xy[0] + self._ghost_vx * ahead_sec,
            base_xy[1] + self._ghost_vy * ahead_sec,
        )

    def _future_separation(
        self,
        robot_xy: tuple,
        angle_rad: float,
        ghost_xy: tuple,
        horizon_sec: float = EVASION_PREDICT_HORIZON_SEC,
        speed: float = EVASION_PREDICT_SPEED,
    ) -> float:
        """Min separation while holding a candidate heading for the horizon."""
        min_sep = float("inf")
        t = 0.0
        while t <= horizon_sec + 1e-9:
            gx, gy = self._future_ghost_xy(t, ghost_xy)
            rx = robot_xy[0] + speed * t * math.cos(angle_rad)
            ry = robot_xy[1] + speed * t * math.sin(angle_rad)
            min_sep = min(min_sep, math.hypot(rx - gx, ry - gy))
            t += EVASION_PREDICT_STEP_SEC
        return min_sep

    def _update_closing_speed(self, dist: Optional[float]) -> None:
        now = self._now
        if dist is None or self._last_eval_dist is None:
            self._closing_speed = 0.0
        else:
            dt = now - self._last_eval_time
            if dt > 1e-6:
                self._closing_speed = max(
                    0.0, (self._last_eval_dist - dist) / dt
                )
        self._last_eval_dist = dist
        self._last_eval_time = now

    def _safety_override_allowed(self) -> bool:
        status = self._task_status
        navigating = (
            status.startswith("NAV_TO_")
            or status.startswith("RETURN_HOME")
            or status.startswith("ARRIVED:")
        )
        if not navigating:
            return False
        dist = self._distance_to_ghost()
        self._update_closing_speed(dist)
        if dist is not None and dist < SAFETY_CRITICAL_DIST:
            return True
        if dist is None:
            return False
        if (self._t_evade_release is not None
                and self._now - self._t_evade_release < EVASION_SUPPRESS_SEC):
            return False
        # Nav2 正常运行期间保持“让行但不抢道”:只有当人员正在逼近、或导航命令
        # 已经停顿超过阈值时才接管,避免 ghost 在 2m 外平行伴行时把机器人
        # 反复推出全局可规划路径。
        if self._closing_speed >= CLOSING_OVERRIDE_SPEED \
                and dist < SAFETY_WARN_DIST + SAFETY_EVADE_HYSTERESIS:
            return True
        return self._nav2_is_idle

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
        ox, oy, yaw = self._map_to_costmap_offset()
        if ox is None:
            return False
        gx, gy = (
            ox + math.cos(yaw) * gx - math.sin(yaw) * gy,
            oy + math.sin(yaw) * gx + math.cos(yaw) * gy,
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

    def _map_to_costmap_offset(self) -> tuple:
        """Return (x, y, yaw) that rotates/translates map points into costmap frame."""
        data = self._costmap_data
        if data is None:
            return (None, None, None)
        costmap_frame = data.header.frame_id or "map"
        if costmap_frame == "map":
            return (0.0, 0.0, 0.0)
        try:
            transform = self._tf_buffer.lookup_transform(
                costmap_frame, "map", rclpy.time.Time()
            )
        except TransformException:
            return (None, None, None)
        yaw = _yaw_from_quat(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
        )

    def _costmap_clearance(self, robot_xy: tuple, angle_rad: float) -> float:
        """Distance to the first occupied/out-of-window cell along a direction."""
        if self._costmap_data is None:
            return COSTMAP_MAX_LOOKAHEAD
        data = self._costmap_data
        ox, oy, yaw = self._map_to_costmap_offset()
        if ox is None:
            return 0.0
        origin_x = data.info.origin.position.x
        origin_y = data.info.origin.position.y
        res = data.info.resolution
        w, h = data.info.width, data.info.height
        rx, ry = robot_xy[0], robot_xy[1]
        cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        steps = int(math.ceil(COSTMAP_MAX_LOOKAHEAD / COSTMAP_LOOKAHEAD_STEP))
        for step in range(1, steps + 1):
            d = step * COSTMAP_LOOKAHEAD_STEP
            px = rx + d * cos_a
            py = ry + d * sin_a
            cx = ox + cos_yaw * px - sin_yaw * py
            cy = oy + sin_yaw * px + cos_yaw * py
            col = int((cx - origin_x) / res)
            row = int((cy - origin_y) / res)
            if col < 0 or col >= w or row < 0 or row >= h:
                return d - COSTMAP_LOOKAHEAD_STEP
            idx = row * w + col
            if idx < 0 or idx >= len(data.data):
                return d - COSTMAP_LOOKAHEAD_STEP
            if data.data[idx] >= COSTMAP_OCCUPIED:
                return d - COSTMAP_LOOKAHEAD_STEP
        return COSTMAP_MAX_LOOKAHEAD

    # ------------------------------------------------------------------
    # Active safety evasion
    # ------------------------------------------------------------------

    def _is_blocked_by_costmap(
        self, robot_xy: tuple, angle_rad: float, lookahead: float = 1.0
    ) -> bool:
        """Check whether moving *lookahead* metres along *angle_rad* hits an obstacle."""
        if self._costmap_data is None:
            return False
        data = self._costmap_data
        ox, oy, yaw = self._map_to_costmap_offset()
        if ox is None:
            return True
        px = robot_xy[0] + lookahead * math.cos(angle_rad)
        py = robot_xy[1] + lookahead * math.sin(angle_rad)
        cx = ox + math.cos(yaw) * px - math.sin(yaw) * py
        cy = oy + math.sin(yaw) * px + math.cos(yaw) * py
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
        ghost_xy = (ghost.pose.position.x, ghost.pose.position.y)
        chosen, clearance = self._select_evasion_heading(
            robot_xy, ghost_xy, dist
        )

        if chosen is None:
            # 无满足最小净空的安全避让方向:停驻,不朝障碍/墙角硬推.
            # (硬推会顶进静态膨胀区,令 global planner 起点失效 → 巡航 FAILED.)
            self._publish_zero_velocity()
            self._last_safety_cmd = (0.0, 0.0, 0.0, 0.0)
            now = self._now
            if (self._last_evasion_log_time is None
                    or now - self._last_evasion_log_time >= 0.5):
                self._last_evasion_log_time = now
                self.get_logger().info(
                    "evasion hold (no safe heading, blocked): "
                    f"robot=({robot[0]:.2f},{robot[1]:.2f}) "
                    f"ghost=({ghost_xy[0]:.2f},{ghost_xy[1]:.2f}) "
                    f"dist={dist:.2f}"
                )
            return

        yaw = robot[2]
        vx = speed * math.cos(chosen)
        vy = speed * math.sin(chosen)
        msg = Twist()
        msg.linear.x = vx * math.cos(yaw) + vy * math.sin(yaw)
        msg.linear.y = -vx * math.sin(yaw) + vy * math.cos(yaw)
        self._last_safety_cmd = (
            msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z,
        )
        self._safety_cmd_pub.publish(msg)

        now = self._now
        if (self._last_evasion_log_time is None
                or now - self._last_evasion_log_time >= 0.5):
            self._last_evasion_log_time = now
            self.get_logger().info(
                "evade: "
                f"robot=({robot[0]:.2f},{robot[1]:.2f}) "
                f"ghost=({ghost_xy[0]:.2f},{ghost_xy[1]:.2f}) "
                f"dist={dist:.2f} chosen={math.degrees(chosen):.1f}deg "
                f"clear={clearance:.2f}m "
                f"gv=({self._ghost_vx:.2f},{self._ghost_vy:.2f}) "
                f"close={self._closing_speed:.2f}m/s"
            )

    def _select_evasion_heading(
        self, robot_xy: tuple, ghost_xy: tuple, dist: float
    ) -> tuple:
        """Pick a stable evasion direction, preferring lateral stepping."""
        dx = robot_xy[0] - ghost_xy[0]
        dy = robot_xy[1] - ghost_xy[1]
        away_angle = math.atan2(dy, dx)
        gspeed = math.hypot(self._ghost_vx, self._ghost_vy)
        if gspeed > 0.05:
            # 与 ghost 前进方向垂直的两个侧向.
            side_a = math.atan2(self._ghost_vx, -self._ghost_vy)
            side_b = math.atan2(-self._ghost_vx, self._ghost_vy)
        else:
            side_a = away_angle + math.pi / 2
            side_b = away_angle - math.pi / 2

        head = self._preferred_evasion_heading()
        plan_active = self._nav2_active

        candidates = []
        for offset in range(-10, 11):
            candidates.append(away_angle + offset * EVASION_CANDIDATE_SPACING)
        if head is not None:
            candidates.append(head)
            candidates.append(head + math.pi / 2)
            candidates.append(head - math.pi / 2)
        candidates.append(side_a)
        candidates.append(side_b)

        def score(angle: float) -> Optional[tuple]:
            clearance = self._costmap_clearance(robot_xy, angle)
            if clearance <= 0.0 or clearance < EVASION_MIN_CLEARANCE:
                # 净空不足 → 视为被挡(含静态障碍膨胀区/墙角),
                # 避免“落点占用/蠕动逼近”把底盘顶进无法恢复的死角。
                return None
            look = min(clearance, 0.9)
            px = robot_xy[0] + look * math.cos(angle)
            py = robot_xy[1] + look * math.sin(angle)
            dt = look / max(EVASION_PREDICT_SPEED, 1e-3)
            pred_sep = math.hypot(
                px - self._future_ghost_xy(dt, ghost_xy)[0],
                py - self._future_ghost_xy(dt, ghost_xy)[1],
            )
            future_sep = self._future_separation(robot_xy, angle, ghost_xy)
            sep_score = min(pred_sep, 5.0)
            sep_gain = max(0.0, pred_sep - dist)
            lateral_bonus = 0.0
            if dist < SAFETY_CRITICAL_DIST:
                lateral_delta = min(
                    abs(math.atan2(
                        math.sin(angle - side_a), math.cos(angle - side_a)
                    )),
                    abs(math.atan2(
                        math.sin(angle - side_b), math.cos(angle - side_b)
                    )),
                )
                if lateral_delta <= EVASION_LATERAL_TOLERANCE:
                    # 越接近纯侧向加分越高,让候选网格不会因为微小预测差异
                    # 把机器人带到半侧向、半跟着 ghost 的角度上.
                    lateral_bonus = EVASION_LATERAL_BONUS * (
                        1.0 - lateral_delta / EVASION_LATERAL_TOLERANCE
                    )
            # 让行方向朝当前局部规划指向时加分:避让不是“逃跑”,而是
            # 在保持净空的前提下尽量不脱离 Nav2 的可规划走廊。
            align = 0.0
            if head is not None:
                dot = math.cos(angle - head)
                if dot > 0.0:
                    align = PLAN_ALIGN_BONUS * dot
                    if (plan_active
                            and pred_sep >= PLAN_SAFE_PREDICTED_SEPARATION
                            and future_sep >= PLAN_SAFE_PREDICTED_SEPARATION):
                        align *= PLAN_ALIGN_ACTIVE_FACTOR
            return (
                clearance,
                clearance + 1.2 * sep_score + 1.5 * sep_gain
                + EVASION_FUTURE_SEP_WEIGHT * future_sep
                + align
                + lateral_bonus
                - EVASION_COLLISION_PENALTY_FACTOR * max(
                    0.0, SAFETY_CRITICAL_DIST - future_sep
                ),
            )

        def _pick_best(cands) -> Optional[tuple]:
            best: Optional[tuple] = None
            best_score = -float("inf")
            for angle in set(cands):
                s = score(angle)
                if s is None:
                    continue
                if s[1] > best_score:
                    best_score = s[1]
                    best = (angle, s[0])
            return best

        # 近距时优先侧向让行:迎面场景下“远离”只是继续相向而行,
        # 只有与 ghost 运动方向垂直的侧移才能有效拉开距离。
        if dist < SAFETY_CRITICAL_DIST:
            # 先在两个纯侧向之间按完整评分选择;两个侧向都被 costmap
            # 封死时才退到完整候选集,避免跟着 ghost 同向追行.
            best = _pick_best([side_a, side_b])
            if best is None:
                best = _pick_best(candidates)
            if best is not None:
                self._evade_heading = best[0]
                self._evade_heading_set_at = self._now
                self._evade_heading_dist = dist
                return best[0], best[1]
            # 全部候选(含两个纯侧向)都被 costmap 封死:不朝障碍硬推,
            # 原地停驻等待 ghost 通过,避免把车推出可规划走廊/顶进静态死角.
            self._evade_heading = None
            self._evade_heading_set_at = None
            self._evade_heading_dist = None
            return None, 0.0

        now = self._now
        if (self._evade_heading is not None
                and self._evade_heading_set_at is not None
                and now - self._evade_heading_set_at < EVASION_HEADING_HOLD_SEC
                and self._evade_heading_dist is not None
                and dist <= self._evade_heading_dist + 0.10):
            s = score(self._evade_heading)
            if s is not None and s[0] >= 0.35:
                return self._evade_heading, s[0]

        best = _pick_best(candidates)
        if best is not None:
            self._evade_heading = best[0]
            self._evade_heading_set_at = now
            self._evade_heading_dist = dist
            return best[0], best[1]

        # 极端场景所有候选方向都被 costmap 阻挡:停驻而非朝障碍硬推.
        # 硬推会把底盘顶进桌腿/墙角(global costmap 起点判占用),使巡航
        # 在同站重试时全局规划永久失败;停驻等 ghost 离开后 DWB 再恢复即可.
        self._evade_heading = None
        self._evade_heading_set_at = None
        self._evade_heading_dist = None
        return None, 0.0

    def _preferred_evasion_heading(self) -> Optional[float]:
        """当前局部规划的 heading;没有规划时不附加路径偏向加分."""
        return _plan_heading(self._local_plan) if self._local_plan else None

    def _publish_zero_velocity(self) -> None:
        self._safety_cmd_pub.publish(Twist())

    def _begin_event(self, dist: Optional[float]) -> None:
        """Start metric timestamps if an avoidance event is not yet active."""
        if self._avoidance_active:
            return
        now = self._now
        self._avoidance_active = True
        self._event_start = now
        self._t_detect = now
        self._t_mark = None
        self._t_deviate = None
        self._min_distance = dist if dist is not None else float("inf")
        if self._local_plan is not None:
            heading = _plan_heading(self._local_plan)
            if heading is not None:
                self._baseline_heading = heading
        self.get_logger().info(
            f"detection: ghost entered range at dist={dist:.2f}m"
        )

    def _handle_safety_evasion(self, dist: Optional[float]) -> None:
        """Decide whether to actively evade and publish commands."""
        if dist is None:
            # 仅当 ghost 确实丢失(actor 消失)时才停避让;
            # robot TF 短暂失败时保持避让状态,避免静止被 ghost 穿透.
            if self._ghost_pose is None and self._evading:
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

        # 接管不是无界“逃跑”:连续避让超过阈值后释放给 Nav2,
        # 避免把机器人推进 costmap 与地图都无法恢复的边角区。
        if (self._evading and self._t_evade is not None
                and self._now - self._t_evade > EVASION_TIMEOUT_SEC
                and dist >= SAFETY_CRITICAL_DIST + 0.15):
            need_evade = False
            self._t_evade_release = self._now
            self._t_evade = None
            self._evade_heading = None
            self._evade_heading_set_at = None
            self._evade_heading_dist = None

        if need_evade:
            self._publish_evasion(evade_speed)
            if not self._evading:
                self._t_evade_release = None
                if self._t_evade is None:
                    self._t_evade = self._now
                    self._t_cmd_vel = None
                    self._t_safety_cmd_pub = self._now
                    self._t_warn_detected = self._now
                    self._t_warn_stamp = self._ghost_stamp_seconds()
                self._begin_event(dist)
                self.get_logger().info(
                    f"safety evasion ON: dist={dist:.2f}m speed={evade_speed:.2f}m/s"
                )
            self._evading = True
        elif self._evading:
            self._publish_zero_velocity()
            self.get_logger().info("safety evasion OFF: ghost cleared")
            self._evading = False
            self._evade_heading = None
            self._evade_heading_set_at = None
            self._evade_heading_dist = None

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
            self._begin_event(dist)

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

        def _latency(start, end):
            if start is None or end is None:
                return None
            value = round(1000.0 * (end - start), 1)
            return value if value >= 0 else None

        event = {
            "t_detect": self._event_start,
            "t_mark": self._t_mark,
            "t_deviate": self._t_deviate,
            "t_evade": self._t_evade,
            "t_clear": now,
            "perception_latency_ms": round(1000 * (self._t_mark - self._event_start), 1)
            if self._t_mark else None,
            "replan_latency_ms": round(1000 * (self._t_deviate - self._t_mark), 1)
            if self._t_deviate and self._t_mark else None,
            "total_response_ms": round(1000 * (self._t_deviate - self._event_start), 1)
            if self._t_deviate else None,
            "safety_response_ms": round(1000 * (self._t_evade - self._event_start), 1)
            if self._t_evade else None,
            "sensor_to_safety_ms": _latency(
                self._t_warn_stamp, self._t_safety_cmd_pub
            ),
            "sensor_to_cmd_ms": _latency(
                self._t_warn_stamp, self._t_cmd_vel
            ),
            "mux_hop_ms": _latency(
                self._t_safety_cmd_pub, self._t_cmd_vel
            ),
            "control_loop_ms": _latency(
                self._t_warn_detected, self._t_cmd_vel
            ),
            "total_duration_ms": round(1000 * (now - self._event_start), 1),
            "min_distance_m": round(self._min_distance, 3),
        }
        self._events.append(event)
        msg = String(data=json.dumps(event))
        self._metrics_pub.publish(msg)
        try:
            with open(self._events_file, "a", encoding="utf-8") as _fp:
                _fp.write(json.dumps(event, ensure_ascii=False) + "\n")
        except OSError:
            pass
        self.get_logger().info(
            f"event #{len(self._events)}: response={event['total_response_ms']}ms "
            f"min_dist={event['min_distance_m']}m "
            f"reactive_cmd={event['sensor_to_cmd_ms']}ms "
            f"mux_hop={event['mux_hop_ms']}ms "
            f"control_loop={event['control_loop_ms']}ms "
            f"perception={event['perception_latency_ms']}ms "
            f"replan={event['replan_latency_ms']}ms"
        )
        self._avoidance_active = False
        self._event_start = None
        self._t_detect = None
        self._t_mark = None
        self._t_deviate = None
        self._t_evade = None
        self._t_safety_cmd_pub = None
        self._t_cmd_vel = None
        self._t_warn_detected = None
        self._t_warn_stamp = None
        self._last_safety_cmd = None
        self._baseline_heading = None
        self._min_distance = float("inf")
        self._evade_heading = None
        self._evade_heading_set_at = None
        self._evade_heading_dist = None
        self._closing_speed = 0.0
        self._t_evade_release = None
        self._evading = False


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
