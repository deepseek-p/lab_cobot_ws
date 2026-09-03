"""Contracts for actor shadow reliability and safety velocity arbitration."""
import math
from pathlib import Path

import pytest


GAZEBO = Path(__file__).resolve().parents[1]


def test_actor_shadow_retries_gazebo_service_without_startup_block():
    source = (GAZEBO / "scripts" / "actor_collision_shadow.py").read_text(
        encoding="utf-8"
    )

    assert "wait_for_service" not in source
    assert "service_is_ready()" in source
    assert "SetEntityState" in source
    assert '"/gazebo/set_entity_state"' in source
    assert "SetModelState" not in source
    assert 'GHOST_ENTITY_NAME = "actor_ghost_collision::body"' in source
    assert 'req.state.reference_frame = "world"' in source
    assert "_gazebo_connected" in source
    assert "/actor_ghost/obstacle_cloud" in source


def test_actor_shadow_uses_live_gazebo_actor_pose_not_a_duplicate_trajectory():
    source = (GAZEBO / "scripts" / "actor_collision_shadow.py").read_text(
        encoding="utf-8"
    )

    assert "ModelStates" in source
    assert 'ACTOR_MODEL_NAME = "test_engineer_actor"' in source
    assert '"/gazebo/model_states"' in source
    assert "self._actor_pose" in source
    assert "DEFAULT_WAYPOINTS" not in source


def test_metrics_publishes_safety_velocity_on_dedicated_topic():
    source = (GAZEBO / "scripts" / "obstacle_avoidance_metrics.py").read_text(
        encoding="utf-8"
    )

    assert 'create_publisher(Twist, "/cmd_vel_safety", 10)' in source
    assert 'create_publisher(Twist, "/cmd_vel_nav", 10)' not in source


def test_cmd_vel_safety_mux_owns_final_cmd_vel():
    source = (GAZEBO / "scripts" / "cmd_vel_safety_mux.py").read_text(
        encoding="utf-8"
    )

    assert '"/cmd_vel_nav_smoothed"' in source
    assert '"/cmd_vel_safety"' in source
    assert '"/cmd_vel"' in source
    assert "SAFETY_TIMEOUT_SEC" in source


def test_cmd_vel_safety_mux_arbitrates_manual_below_safety():
    source = (GAZEBO / "scripts" / "cmd_vel_safety_mux.py").read_text(
        encoding="utf-8"
    )

    assert '"/cmd_vel_manual"' in source
    assert "MANUAL_TIMEOUT_SEC" in source
    assert "self._manual_cmd" in source


def test_safety_mux_priority_safety_gt_manual_gt_nav():
    import rclpy
    from geometry_msgs.msg import Twist

    import sys
    sys.path.insert(0, str(GAZEBO / "scripts"))
    import cmd_vel_safety_mux as mux

    if not rclpy.ok():
        rclpy.init()
    try:
        node = mux.CmdVelSafetyMux()
        published = []
        node._pub.publish = lambda msg: published.append(msg)
        fixed = {"t": 1000.0}

        class FakeClock:
            def now(self):
                return type(
                    "Now", (), {"nanoseconds": int(fixed["t"] * 1e9)}
                )()

        node.get_clock = lambda: FakeClock()

        nav = Twist()
        nav.linear.x = 0.3
        manual = Twist()
        manual.linear.x = -0.2
        safety = Twist()
        safety.linear.y = 0.5

        node._nav_cb(nav)
        node._manual_cb(manual)
        node._tick()
        assert published[-1].linear.x == pytest.approx(-0.2)

        node._safety_cb(safety)
        node._tick()
        assert published[-1].linear.y == pytest.approx(0.5)

        # Safety release resumes the still-fresh manual command.
        node._last_safety_time = None
        node._safety_cmd = Twist()
        node._tick()
        assert published[-1].linear.x == pytest.approx(-0.2)

        # Fresh zero manual holds the robot instead of letting Nav2 drive.
        node._manual_cb(Twist())
        node._tick()
        assert published[-1].linear.x == pytest.approx(0.0)
        assert published[-1].linear.y == pytest.approx(0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_safety_mux_zero_command_releases_nav2():
    source = (GAZEBO / "scripts" / "cmd_vel_safety_mux.py").read_text(
        encoding="utf-8"
    )

    assert "def _nonzero" in source
    assert "A zero safety message means \"release override\"" in source
    assert "and self._nonzero(self._safety_cmd)" in source


def test_safety_evasion_starts_with_safe_clearance_and_uses_emergency_speed():
    source = (GAZEBO / "scripts" / "obstacle_avoidance_metrics.py").read_text(
        encoding="utf-8"
    )

    assert "GHOST_RADIUS = 0.60" in source
    assert "SAFETY_WARN_DIST = 2.00" in source
    assert "SAFETY_CRITICAL_DIST = 1.20" in source
    assert "SAFETY_EVADE_CRITICAL_SPEED = 0.60" in source
    assert "SAFETY_EVADE_HYSTERESIS = 0.40" in source
    assert "EVASION_HEADING_HOLD_SEC" in source


def test_safety_evasion_prefers_lateral_step_when_ghost_is_close():
    source = (GAZEBO / "scripts" / "obstacle_avoidance_metrics.py").read_text(
        encoding="utf-8"
    )

    assert "def _select_evasion_heading" in source
    assert "dist < SAFETY_CRITICAL_DIST" in source
    # 近距也要评估完整候选集,不能只比较两个侧向后盲目开出,
    # 否则迎面场景会选错侧向、跟着 ghost 同向运动直到碰撞.
    close_branch = source.split("if dist < SAFETY_CRITICAL_DIST:", 1)[1]
    assert "best = _pick_best([side_a, side_b])" in close_branch.split(
        "if self._evade_heading is not None", 1
    )[0]
    assert "best = _pick_best(candidates)" in close_branch.split(
        "if self._evade_heading is not None", 1
    )[0]
    assert "math.atan2(self._ghost_vx, -self._ghost_vy)" in source
    assert "math.atan2(-self._ghost_vx, self._ghost_vy)" in source
    assert "self._evade_heading" in source


def test_safety_evasion_lateral_heading_is_perpendicular_to_ghost_velocity():
    import rclpy

    import sys
    sys.path.insert(0, str(GAZEBO / "scripts"))
    import obstacle_avoidance_metrics as metrics

    if not rclpy.ok():
        rclpy.init()
    try:
        node = metrics.AvoidanceMetrics()
        node._ghost_vx = 1.0
        node._ghost_vy = 0.0
        heading, _ = node._select_evasion_heading(
            (0.0, 0.0), (1.0, 0.0), 1.0
        )
        assert abs(math.cos(heading)) < 0.05
        assert abs(math.sin(heading)) > 0.99
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_safety_evasion_prefers_local_plan_heading_when_clear():
    import rclpy
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped

    import sys
    sys.path.insert(0, str(GAZEBO / "scripts"))
    import obstacle_avoidance_metrics as metrics

    if not rclpy.ok():
        rclpy.init()
    try:
        node = metrics.AvoidanceMetrics()
        plan = Path()
        p0 = PoseStamped()
        p0.pose.position.x = 0.0
        p0.pose.position.y = 0.0
        p1 = PoseStamped()
        p1.pose.position.x = 2.0
        p1.pose.position.y = 0.0
        plan.poses = [p0, p1]
        node._local_plan = plan
        node._ghost_vx = -0.8
        node._ghost_vy = 0.6
        # Ghost is behind the robot, so the local-plan direction (+x) is both
        # clear and moves away from it. The evasion heading should not drift
        # toward an off-path map corner just because the ghost exists.
        heading, clearance = node._select_evasion_heading(
            (0.0, 0.0), (-1.8, 0.0), 1.8
        )
        diff = abs(math.atan2(
            math.sin(heading), math.cos(heading)
        ))
        assert diff < math.radians(30) or diff > math.radians(330)
        assert clearance > 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_safety_evasion_keeps_active_nav2_plan_heading_when_separation_safe():
    import rclpy
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped

    import sys
    sys.path.insert(0, str(GAZEBO / "scripts"))
    import obstacle_avoidance_metrics as metrics

    if not rclpy.ok():
        rclpy.init()
    try:
        node = metrics.AvoidanceMetrics()
        plan = Path()
        p0 = PoseStamped()
        p0.pose.position.x = 0.0
        p0.pose.position.y = 0.0
        p1 = PoseStamped()
        p1.pose.position.x = 2.0
        p1.pose.position.y = 0.0
        plan.poses = [p0, p1]
        node._local_plan = plan
        node._ghost_vx = -0.8
        node._ghost_vy = 0.6
        node._last_nav2_cmd_time = node._now
        # Nav2 is actively commanding motion and the ghost is behind the robot.
        # The evasion heading should strongly prefer the local-plan direction
        # when the predicted separation after that step stays safe.
        heading, clearance = node._select_evasion_heading(
            (0.0, 0.0), (-1.8, 0.0), 1.8
        )
        diff = abs(math.atan2(
            math.sin(heading), math.cos(heading)
        ))
        assert diff < math.radians(15) or diff > math.radians(345)
        assert clearance > 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_actor_shadow_cloud_radius_keeps_corridors_navigable():
    source = (GAZEBO / "scripts" / "actor_collision_shadow.py").read_text(
        encoding="utf-8"
    )

    assert "GHOST_RADIUS = 0.60" in source


def test_actor_shadow_publishes_visible_debug_marker():
    source = (GAZEBO / "scripts" / "actor_collision_shadow.py").read_text(
        encoding="utf-8"
    )

    assert "visualization_msgs.msg" in source
    assert '"/actor_ghost/debug_marker"' in source
    assert "Marker.CYLINDER" in source
    assert "marker.color.r = 1.0" in source


def test_safety_evasion_checks_costmap_before_driving():
    source = (GAZEBO / "scripts" / "obstacle_avoidance_metrics.py").read_text(
        encoding="utf-8"
    )

    assert "def _is_blocked_by_costmap" in source
    assert "candidates" in source
    assert "COSTMAP_OCCUPIED" in source


def test_safety_evasion_tracks_ghost_velocity_for_predictive_evasion():
    source = (GAZEBO / "scripts" / "obstacle_avoidance_metrics.py").read_text(
        encoding="utf-8"
    )

    assert "self._ghost_vx" in source
    assert "self._ghost_vy" in source
    assert "self._last_ghost_xy" in source
    assert "gspeed" in source


def test_metrics_ignores_idle_robot_proximity():
    source = (GAZEBO / "scripts" / "obstacle_avoidance_metrics.py").read_text(
        encoding="utf-8"
    )

    assert "if not self._safety_override_allowed():" in source
    assert "return" in source


def test_metrics_measures_reactive_latency_to_final_cmd_vel():
    source = (GAZEBO / "scripts" / "obstacle_avoidance_metrics.py").read_text(
        encoding="utf-8"
    )

    assert 'Twist, "/cmd_vel", self._cmd_vel_cb, 10' in source
    assert "def _safety_matches" in source
    assert "sensor_to_safety_ms" in source
    assert "sensor_to_cmd_ms" in source
    assert "mux_hop_ms" in source
    assert "control_loop_ms" in source


def test_safety_evasion_translates_map_points_into_costmap_frame():
    source = (GAZEBO / "scripts" / "obstacle_avoidance_metrics.py").read_text(
        encoding="utf-8"
    )

    assert "def _map_to_costmap_offset" in source
    assert 'lookup_transform(costmap_frame, "map"' in source or (
        'self._tf_buffer.lookup_transform(\n                costmap_frame, "map", '
        "rclpy.time.Time()\n            )" in source
    )
    assert "def _costmap_clearance" in source
    assert "EVASION_CANDIDATE_SPACING" in source


def test_costmap_query_uses_map_to_odom_transform():
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid

    import sys
    sys.path.insert(0, str(GAZEBO / "scripts"))
    import obstacle_avoidance_metrics as metrics

    if not rclpy.ok():
        rclpy.init()
    try:
        node = metrics.AvoidanceMetrics()

        class FakeTransform:
            def __init__(self):
                self.transform = TransformStamped().transform
                self.transform.translation.x = -4.5
                self.transform.translation.y = 4.2
                self.transform.rotation.w = 1.0

        fake = FakeTransform()
        node._tf_buffer.lookup_transform = lambda _a, _b, _c: fake

        grid = OccupancyGrid()
        grid.header.frame_id = "odom"
        grid.info.resolution = 0.05
        grid.info.width = 140
        grid.info.height = 140
        grid.info.origin.position.x = -3.5
        grid.info.origin.position.y = -3.5
        grid.data = [0] * (140 * 140)
        node._costmap_data = grid

        robot_xy = (4.5, -4.2)
        # Map (5.5, -4.2) -> odom (1.0, 0.0), inside the empty rolling window.
        assert not node._is_blocked_by_costmap(robot_xy, 0.0, lookahead=1.0)
        assert not node._costmap_clearance(robot_xy, 0.0) < 1.0
        # Rapid sweep over map (4.5..5.7, -4.2..-3.0) also stays inside odom grid.
        clearance = node._costmap_clearance(robot_xy, math.pi / 4)
        assert clearance >= 0.8

        ghost_col = int((0.0 - grid.info.origin.position.x) / grid.info.resolution)
        ghost_row = int((1.0 - grid.info.origin.position.y) / grid.info.resolution)
        grid.data[ghost_row * grid.info.width + ghost_col] = 100
        fake.transform.rotation.w = 1.0
        ghost = PoseStamped()
        ghost.pose.position.x = 4.5
        ghost.pose.position.y = -3.2
        node._ghost_pose = ghost
        assert node._is_ghost_in_costmap()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_actor_shadow_updates_from_each_live_actor_model_state():
    source = (GAZEBO / "scripts" / "actor_collision_shadow.py").read_text(
        encoding="utf-8"
    )

    assert "def _on_model_states" in source
    assert "self._publish_actor_state" in source
    assert "_set_state_request_in_flight" in source


def test_actor_world_collision_radius_matches_safety_cloud_radius():
    for world_name in (
        "lab_actor.world",
        "lab_dark_actor.world",
        "lab_reflective_actor.world",
    ):
        world = (GAZEBO / "worlds" / world_name).read_text(encoding="utf-8")
        ghost = world.split('<model name="actor_ghost_collision">', 1)[1]
        assert "<radius>0.60</radius>" in ghost.split("</model>", 1)[0]


def test_safety_override_allows_arrived_state_when_ghost_is_critical():
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped

    import sys
    sys.path.insert(0, str(GAZEBO / "scripts"))
    import obstacle_avoidance_metrics as metrics

    if not rclpy.ok():
        rclpy.init()
    try:
        node = metrics.AvoidanceMetrics()
        node._task_status = "ARRIVED:station_b"

        # Robot at map (1.0, 0.0), ghost at (2.0, 0.0): the clearance after
        # subtracting both safety radii is below SAFETY_CRITICAL_DIST, so the
        # ARRIVED pause between stations must not become an avoidance blind spot.
        class FakeTransform:
            def __init__(self):
                self.transform = TransformStamped().transform
                self.transform.translation.x = 1.0
                self.transform.translation.y = 0.0
                self.transform.rotation.w = 1.0

        node._tf_buffer.lookup_transform = lambda _a, _b, _c: FakeTransform()
        ghost = PoseStamped()
        ghost.pose.position.x = 2.0
        ghost.pose.position.y = 0.0
        node._ghost_pose = ghost

        assert node._safety_override_allowed() is True
    finally:
        node.destroy_node()
        rclpy.shutdown()
