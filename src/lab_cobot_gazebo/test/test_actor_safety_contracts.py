"""Contracts for actor shadow reliability and safety velocity arbitration."""
from pathlib import Path


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
    assert "SAFETY_WARN_DIST = 1.00" in source
    assert "SAFETY_CRITICAL_DIST = 0.70" in source
    assert "SAFETY_EVADE_CRITICAL_SPEED = 0.55" in source
    assert "SAFETY_EVADE_HYSTERESIS = 0.25" in source


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
