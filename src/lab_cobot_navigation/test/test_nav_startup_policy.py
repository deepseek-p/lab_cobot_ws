"""Nav2 启动就绪策略单测(纯逻辑,无 ROS 运行时)."""
from lab_cobot_navigation.nav_startup_policy import (
    AMCL_NAME,
    LIFECYCLE_ACTIVATE,
    LIFECYCLE_CONFIGURE,
    MAP_SERVER_NAME,
    Readiness,
    decide_recovery,
    transitions_to_active,
)


def test_unconfigured_needs_configure_then_activate():
    assert transitions_to_active(1) == (LIFECYCLE_CONFIGURE, LIFECYCLE_ACTIVATE)


def test_inactive_only_needs_activate():
    assert transitions_to_active(2) == (LIFECYCLE_ACTIVATE,)


def test_active_needs_no_transition():
    assert transitions_to_active(3) == ()


def test_unknown_state_defaults_to_full_recovery():
    assert transitions_to_active(0) == (LIFECYCLE_CONFIGURE, LIFECYCLE_ACTIVATE)


def test_missing_map_recovery_targets_map_server():
    readiness = Readiness(
        map_received=False,
        map_to_odom_ready=False,
        lifecycle_states={},
    )
    plan = decide_recovery(readiness)

    assert plan is not None
    assert plan.node_name == MAP_SERVER_NAME


def test_missing_tf_recovery_targets_amcl():
    readiness = Readiness(
        map_received=True,
        map_to_odom_ready=False,
        lifecycle_states={},
    )
    plan = decide_recovery(readiness)

    assert plan is not None
    assert plan.node_name == AMCL_NAME


def test_ready_has_no_recovery():
    readiness = Readiness(
        map_received=True,
        map_to_odom_ready=True,
        lifecycle_states={},
    )

    assert decide_recovery(readiness) is None
