"""Mission navigation handoff policy tests."""
import math

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from lab_cobot_bringup import mission_node
from lab_cobot_bringup import mecanum_wheel_visualizer


class _FakeRosNow:
    def __init__(self, stamp):
        self._stamp = stamp

    def to_msg(self):
        return self._stamp


class _SequenceClock:
    def __init__(self, stamps):
        self._stamps = iter(stamps)

    def now(self):
        return _FakeRosNow(next(self._stamps))


def _policy(name):
    assert hasattr(mission_node, name), f"{name} policy is missing"
    return getattr(mission_node, name)


def test_mission_local_docking_commands_reach_mecanum_driver():
    assert mission_node.RETREAT_TOPIC == mecanum_wheel_visualizer.CMD_VEL_TOPIC


def test_base_pose_from_odom_msg_extracts_planar_pose():
    msg = Odometry()
    msg.pose.pose.position.x = 1.2
    msg.pose.pose.position.y = -0.4
    yaw = math.radians(90.0)
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

    assert mission_node.base_pose_from_odom_msg(msg) == pytest.approx([1.2, -0.4, yaw])


def test_raw_odom_cache_remains_available_for_non_safety_consumers():
    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    node._latest_odom_pose = [2.0, 0.62, math.pi / 2.0]

    assert mission_node.MissionNode._base_pose_in_odom(node) == node._latest_odom_pose


def test_station_safety_uses_map_pose_when_map_and_odom_are_offset():
    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    safe_y = mission_node.station_safe_base_y(math.pi / 2.0, "station_a")
    calls = []
    node.get_logger = lambda: type(
        "Logger", (), {"info": lambda *args: None, "warn": lambda *args: None}
    )()
    node.get_clock = lambda: type("Clock", (), {"now": lambda self: object()})()
    node._duration_elapsed = lambda *_args: False
    node._base_pose_in_map = lambda timeout_sec=2.0: [2.0, safe_y, math.pi / 2.0]
    node._base_pose_in_odom = lambda timeout_sec=2.0: (_ for _ in ()).throw(
        AssertionError("map safety must not consume offset odom coordinates")
    )
    node._stop_base = lambda duration: calls.append(duration)

    assert mission_node.MissionNode._dock_to_station_pose(node, "station_a")
    assert calls == [mission_node.STATION_DOCK_STOP_SEC]


def test_pick_visual_safety_uses_map_pose_when_map_and_odom_are_offset():
    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    safe_y = mission_node.station_safe_base_y(math.pi / 2.0, "station_a")
    node.get_logger = lambda: type(
        "Logger", (), {"info": lambda *args: None, "warn": lambda *args: None}
    )()
    node.get_clock = lambda: type("Clock", (), {"now": lambda self: object()})()
    node._duration_elapsed = lambda *_args: False
    node._detect = lambda: [
        mission_node.DOCK_SAFE_HANDOFF_MAX_X - 0.01,
        0.0,
        0.63,
    ]
    node._base_pose_in_map = lambda timeout_sec=2.0: [2.0, safe_y, math.pi / 2.0]
    node._base_pose_in_odom = lambda timeout_sec=2.0: (_ for _ in ()).throw(
        AssertionError("map safety must not consume offset odom coordinates")
    )
    node._stop_base = lambda _duration: None

    assert mission_node.MissionNode._dock_to_pick_target(node)


def test_detect_uses_fresh_perception_pose_topic_before_tf_lookup():
    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    node.object_id = 0
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.header.stamp = Time(sec=20, nanosec=0)
    pose.pose.position.x = 0.82
    pose.pose.position.y = -0.01
    pose.pose.position.z = 0.79
    node._latest_detection_pose = pose
    now_msg = Time(sec=20, nanosec=200_000_000)
    node.get_clock = lambda: type(
        "Clock",
        (),
        {"now": lambda self: type("Now", (), {"to_msg": lambda self: now_msg})()},
    )()
    node.get_logger = lambda: type("Logger", (), {"warn": lambda *args, **kwargs: None})()

    class FailingTfBuffer:
        def lookup_transform(self, *_args, **_kwargs):
            raise AssertionError("fresh perception topic should be used before TF")

    node.tf_buffer = FailingTfBuffer()

    assert mission_node.MissionNode._detect(node, timeout_sec=0.0) == pytest.approx([
        0.82,
        -0.01,
        0.79,
    ])


def test_refine_callback_waits_for_sample_newer_than_callback_start(monkeypatch):
    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    node._latest_wrist_detection_pose = None
    clock = _SequenceClock([
        Time(sec=10, nanosec=0),
        Time(sec=10, nanosec=200_000_000),
    ])
    node.get_clock = lambda: clock
    node._duration_elapsed = lambda _start, _duration: False

    def publish_new_sample(_duration):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = Time(sec=10, nanosec=100_000_000)
        pose.pose.position.x = 0.82
        pose.pose.position.y = -0.01
        pose.pose.position.z = 0.79
        node._latest_wrist_detection_pose = pose

    monkeypatch.setattr(mission_node.time, "sleep", publish_new_sample)

    refine_cb = mission_node.MissionNode._make_refine_cb(node)

    assert refine_cb() == pytest.approx([0.82, -0.01, 0.79])


def test_refine_callback_rejects_cache_older_than_callback_start(monkeypatch):
    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.header.stamp = Time(sec=9, nanosec=900_000_000)
    node._latest_wrist_detection_pose = pose
    clock = _SequenceClock([Time(sec=10, nanosec=0)])
    node.get_clock = lambda: clock
    elapsed = iter([False, True])
    node._duration_elapsed = lambda _start, _duration: next(elapsed)
    monkeypatch.setattr(mission_node.time, "sleep", lambda _duration: None)

    refine_cb = mission_node.MissionNode._make_refine_cb(node)

    assert refine_cb() is None


@pytest.mark.parametrize("enabled", [False, True])
def test_pick_state_passes_refine_callback_only_when_enabled(enabled):
    calls = []
    callback = object()

    class FakePickPlace:
        def pick(self, pose, refine_cb=None):
            calls.append((list(pose), refine_cb))
            return True

    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    node.pp = FakePickPlace()
    node._use_refine_detect = enabled
    node._detect = lambda: [0.8, 0.0, 0.78]
    node._make_refine_cb = lambda: callback
    node._finish_station_step = lambda _state, ok: ok

    assert mission_node.MissionNode._execute(node, mission_node.TaskState.PICK)
    assert calls == [([0.8, 0.0, 0.78], callback if enabled else None)]


def test_pick_navigation_can_handoff_when_visual_docking_can_finish_alignment():
    pick_navigation_handoff_ready = _policy("pick_navigation_handoff_ready")

    assert pick_navigation_handoff_ready([0.80, 0.10, 0.63])


def test_pick_navigation_keeps_nav2_for_gui_failed_lateral_offset():
    pick_navigation_handoff_ready = _policy("pick_navigation_handoff_ready")

    assert not pick_navigation_handoff_ready([0.793, 0.235, 0.63])


def test_pick_navigation_keeps_nav2_when_object_is_too_lateral_for_docking():
    pick_navigation_handoff_ready = _policy("pick_navigation_handoff_ready")

    assert not pick_navigation_handoff_ready([0.87, 0.52, 0.63])


def test_pick_map_navigation_can_handoff_near_station_without_marker():
    pick_map_handoff_ready = _policy("pick_map_handoff_ready")
    base_pose = (1.71, 0.48, math.radians(16.0))

    assert pick_map_handoff_ready(base_pose)


def test_pick_map_navigation_keeps_nav2_when_far_from_station():
    pick_map_handoff_ready = _policy("pick_map_handoff_ready")

    assert not pick_map_handoff_ready((0.0, 0.0, 0.0))


def test_station_dock_velocity_corrects_observed_pick_nav_overshoot():
    station_dock_velocity_for_base = _policy("station_dock_velocity_for_base")

    done, cmd = station_dock_velocity_for_base(
        (2.46, 0.60, math.radians(7.0)),
        "station_a",
    )

    assert not done
    assert cmd.linear.x < 0.0
    assert cmd.linear.y > 0.0
    assert cmd.angular.z > 0.0


def test_station_docking_speed_budget_handles_force_based_base_model():
    assert mission_node.STATION_DOCK_MAX_LINEAR_X >= 0.16
    assert mission_node.STATION_DOCK_MAX_LINEAR_Y >= 0.16


def test_nav_to_pick_uses_nav2_before_local_station_docking():
    events = []
    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    node._navigate = lambda station: events.append(("nav", station)) or True
    node._dock_to_station_pose = (
        lambda station: events.append(("station", station)) or True
    )
    node._dock_to_pick_target = lambda: events.append("pick") or True

    assert mission_node.MissionNode._execute(node, mission_node.TaskState.NAV_TO_PICK)
    assert events == [
        ("nav", "station_a"),
        ("station", "station_a"),
        "pick",
    ]


def test_nav_to_place_uses_nav2_before_local_place_docking():
    events = []
    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    node._navigate = lambda station: events.append(("nav", station)) or True
    node._dock_to_station_pose = (
        lambda station: events.append(("station", station)) or True
    )
    node._dock_to_place_target = lambda: events.append("place") or True

    assert mission_node.MissionNode._execute(node, mission_node.TaskState.NAV_TO_PLACE)
    assert events == [
        ("nav", "station_b"),
        ("station", "station_b"),
        "place",
    ]


def test_return_home_retracts_arm_before_base_navigation():
    events = []

    class FakePickPlace:
        def go_home(self):
            events.append("arm_home")
            return True

    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    node.pp = FakePickPlace()
    node._navigate = lambda station: events.append(("nav", station)) or True
    node._dock_to_station_pose = (
        lambda station: events.append(("station", station)) or True
    )

    assert mission_node.MissionNode._execute(node, mission_node.TaskState.RETURN_HOME)
    assert events == [
        "arm_home",
        ("nav", "home"),
        ("station", "home"),
    ]


def test_return_home_does_not_move_base_when_arm_home_fails():
    events = []

    class FakePickPlace:
        def go_home(self):
            events.append("arm_home")
            return False

    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    node.pp = FakePickPlace()
    node._navigate = lambda station: events.append(("nav", station)) or True
    node._dock_to_station_pose = (
        lambda station: events.append(("station", station)) or True
    )

    assert not mission_node.MissionNode._execute(node, mission_node.TaskState.RETURN_HOME)
    assert events == ["arm_home"]


def test_station_dock_velocity_stops_when_station_a_aligned():
    station_dock_velocity_for_base = _policy("station_dock_velocity_for_base")

    done, cmd = station_dock_velocity_for_base(
        (2.0, 0.62, math.radians(90.0)), "station_a"
    )

    assert done
    assert cmd.linear.x == pytest.approx(0.0)
    assert cmd.linear.y == pytest.approx(0.0)
    assert cmd.angular.z == pytest.approx(0.0)


@pytest.mark.parametrize("station,x", [("station_a", 2.0), ("station_b", -2.0)])
def test_station_dock_stops_on_worktable_safety_line(station, x):
    safe_y = mission_node.station_safe_base_y(math.pi / 2.0, station)
    done, cmd = mission_node.station_dock_velocity_for_base(
        (x, safe_y, math.pi / 2.0), station
    )
    assert done
    clearance = mission_node.worktable_clearance(
        (x, safe_y, math.pi / 2.0), station
    )
    assert clearance == pytest.approx(0.35)
    assert cmd.linear.x == pytest.approx(0.0)


def test_station_dock_slows_continuously_before_safety_line():
    far_y = mission_node.station_safe_base_y(math.pi / 2.0, "station_a") - 0.30
    near_y = mission_node.station_safe_base_y(math.pi / 2.0, "station_a") - 0.08
    _, far_cmd = mission_node.station_dock_velocity_for_base(
        (2.0, far_y, math.pi / 2.0), "station_a"
    )
    _, near_cmd = mission_node.station_dock_velocity_for_base(
        (2.0, near_y, math.pi / 2.0), "station_a"
    )
    assert far_cmd.linear.x > near_cmd.linear.x > 0.0


def test_station_dock_past_safety_line_only_commands_exit():
    safe_y = mission_node.station_safe_base_y(math.pi / 2.0, "station_b")
    done, cmd = mission_node.station_dock_velocity_for_base(
        (-2.0, safe_y + 0.05, math.pi / 2.0), "station_b"
    )
    assert not done
    assert cmd.linear.x < 0.0
    assert cmd.linear.y == pytest.approx(0.0)
    assert cmd.angular.z == pytest.approx(0.0)


@pytest.mark.parametrize("station,x", [("station_a", 2.10), ("station_b", -1.90)])
def test_station_dock_past_safety_line_suppresses_lateral_and_yaw(station, x):
    safe_y = mission_node.station_safe_base_y(math.pi / 2.0, station)
    done, cmd = mission_node.station_dock_velocity_for_base(
        (x, safe_y + 0.001, math.radians(84.0)), station
    )
    assert not done
    yaw = math.radians(84.0)
    away_map = math.sin(yaw) * cmd.linear.x + math.cos(yaw) * cmd.linear.y
    lateral_map = math.cos(yaw) * cmd.linear.x - math.sin(yaw) * cmd.linear.y
    assert away_map <= -0.02
    assert lateral_map == pytest.approx(0.0)
    assert cmd.angular.z == pytest.approx(0.0)


def test_station_dock_at_safety_line_still_allows_lateral_and_yaw_alignment():
    yaw = math.radians(84.0)
    safe_y = mission_node.station_safe_base_y(yaw, "station_a")
    done, cmd = mission_node.station_dock_velocity_for_base((2.10, safe_y, yaw), "station_a")
    assert not done
    map_forward = math.sin(yaw) * cmd.linear.x + math.cos(yaw) * cmd.linear.y
    assert map_forward <= 0.0
    assert abs(cmd.linear.y) > 0.0
    assert cmd.angular.z > 0.0


def test_home_station_docking_keeps_original_waypoint_behavior():
    done, cmd = mission_node.station_dock_velocity_for_base((0.0, -0.20, 0.0), "home")
    assert not done
    assert cmd.linear.y > 0.0


def test_place_navigation_can_handoff_when_tcp_target_is_on_station_b_table():
    place_navigation_handoff_ready = _policy("place_navigation_handoff_ready")
    base_pose = (-1.95, 0.36, math.radians(80.0))

    assert place_navigation_handoff_ready(base_pose, mission_node.DEFAULT_PLACE_POSE)


def test_place_navigation_can_handoff_when_projected_drop_point_is_on_table():
    place_navigation_handoff_ready = _policy("place_navigation_handoff_ready")
    base_pose = (-1.80, 0.75, math.radians(90.0))
    station_b = mission_node._station_base_pose("station_b")
    distance_to_station = math.hypot(
        base_pose[0] - station_b[0],
        base_pose[1] - station_b[1],
    )
    place_x, place_y = mission_node._base_target_to_map(
        base_pose,
        mission_node.DEFAULT_PLACE_POSE[:2],
    )

    assert distance_to_station > mission_node.PLACE_NAV_HANDOFF_MAX_DISTANCE
    assert mission_node._station_b_table_contains(place_x, place_y)
    assert place_navigation_handoff_ready(base_pose, mission_node.DEFAULT_PLACE_POSE)


def test_place_navigation_keeps_nav2_when_projected_drop_point_misses_table():
    place_navigation_handoff_ready = _policy("place_navigation_handoff_ready")
    base_pose = (-1.80, 0.20, math.radians(90.0))
    station_b = mission_node._station_base_pose("station_b")
    distance_to_station = math.hypot(
        base_pose[0] - station_b[0],
        base_pose[1] - station_b[1],
    )
    place_x, place_y = mission_node._base_target_to_map(
        base_pose,
        mission_node.DEFAULT_PLACE_POSE[:2],
    )

    assert distance_to_station > mission_node.PLACE_NAV_HANDOFF_MAX_DISTANCE
    assert not mission_node._station_b_table_contains(place_x, place_y)
    assert not place_navigation_handoff_ready(
        base_pose,
        mission_node.DEFAULT_PLACE_POSE,
    )


def test_place_navigation_can_handoff_when_base_is_close_enough_for_local_docking():
    place_navigation_handoff_ready = _policy("place_navigation_handoff_ready")
    base_pose = (-1.80, 0.47, math.radians(147.0))

    assert place_navigation_handoff_ready(base_pose, mission_node.DEFAULT_PLACE_POSE)


def test_place_navigation_keeps_nav2_until_place_target_reaches_table_front():
    place_navigation_handoff_ready = _policy("place_navigation_handoff_ready")
    base_pose = (-1.71, 0.22, math.radians(79.0))

    assert not place_navigation_handoff_ready(base_pose, mission_node.DEFAULT_PLACE_POSE)


def test_place_dock_velocity_drives_forward_and_rotates_toward_station_b_pose():
    place_dock_velocity_for_base = _policy("place_dock_velocity_for_base")

    done, cmd = place_dock_velocity_for_base((-2.00, 0.35, math.radians(106.0)))

    assert not done
    assert cmd.linear.x > 0.0
    assert cmd.angular.z < 0.0


def test_place_dock_velocity_stops_when_base_is_aligned_for_table_place():
    place_dock_velocity_for_base = _policy("place_dock_velocity_for_base")

    done, cmd = place_dock_velocity_for_base((-2.01, 0.62, math.radians(88.0)))

    assert done
    assert cmd.linear.x == pytest.approx(0.0)
    assert cmd.linear.y == pytest.approx(0.0)
    assert cmd.angular.z == pytest.approx(0.0)


def test_place_dock_velocity_keeps_docking_when_drop_point_is_near_front_edge():
    place_dock_velocity_for_base = _policy("place_dock_velocity_for_base")

    done, cmd = place_dock_velocity_for_base((-2.00, 0.46, math.radians(90.0)))

    assert not done
    assert cmd.linear.x > 0.0


def test_place_dock_velocity_accepts_gui_verified_table_pose():
    place_dock_velocity_for_base = _policy("place_dock_velocity_for_base")

    done, cmd = place_dock_velocity_for_base((-1.995, 0.62, math.radians(94.7)))

    assert done
    assert cmd.linear.x == pytest.approx(0.0)
    assert cmd.linear.y == pytest.approx(0.0)
    assert cmd.angular.z == pytest.approx(0.0)


def test_place_dock_velocity_keeps_moving_when_drop_point_is_before_table():
    place_dock_velocity_for_base = _policy("place_dock_velocity_for_base")

    done, cmd = place_dock_velocity_for_base((-2.00, 0.40, math.radians(100.0)))

    assert not done
    assert cmd.linear.x > 0.0


def test_home_navigation_can_handoff_when_map_pose_is_close_enough():
    home_navigation_handoff_ready = _policy("home_navigation_handoff_ready")

    assert home_navigation_handoff_ready((-0.02, 0.16, math.radians(17.6)))


def test_home_navigation_keeps_nav2_when_pose_is_still_far_from_home():
    home_navigation_handoff_ready = _policy("home_navigation_handoff_ready")

    assert not home_navigation_handoff_ready((0.09, 0.38, math.radians(23.0)))


class _FakeStateFuture:
    def __init__(self, state_id):
        self._state_id = state_id

    def done(self):
        return True

    def result(self):
        class _Resp:
            pass
        resp = _Resp()

        class _State:
            pass
        resp.current_state = _State()
        resp.current_state.id = self._state_id
        return resp


class _FakeBtStateClient:
    def __init__(self, ready, state_id=3):
        self._ready = ready
        self._state_id = state_id
        self.calls = 0

    def service_is_ready(self):
        return self._ready

    def call_async(self, request):
        self.calls += 1
        return _FakeStateFuture(self._state_id)


def _make_wait_node(client):
    # 2026-07-10 GUI 竞态修复:wait_for_server 只保证 server 存在,不保证
    # active;本组用例锁定 _wait_for_nav_active 的等待/放行行为。
    node = mission_node.MissionNode.__new__(mission_node.MissionNode)
    node._bt_state_client = client
    node.get_logger = lambda: type(
        "Logger", (), {"warn": staticmethod(lambda _msg: None)}
    )()
    return node


def _install_monotonic_wait_clock(monkeypatch):
    clock = {"seconds": 0.0}
    monkeypatch.setattr(
        mission_node.time, "monotonic", lambda: clock["seconds"]
    )
    monkeypatch.setattr(
        mission_node.time,
        "sleep",
        lambda seconds: clock.__setitem__(
            "seconds", clock["seconds"] + seconds
        ),
    )
    monkeypatch.setattr(mission_node.rclpy, "ok", lambda: True)


def test_wait_for_nav_active_passes_when_bt_navigator_active(monkeypatch):
    _install_monotonic_wait_clock(monkeypatch)
    client = _FakeBtStateClient(ready=True, state_id=3)
    node = _make_wait_node(client)

    assert mission_node.MissionNode._wait_for_nav_active(node)
    assert client.calls == 1


def test_wait_for_nav_active_keeps_polling_until_active(monkeypatch):
    _install_monotonic_wait_clock(monkeypatch)
    # 前两轮 inactive(id=2),第三轮 active
    client = _FakeBtStateClient(ready=True, state_id=2)
    node = _make_wait_node(client)
    calls = {"n": 0}
    real_call = client.call_async

    def flip_to_active(request):
        calls["n"] += 1
        if calls["n"] >= 3:
            client._state_id = 3
        return real_call(request)

    client.call_async = flip_to_active

    assert mission_node.MissionNode._wait_for_nav_active(node)
    assert calls["n"] == 3


def test_wait_for_nav_active_times_out_when_never_active(monkeypatch):
    _install_monotonic_wait_clock(monkeypatch)
    client = _FakeBtStateClient(ready=True, state_id=2)
    node = _make_wait_node(client)

    assert not mission_node.MissionNode._wait_for_nav_active(
        node, timeout_sec=1.5
    )
