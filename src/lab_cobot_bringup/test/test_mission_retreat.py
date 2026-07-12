"""Mission-level retreat policy tests."""
from builtin_interfaces.msg import Time
import rclpy.duration
from std_msgs.msg import String

from lab_cobot_bringup import mission_node
from lab_cobot_bringup import mecanum_wheel_visualizer
from lab_cobot_bringup.mission_node import (
    DEFAULT_PLACE_POSE,
    MissionNode,
    RETREAT_DURATION_SEC,
    RETREAT_LINEAR_X,
    RETREAT_TOPIC,
    requires_departure_retreat,
)
from lab_cobot_bringup.task_state_machine import TaskState


class FakeStamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        return rclpy.duration.Duration(seconds=self.seconds - other.seconds)

    def to_msg(self):
        msg = Time()
        msg.sec = int(self.seconds)
        msg.nanosec = int((self.seconds - int(self.seconds)) * 1e9)
        return msg


class FakeClock:
    def __init__(self):
        self.seconds = 0.0
        self.now_calls = 0

    def now(self):
        self.now_calls += 1
        return FakeStamp(self.seconds)

    def advance(self, seconds):
        self.seconds += seconds


class FakeNavActionClient:
    """Fake nav action client with a configurable availability probe."""

    def __init__(self, available=True):
        self.available = available
        self.wait_calls = []

    def wait_for_server(self, timeout_sec=None):
        self.wait_calls.append(timeout_sec)
        return self.available


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def warn(self, message):
        self.messages.append(("warn", message))

    def error(self, message):
        self.messages.append(("error", message))


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_departure_retreat_uses_mecanum_driver_command_topic_and_backs_up():
    assert RETREAT_TOPIC == mecanum_wheel_visualizer.CMD_VEL_TOPIC
    assert RETREAT_LINEAR_X <= -0.15
    assert RETREAT_DURATION_SEC >= 5.0


def test_departure_retreat_runs_after_manipulation_steps_only():
    assert requires_departure_retreat(TaskState.PICK)
    assert not requires_departure_retreat(TaskState.PLACE)
    assert not requires_departure_retreat(TaskState.NAV_TO_PICK)
    assert not requires_departure_retreat(TaskState.NAV_TO_PLACE)
    assert not requires_departure_retreat(TaskState.RETURN_HOME)


def test_instruction_marks_busy_before_starting_worker_thread(monkeypatch):
    starts = []

    class FakeThread:
        def __init__(self, target, daemon):
            self.target = target
            self.daemon = daemon

        def start(self):
            starts.append(self.target)

    class FakeLogger:
        def info(self, _msg):
            pass

        def warn(self, _msg):
            pass

    monkeypatch.setattr(mission_node, "Thread", FakeThread)
    node = MissionNode.__new__(MissionNode)
    node._busy = False
    node.get_logger = lambda: FakeLogger()

    msg = String()
    msg.data = "把样件从A送到B"
    MissionNode._on_instruction(node, msg)
    MissionNode._on_instruction(node, msg)

    assert node._busy is True
    assert len(starts) == 1


def test_navigate_fails_fast_when_action_server_unavailable():
    """Navigation must fail bounded when bt_navigator never comes up."""
    # 根因回归(E2E 重复统计实测):bt_navigator 偶发未就绪时,
    # BasicNavigator.goToPose 内部 wait_for_server 无限等待,
    # 任务挂死 NAV_TO_PICK 直到 420s 超时。导航前必须有界探测。
    node = MissionNode.__new__(MissionNode)
    node.get_logger = lambda: FakeLogger()

    goto_calls = []

    class UnavailableNav:
        def __init__(self):
            self.header_clock = FakeClock()
            self.nav_to_pose_client = FakeNavActionClient(available=False)

        def get_clock(self):
            return self.header_clock

        def goToPose(self, goal):
            goto_calls.append(goal)

    node.nav = UnavailableNav()

    assert MissionNode._navigate(node, "station_a") is False
    assert goto_calls == []  # 服务不可用时绝不发送目标
    assert node.nav.nav_to_pose_client.wait_calls  # 确实做了有界探测


def test_mission_waits_for_navigation_readiness_before_starting_task():
    """Cold-start infrastructure must not consume a business-state retry."""
    events = []
    node = MissionNode.__new__(MissionNode)
    node._instruction = "把样件从A送到B"
    node._planner_config = None
    node._busy = True
    node._wait_for_navigation_ready = lambda: events.append("ready") or False
    node._execute = lambda state: events.append(("execute", state)) or False
    node._publish = lambda state: events.append(("publish", state))
    node._failsafe_cleanup = lambda: events.append("cleanup")
    node.get_logger = lambda: FakeLogger()

    MissionNode._run_mission(node)

    assert events == ["ready"]
    assert node._busy is False


def test_navigation_readiness_waits_through_initial_action_unavailability(
    monkeypatch,
):
    clock = FakeClock()

    class DelayedActionClient:
        def __init__(self):
            self.calls = 0

        def wait_for_server(self, timeout_sec=None):
            self.calls += 1
            clock.advance(float(timeout_sec))
            return self.calls >= 3

    node = MissionNode.__new__(MissionNode)
    node.nav = type("FakeNav", (), {})()
    node.nav.nav_to_pose_client = DelayedActionClient()
    node._wait_for_nav_active = lambda timeout_sec=None: True
    node.get_logger = lambda: FakeLogger()
    monkeypatch.setattr(mission_node.time, "monotonic", lambda: clock.seconds)

    assert MissionNode._wait_for_navigation_ready(node, timeout_sec=10.0)
    assert node.nav.nav_to_pose_client.calls == 3


def test_navigate_waits_for_map_tf_before_sending_goal(monkeypatch):
    # Nav2 can accept an action goal before AMCL has populated map TF history.
    # That stamps the goal too early and can poison the BT with past extrapolation.
    clock = FakeClock()
    goto_calls = []

    class FakeNav:
        def __init__(self):
            self.header_clock = clock
            self.nav_to_pose_client = FakeNavActionClient(available=True)

        def get_clock(self):
            return self.header_clock

        def goToPose(self, goal):
            goto_calls.append(goal)

        def isTaskComplete(self):
            return True

        def getResult(self):
            return None

    monkeypatch.setattr(
        mission_node.time,
        "sleep",
        lambda seconds: clock.advance(max(float(seconds), 100.0)),
    )

    node = MissionNode.__new__(MissionNode)
    node.nav = FakeNav()
    node._wait_for_nav_active = lambda: True
    node._base_pose_in_map = lambda timeout_sec=2.0: None
    node._navigation_handoff_ready = lambda _station: False
    node._stop_base = lambda _duration: None
    node.get_clock = lambda: clock
    node.get_logger = lambda: FakeLogger()

    assert MissionNode._navigate(node, "station_a") is False
    assert goto_calls == []


def test_navigate_cancels_nav_task_when_timeout_expires(monkeypatch):
    class FakeNav:
        def __init__(self):
            self.cancel_calls = 0
            self.task_checks = 0
            self.header_clock = FakeClock()
            self.nav_to_pose_client = FakeNavActionClient(available=True)

        def get_clock(self):
            return self.header_clock

        def goToPose(self, _goal):
            pass

        def isTaskComplete(self):
            self.task_checks += 1
            return self.task_checks > 10

        def cancelTask(self):
            self.cancel_calls += 1

        def getResult(self):
            return None

    class TimeoutClock:
        def __init__(self):
            self.times = iter([0.0, 0.0, 0.0, 10.0, 30.0, 61.0])

        def now(self):
            return FakeStamp(next(self.times))

    monkeypatch.setattr(mission_node.time, "sleep", lambda _seconds: None)

    node = MissionNode.__new__(MissionNode)
    timeout_clock = TimeoutClock()
    node.nav = FakeNav()
    node._wait_for_nav_active = lambda: True
    node._base_pose_in_map = lambda timeout_sec=2.0: [0.0, 0.0, 0.0]
    node._navigation_handoff_ready = lambda _station: False
    node._stop_base = lambda _duration: None
    node.get_clock = lambda: timeout_clock
    node.get_logger = lambda: FakeLogger()

    assert not MissionNode._navigate(node, "station_a")
    assert node.nav.cancel_calls == 1


def test_run_mission_clears_busy_when_execute_raises():
    node = MissionNode.__new__(MissionNode)
    node._busy = True
    node._publish = lambda _state: None
    node.get_logger = lambda: FakeLogger()

    def raise_from_execute(_state):
        raise RuntimeError("boom")

    node._execute = raise_from_execute

    try:
        MissionNode._run_mission(node)
    except RuntimeError:
        pass

    assert node._busy is False


def test_failed_mission_releases_object_stops_base_and_goes_home(monkeypatch):
    events = []

    class FakeTask:
        def __init__(self, steps, max_retries):
            self.state = TaskState.PICK

        def start(self):
            return self.state

        def is_terminal(self):
            return self.state == TaskState.FAILED

        def on_result(self, ok):
            events.append(("result", ok))
            self.state = TaskState.FAILED
            return self.state

    class FakeGripper:
        def release_object(self):
            events.append("release")

    class FakePickPlace:
        def __init__(self):
            self.gripper = FakeGripper()

        def go_home(self):
            events.append("home")

    monkeypatch.setattr(mission_node, "SequentialTask", FakeTask)

    node = MissionNode.__new__(MissionNode)
    node._busy = True
    node.pp = FakePickPlace()
    node._wait_for_navigation_ready = lambda: True
    node._execute = lambda _state: False
    node._publish = lambda state: events.append(("publish", state))
    node._stop_base = lambda duration: events.append(("stop", duration))
    node.get_logger = lambda: FakeLogger()

    MissionNode._run_mission(node)

    assert "release" in events
    assert ("stop", mission_node.DOCK_STOP_SEC) in events
    assert "home" in events


def test_retreat_and_stop_base_use_ros_clock(monkeypatch):
    clock = FakeClock()
    publisher = FakePublisher()
    monkeypatch.setattr(mission_node, "RETREAT_DURATION_SEC", 0.15)
    monkeypatch.setattr(mission_node, "RETREAT_STOP_SEC", 0.10)
    monkeypatch.setattr(mission_node, "RETREAT_PUBLISH_PERIOD_SEC", 0.05)
    monkeypatch.setattr(mission_node.time, "time", lambda: clock.seconds)
    monkeypatch.setattr(mission_node.time, "sleep", clock.advance)

    node = MissionNode.__new__(MissionNode)
    node.retreat_pub = publisher
    node.get_clock = lambda: clock
    node.get_logger = lambda: FakeLogger()

    assert MissionNode._retreat_from_station(node)

    assert clock.now_calls > 0
    assert any(msg.linear.x == RETREAT_LINEAR_X for msg in publisher.messages)
    assert publisher.messages[-1].linear.x == 0.0


def test_pick_docking_timeout_uses_ros_clock(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(mission_node, "DOCK_TIMEOUT_SEC", 0.15)
    monkeypatch.setattr(mission_node, "DOCK_PUBLISH_PERIOD_SEC", 0.05)
    monkeypatch.setattr(mission_node.time, "time", lambda: clock.seconds)
    monkeypatch.setattr(mission_node.time, "sleep", clock.advance)

    node = MissionNode.__new__(MissionNode)
    node.retreat_pub = FakePublisher()
    node._detect = lambda timeout_sec=2.0: None
    node._stop_base = lambda duration: None
    node.get_clock = lambda: clock
    node.get_logger = lambda: FakeLogger()

    assert not MissionNode._dock_to_pick_target(node)
    assert clock.now_calls > 0


def test_place_docking_timeout_uses_ros_clock(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(mission_node, "PLACE_DOCK_TIMEOUT_SEC", 0.15)
    monkeypatch.setattr(mission_node, "PLACE_DOCK_PUBLISH_PERIOD_SEC", 0.05)
    monkeypatch.setattr(mission_node.time, "time", lambda: clock.seconds)
    monkeypatch.setattr(mission_node.time, "sleep", clock.advance)

    node = MissionNode.__new__(MissionNode)
    node.retreat_pub = FakePublisher()
    node.place_pose = DEFAULT_PLACE_POSE
    node._base_pose_in_map = lambda timeout_sec=2.0: None
    node._stop_base = lambda duration: None
    node.get_clock = lambda: clock
    node.get_logger = lambda: FakeLogger()

    assert not MissionNode._dock_to_place_target(node)
    assert clock.now_calls > 0
