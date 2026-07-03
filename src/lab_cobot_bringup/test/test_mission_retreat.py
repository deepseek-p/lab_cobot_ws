"""Mission-level retreat policy tests."""
from std_msgs.msg import String

from lab_cobot_bringup import mission_node
from lab_cobot_bringup.mission_node import (
    MissionNode,
    RETREAT_DURATION_SEC,
    RETREAT_LINEAR_X,
    RETREAT_TOPIC,
    requires_departure_retreat,
)
from lab_cobot_bringup.task_state_machine import TaskState


def test_departure_retreat_uses_nav_velocity_chain_and_backs_up():
    assert RETREAT_TOPIC == "/cmd_vel_nav"
    assert RETREAT_LINEAR_X <= -0.15
    assert RETREAT_DURATION_SEC >= 5.0


def test_departure_retreat_runs_after_manipulation_steps_only():
    assert requires_departure_retreat(TaskState.PICK)
    assert requires_departure_retreat(TaskState.PLACE)
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
