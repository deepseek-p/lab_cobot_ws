"""Unit contracts for the gripper backend boundary."""

from lab_cobot_manipulation.gripper_driver import SimAttachGripperDriver


class FakePublisher:
    def __init__(self, topic, recorder):
        self.topic = topic
        self._recorder = recorder

    def publish(self, msg):
        self._recorder(self.topic, msg)


class FakeNode:
    def __init__(self, attach_status=None):
        self.float_arrays = []
        self.empty_topics = []
        self.logs = []
        self._attach_status = attach_status
        self._status_callback = None

    def create_publisher(self, msg_type, topic, _qos):
        if msg_type.__name__ == "Float64MultiArray":
            return FakePublisher(topic, self._record_float_array)
        return FakePublisher(topic, self._record_empty)

    def create_subscription(self, msg_type, topic, callback, _qos):
        assert msg_type.__name__ == "String"
        assert topic == "/gripper/attach/status"
        self._status_callback = callback
        return object()

    def _record_float_array(self, _topic, msg):
        self.float_arrays.append(list(msg.data))

    def _record_empty(self, topic, _msg):
        self.empty_topics.append(topic)
        if topic == "/gripper/attach/aruco_sample" and self._attach_status:
            status_msg = type("Msg", (), {})()
            status_msg.data = self._attach_status
            self._status_callback(status_msg)

    def get_logger(self):
        return self

    def info(self, msg):
        self.logs.append(("info", msg))

    def warn(self, msg):
        self.logs.append(("warn", msg))


def test_sim_attach_driver_publishes_open_close_and_detach_topics():
    fake_node = FakeNode()
    driver = SimAttachGripperDriver(fake_node, attach_timeout_sec=0.0)

    assert driver.open()
    assert driver.close()
    assert driver.release_object()
    assert fake_node.float_arrays[0] == [0.0, 0.0]
    assert fake_node.float_arrays[1] == [0.012, 0.012]
    assert "/gripper/detach/aruco_sample" in fake_node.empty_topics


def test_acquire_object_returns_true_when_bridge_accepts_attach():
    fake_node = FakeNode(attach_status="attached aruco_sample")
    driver = SimAttachGripperDriver(fake_node, attach_timeout_sec=0.0)

    assert driver.acquire_object()
    assert "/gripper/attach/aruco_sample" in fake_node.empty_topics


def test_acquire_object_returns_false_when_bridge_refuses_attach():
    fake_node = FakeNode(
        attach_status="refused aruco_sample object_outside_finger_gap"
    )
    driver = SimAttachGripperDriver(fake_node, attach_timeout_sec=0.0)

    assert not driver.acquire_object()
    assert any("refused" in message for _level, message in fake_node.logs)


def test_acquire_object_returns_false_when_bridge_status_times_out():
    fake_node = FakeNode()
    driver = SimAttachGripperDriver(fake_node, attach_timeout_sec=0.0)

    assert not driver.acquire_object()
    assert any("timed out" in message for _level, message in fake_node.logs)
