"""Object detector node tests."""
import ast
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from lab_cobot_perception import object_detector  # noqa: E402
from lab_cobot_perception.yolo_backend import Detection2D  # noqa: E402


def test_camera_stream_subscriptions_use_sensor_data_qos():
    source = Path(object_detector.__file__).read_text(encoding="utf-8")
    assert source.count("qos_profile_sensor_data") >= 4


class FakeLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def error(self, message, **kwargs):
        self.errors.append((message, kwargs))

    def warning(self, message, **kwargs):
        self.warnings.append((message, kwargs))

    def warn(self, message, **kwargs):
        self.warning(message, **kwargs)


class FakeClock:
    def now(self):
        return self

    def to_msg(self):
        return Time()


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeBuffer:
    def lookup_transform(self, target, source, _time, timeout=None):
        assert target == "base_link"
        assert source == "camera_optical_frame"
        transform = TransformStamped()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "camera_optical_frame"
        transform.transform.rotation.w = 1.0
        return transform


def _configured_detector(monkeypatch, detections, clusters, aruco_xyz=None):
    detector = object.__new__(object_detector.ObjectDetector)
    detector.rgb_img = np.zeros((4, 4, 3), dtype=np.uint8)
    detector.depth_img = np.ones((4, 4), dtype=np.float32)
    detector.fx = 100.0
    detector.fy = 100.0
    detector.cx = 0.0
    detector.cy = 0.0
    detector.z_min = 0.4
    detector.z_max = 1.4
    detector.voxel_size = 0.005
    detector.plane_dist = 0.008
    detector.cluster_eps = 0.02
    detector.cluster_min_points = 15
    detector.aruco_gate_m = 0.06
    detector.optical_frame = "camera_optical_frame"
    detector.target_frame = "base_link"
    detector.object_pub = FakePublisher()
    detector.tf_buffer = FakeBuffer()
    detector.get_clock = lambda: FakeClock()
    detector.get_logger = lambda: FakeLogger()
    detector._infer = lambda _image: detections
    detector.aruco_xyz = aruco_xyz
    monkeypatch.setattr(
        object_detector.pointcloud_ops,
        "segment_objects",
        lambda *_args, **_kwargs: clusters,
    )
    return detector


def _cluster(x=0.0, y=0.0, z=1.0):
    return {
        "centroid": np.array([x, y, z], dtype=np.float64),
        "extent": np.array([0.07, 0.07, 0.16], dtype=np.float64),
        "n_points": 42,
    }


def test_process_publishes_classified_detection_in_base_link(monkeypatch):
    detector = _configured_detector(
        monkeypatch,
        [Detection2D("blue cylinder", 0.72, (-5.0, -5.0, 5.0, 5.0))],
        [_cluster()],
    )

    detector._process()

    msg = detector.object_pub.messages[-1]
    assert msg.header.frame_id == "base_link"
    assert len(msg.detections) == 1
    detection = msg.detections[0]
    assert detection.id == "cluster_0"
    assert detection.header.frame_id == "base_link"
    assert detection.results[0].hypothesis.class_id == "blue cylinder"
    assert detection.results[0].hypothesis.score == 0.72
    assert detection.bbox.center.position.z == 1.0
    assert detection.bbox.size.z == 0.16


def test_process_publishes_unknown_when_yolo_has_no_matching_box(monkeypatch):
    detector = _configured_detector(monkeypatch, [], [_cluster()])

    detector._process()

    detection = detector.object_pub.messages[-1].detections[0]
    assert detection.id == "cluster_0"
    assert detection.results[0].hypothesis.class_id == "unknown"
    assert detection.results[0].hypothesis.score == 0.0


def test_process_marks_cluster_as_aruco_sample_when_pose_is_inside_gate(monkeypatch):
    detector = _configured_detector(
        monkeypatch,
        [Detection2D("white box", 0.64, (-5.0, -5.0, 5.0, 5.0))],
        [_cluster()],
        aruco_xyz=np.array([0.01, 0.0, 1.0], dtype=np.float64),
    )

    detector._process()

    detection = detector.object_pub.messages[-1].detections[0]
    assert detection.id == "aruco_0"
    assert detection.results[0].hypothesis.class_id == "sample cube"


def test_aruco_pose_callback_caches_pose_in_target_frame():
    detector = object.__new__(object_detector.ObjectDetector)
    detector.target_frame = "base_link"
    detector.get_logger = lambda: FakeLogger()
    msg = PoseStamped()
    msg.header.frame_id = "base_link"
    msg.pose.position.x = 0.1
    msg.pose.position.y = 0.2
    msg.pose.position.z = 0.9

    detector._aruco_pose_cb(msg)

    np.testing.assert_allclose(detector.aruco_xyz, np.array([0.1, 0.2, 0.9]))


def test_missing_model_path_disables_runtime_without_publishers(tmp_path):
    detector = object.__new__(object_detector.ObjectDetector)
    detector.model_path = str(tmp_path / "missing.pt")
    logger = FakeLogger()
    detector.get_logger = lambda: logger
    detector.create_publisher = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        AssertionError("publisher should not be created")
    )
    detector.create_subscription = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        AssertionError("subscription should not be created")
    )
    detector.create_timer = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        AssertionError("timer should not be created")
    )

    detector._setup_runtime()

    assert detector.disabled is True
    assert logger.errors
    assert not hasattr(detector, "object_pub")


def test_module_top_level_has_no_dl_heavy_imports():
    source = Path(object_detector.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imports = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.add((node.module or "").split(".")[0])

    assert imports.isdisjoint({"ultralytics", "open3d", "torch"})


def test_package_declares_vision_msgs_runtime_dependency():
    package_xml = Path(__file__).resolve().parents[1] / "package.xml"
    root = ET.parse(package_xml).getroot()
    exec_depends = [node.text for node in root.findall("exec_depend")]

    assert "vision_msgs" in exec_depends
