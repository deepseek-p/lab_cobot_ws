"""ArUco detector regression tests."""
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from builtin_interfaces.msg import Time  # noqa: E402
from lab_cobot_perception import aruco_detector  # noqa: E402
from lab_cobot_perception.aruco_detector import (  # noqa: E402
    ARUCO_AREA_THRESHOLD,
    _make_aruco_detector,
)
from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402


class FakeBridge:
    def __init__(self, image):
        self.image = image

    def imgmsg_to_cv2(self, _msg, desired_encoding="passthrough"):
        return self.image


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message, **kwargs):
        self.warnings.append((message, kwargs))

    def warn(self, message, **kwargs):
        self.warning(message, **kwargs)


class FakeClock:
    def now(self):
        return self

    def to_msg(self):
        return Time()


class FakeBroadcaster:
    def __init__(self):
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)


class FakePosePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_detector_matches_project_sample_texture():
    texture = (
        Path(__file__).resolve().parents[2]
        / "lab_cobot_gazebo"
        / "models"
        / "aruco_sample"
        / "materials"
        / "textures"
        / "aruco_0.png"
    )
    image = cv2.imread(str(texture), cv2.IMREAD_GRAYSCALE)
    assert image is not None

    _, ids = _make_aruco_detector()(image)

    assert ids is not None
    assert ids.flatten().tolist() == [0]


def test_area_threshold_accepts_station_a_runtime_marker_size():
    assert ARUCO_AREA_THRESHOLD <= 850


def test_depth_cb_converts_16uc1_depth_to_meters():
    detector = object.__new__(aruco_detector.ArucoDetector)
    detector.bridge = FakeBridge(np.array([[1500]], dtype=np.uint16))
    detector.get_logger = lambda: FakeLogger()
    msg = Image()
    msg.encoding = "16UC1"

    detector._depth_cb(msg)

    assert detector.depth_img.dtype == np.float32
    assert detector.depth_img[0, 0] == pytest.approx(1.5)


def test_depth_cb_keeps_32fc1_depth_in_meters():
    detector = object.__new__(aruco_detector.ArucoDetector)
    detector.bridge = FakeBridge(np.array([[1.5]], dtype=np.float32))
    detector.get_logger = lambda: FakeLogger()
    msg = Image()
    msg.encoding = "32FC1"

    detector._depth_cb(msg)

    assert detector.depth_img.dtype == np.float32
    assert detector.depth_img[0, 0] == pytest.approx(1.5)


def test_info_cb_stores_camera_matrix_and_distortion_coefficients():
    detector = object.__new__(aruco_detector.ArucoDetector)
    msg = CameraInfo()
    msg.k = [500.0, 0.0, 320.0, 0.0, 510.0, 240.0, 0.0, 0.0, 1.0]
    msg.d = [0.1, -0.02, 0.001, 0.002, 0.0]

    detector._info_cb(msg)

    assert detector.fx == pytest.approx(500.0)
    assert detector.fy == pytest.approx(510.0)
    assert detector.cx == pytest.approx(320.0)
    assert detector.cy == pytest.approx(240.0)
    assert detector.camera_matrix.tolist() == [
        [500.0, 0.0, 320.0],
        [0.0, 510.0, 240.0],
        [0.0, 0.0, 1.0],
    ]
    assert detector.dist_coeffs.tolist() == pytest.approx(
        [0.1, -0.02, 0.001, 0.002, 0.0]
    )


def test_estimate_marker_pose_recovers_synthetic_6d_pose():
    marker_size = 0.07
    half = marker_size / 2.0
    object_points = np.array(
        [[-half, -half, 0.0], [half, -half, 0.0],
         [half, half, 0.0], [-half, half, 0.0]],
        dtype=np.float32,
    )
    camera_matrix = np.array(
        [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    dist_coeffs = np.zeros(5)
    rvec = np.array([[0.0], [0.0], [0.4]], dtype=np.float64)
    tvec = np.array([[0.10], [0.02], [1.20]], dtype=np.float64)
    image_points, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs,
    )

    position, orientation = aruco_detector.estimate_marker_pose_from_corners(
        image_points.reshape(4, 2),
        marker_size,
        camera_matrix,
        dist_coeffs,
    )

    assert position == pytest.approx((0.10, 0.02, 1.20), abs=1e-5)
    assert orientation[2] == pytest.approx(np.sin(0.2), abs=1e-4)
    assert orientation[3] == pytest.approx(np.cos(0.2), abs=1e-4)


def test_publish_transforms_camera_pose_to_target_frame():
    detector = object.__new__(aruco_detector.ArucoDetector)
    detector.optical_frame = "camera_optical_frame"
    detector.target_frame = "base_link"
    detector.br = FakeBroadcaster()
    detector.pose_pubs = {3: FakePosePublisher()}
    detector.get_clock = lambda: FakeClock()

    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "camera_optical_frame"
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.translation.z = 3.0
    transform.transform.rotation.w = 1.0

    class FakeBuffer:
        def lookup_transform(self, target, source, _time, timeout=None):
            assert target == "base_link"
            assert source == "camera_optical_frame"
            return transform

    detector.tf_buffer = FakeBuffer()

    detector._publish(
        3,
        (0.1, 0.2, 1.2),
        (0.0, 0.0, np.sin(0.2), np.cos(0.2)),
    )

    published_tf = detector.br.transforms[-1]
    assert published_tf.header.frame_id == "base_link"
    assert published_tf.transform.translation.x == pytest.approx(1.1)
    assert published_tf.transform.translation.y == pytest.approx(2.2)
    assert published_tf.transform.translation.z == pytest.approx(4.2)
    assert published_tf.transform.rotation.z == pytest.approx(np.sin(0.2))
    assert detector.pose_pubs[3].messages[-1].header.frame_id == "base_link"


def test_process_offsets_marker_surface_depth_to_sample_center():
    detector = object.__new__(aruco_detector.ArucoDetector)
    detector.rgb_img = np.zeros((200, 200, 3), dtype=np.uint8)
    detector.depth_img = np.full((200, 200), 1.5, dtype=np.float32)
    detector.fx = 100.0
    detector.fy = 100.0
    detector.cx = 100.0
    detector.cy = 100.0
    detector.camera_matrix = np.array(
        [[100.0, 0.0, 100.0], [0.0, 100.0, 100.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    detector.dist_coeffs = np.zeros(5)
    detector.marker_size_m = 0.07
    detector.marker_to_object_center_m = 0.035
    detector.detect = lambda _gray: (
        [
            np.array(
                [[[50.0, 50.0], [150.0, 50.0], [150.0, 150.0], [50.0, 150.0]]],
                dtype=np.float32,
            )
        ],
        np.array([[7]], dtype=np.int32),
    )
    published = {}
    detector._publish = lambda mid, position, orientation: published.update(
        {
            "mid": mid,
            "position": position,
            "orientation": orientation,
        }
    )

    detector._process()

    assert published["mid"] == 7
    assert published["position"] == pytest.approx((0.0, 0.0, 1.535))
    assert len(published["orientation"]) == 4


def test_model_pose_to_object_transform_defaults_to_odom_truth_frame():
    from geometry_msgs.msg import Pose

    assert aruco_detector.DEFAULT_GAZEBO_REFERENCE_FRAME == "odom"
    assert hasattr(aruco_detector, "model_pose_to_object_transform")
    pose = Pose()
    pose.position.x = 2.0
    pose.position.y = 1.5
    pose.position.z = 0.785
    pose.orientation.w = 1.0

    transform = aruco_detector.model_pose_to_object_transform(
        object_id=0,
        pose=pose,
    )

    assert transform.header.frame_id == "odom"
    assert transform.child_frame_id == "obj_0"
    assert transform.transform.translation.x == 2.0
    assert transform.transform.translation.y == 1.5
    assert transform.transform.translation.z == 0.785
    assert transform.transform.rotation.w == 1.0
