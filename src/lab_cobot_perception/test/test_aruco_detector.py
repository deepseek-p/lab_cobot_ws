"""ArUco detector regression tests."""
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lab_cobot_perception import aruco_detector  # noqa: E402
from lab_cobot_perception.aruco_detector import (  # noqa: E402
    ARUCO_AREA_THRESHOLD,
    _make_aruco_detector,
)


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


def test_process_offsets_marker_surface_depth_to_sample_center():
    detector = object.__new__(aruco_detector.ArucoDetector)
    detector.rgb_img = np.zeros((200, 200, 3), dtype=np.uint8)
    detector.depth_img = np.full((200, 200), 1.5, dtype=np.float32)
    detector.fx = 100.0
    detector.fy = 100.0
    detector.cx = 100.0
    detector.cy = 100.0
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
    detector._publish = lambda mid, x, y, z: published.update(
        {"mid": mid, "x": x, "y": y, "z": z}
    )

    detector._process()

    assert published["mid"] == 7
    assert published["x"] == 0.0
    assert published["y"] == 0.0
    assert published["z"] == 1.535


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
