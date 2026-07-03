"""ArUco detector regression tests."""
from pathlib import Path
import sys

import cv2

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


def test_detector_offsets_marker_surface_depth_to_sample_center():
    detector = (
        Path(__file__).resolve().parents[1]
        / "lab_cobot_perception"
        / "aruco_detector.py"
    ).read_text(encoding="utf-8")

    assert "marker_to_object_center_m" in detector
    assert "0.035" in detector
    assert "offset_along_camera_ray" in detector


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
