"""ArUco detector regression tests."""
from pathlib import Path
import sys

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lab_cobot_perception.aruco_detector import _make_aruco_detector  # noqa: E402


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
