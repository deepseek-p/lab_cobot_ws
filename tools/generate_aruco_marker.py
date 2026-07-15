#!/usr/bin/env python3
"""Generate the top ArUco marker texture and print its fingerprint."""

import hashlib
from pathlib import Path


OUTPUT_PATH = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "lab_cobot_gazebo"
    / "models"
    / "aruco_sample"
    / "materials"
    / "textures"
    / "aruco_marker_1.png"
)


def main():
    """Generate marker ID 1 with a white quiet-zone border."""
    import cv2
    import numpy as np

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 1, 512)
    border = 64
    texture = np.full((512 + 2 * border, 512 + 2 * border), 255, dtype=np.uint8)
    texture[border:-border, border:-border] = marker
    if not cv2.imwrite(str(OUTPUT_PATH), texture):
        raise RuntimeError(f"Failed to write {OUTPUT_PATH}")

    digest = hashlib.md5(OUTPUT_PATH.read_bytes()).hexdigest()  # noqa: S324
    print(f"path={OUTPUT_PATH}")
    print(f"md5={digest}")


if __name__ == "__main__":
    main()
