#!/usr/bin/env python3
"""Generate distinct ArUco textures for the four colored material cubes.

物料区彩色方块:红=id2、绿=id3、蓝=id4、黄=id5 (DICT_4X4_50).
ID 0/1 留给 aruco_sample(主抓取样件),避免与任务腕相机撞码。
每块方块的 front 与 top 面共用同一张纹理。
"""
import hashlib
from pathlib import Path

# color -> marker id
CUBES = {
    "red": 2,
    "green": 3,
    "blue": 4,
    "yellow": 5,
}

MODELS_DIR = (
    Path(__file__).resolve().parents[1] / "src" / "lab_cobot_gazebo" / "models"
)


def _texture_path(color: str, marker_id: int) -> Path:
    return (
        MODELS_DIR
        / f"material_cube_{color}"
        / "materials"
        / "textures"
        / f"marker_{marker_id}.png"
    )


def _draw_marker(dictionary, marker_id: int, size: int):
    """Draw a DICT_4X4_50 marker, compatible with old and new OpenCV APIs."""
    import cv2

    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, size)
    return cv2.aruco.drawMarker(dictionary, marker_id, size)


def _generate_texture(marker_id: int) -> bytes:
    import cv2
    import numpy as np

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = _draw_marker(dictionary, marker_id, 512)
    border = 64
    texture = np.full(
        (512 + 2 * border, 512 + 2 * border), 255, dtype=np.uint8
    )
    texture[border:-border, border:-border] = marker
    ok, buf = cv2.imencode(".png", texture)
    if not ok:
        raise RuntimeError(f"failed to encode marker id={marker_id}")
    return buf.tobytes()


def main() -> None:
    for color, marker_id in CUBES.items():
        path = _texture_path(color, marker_id)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(_generate_texture(marker_id))
        digest = hashlib.md5(path.read_bytes()).hexdigest()  # noqa: S324
        print(f"{color}: id={marker_id} -> {path}")
        print(f"  md5={digest}")


if __name__ == "__main__":
    main()
