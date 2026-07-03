"""
Camera geometry helpers for RGB-D perception.

这是 ArUco / RGB-D 6D 位姿估计的核心数学,纯函数无 ROS 依赖,便于单元测试。
"""
from __future__ import annotations

from typing import Tuple


def pixel_to_camera(
    u: float, v: float, depth: float,
    fx: float, fy: float, cx: float, cy: float,
) -> Tuple[float, float, float]:
    """
    Back-project a depth pixel into the camera optical frame.

    相机光学系约定(REP-103):z 沿光轴向前,x 向右,y 向下。
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    z = depth
    """
    if fx == 0.0 or fy == 0.0:
        raise ValueError("focal length fx/fy 不能为 0")
    z = float(depth)
    x = (float(u) - cx) * z / fx
    y = (float(v) - cy) * z / fy
    return (x, y, z)


def offset_along_camera_ray(
    point_cam: Tuple[float, float, float],
    offset_m: float,
) -> Tuple[float, float, float]:
    """
    Move a camera-frame point farther along the same pixel ray.

    RGB-D depth on an ArUco marker reports the visible marker surface. For a
    cuboid sample, the object center is farther along that same camera ray by
    approximately half the sample depth.
    """
    x, y, z = (float(point_cam[0]), float(point_cam[1]), float(point_cam[2]))
    if z <= 0.0:
        raise ValueError("camera ray depth must be positive")
    new_z = z + float(offset_m)
    if new_z <= 0.0:
        raise ValueError("offset moves camera ray point behind the camera")
    scale = new_z / z
    return (x * scale, y * scale, new_z)


def fov_to_focal(width_px: int, hfov_rad: float) -> float:
    """Convert horizontal FOV and image width into focal length in pixels."""
    import math
    if hfov_rad <= 0.0:
        raise ValueError("hfov 必须为正")
    return (width_px / 2.0) / math.tan(hfov_rad / 2.0)


def camera_to_base(
    point_cam: Tuple[float, float, float],
    t_base_cam: Tuple[float, float, float],
    r_base_cam,
) -> Tuple[float, float, float]:
    """
    Transform a camera-frame point into the base frame.

    r_base_cam 为 3x3 旋转矩阵(行优先嵌套序列),t_base_cam 为平移。
    """
    px, py, pz = point_cam
    r = r_base_cam
    bx = r[0][0] * px + r[0][1] * py + r[0][2] * pz + t_base_cam[0]
    by = r[1][0] * px + r[1][1] * py + r[1][2] * pz + t_base_cam[1]
    bz = r[2][0] * px + r[2][1] * py + r[2][2] * pz + t_base_cam[2]
    return (bx, by, bz)
