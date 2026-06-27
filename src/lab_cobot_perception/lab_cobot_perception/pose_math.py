"""相机几何:像素+深度 → 相机系 3D 坐标(针孔模型),以及相机系→目标系的辅助。

这是 ArUco / RGB-D 6D 位姿估计的核心数学,纯函数无 ROS 依赖,便于单元测试。
"""
from __future__ import annotations

from typing import Tuple


def pixel_to_camera(
    u: float, v: float, depth: float,
    fx: float, fy: float, cx: float, cy: float,
) -> Tuple[float, float, float]:
    """针孔模型反投影:像素 (u,v) + 深度 depth → 相机光学系 (x,y,z)。

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


def fov_to_focal(width_px: int, hfov_rad: float) -> float:
    """由水平视场角与图像宽度反算焦距(像素)。fx = (W/2) / tan(hfov/2)。"""
    import math
    if hfov_rad <= 0.0:
        raise ValueError("hfov 必须为正")
    return (width_px / 2.0) / math.tan(hfov_rad / 2.0)


def camera_to_base(
    point_cam: Tuple[float, float, float],
    t_base_cam: Tuple[float, float, float],
    r_base_cam,
) -> Tuple[float, float, float]:
    """把相机系点变换到基座系:p_base = R * p_cam + t。

    r_base_cam 为 3x3 旋转矩阵(行优先嵌套序列),t_base_cam 为平移。
    """
    px, py, pz = point_cam
    r = r_base_cam
    bx = r[0][0] * px + r[0][1] * py + r[0][2] * pz + t_base_cam[0]
    by = r[1][0] * px + r[1][1] * py + r[1][2] * pz + t_base_cam[1]
    bz = r[2][0] * px + r[2][1] * py + r[2][2] * pz + t_base_cam[2]
    return (bx, by, bz)
