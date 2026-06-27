"""pose_math 单元测试(纯逻辑,headless pytest 可跑)。"""
import math

import pytest

from lab_cobot_perception.pose_math import (
    pixel_to_camera,
    fov_to_focal,
    camera_to_base,
)


def test_pixel_center_on_optical_axis():
    # 图像中心像素 → 光轴上,x=y=0,z=depth
    x, y, z = pixel_to_camera(320, 240, 1.5, fx=500, fy=500, cx=320, cy=240)
    assert abs(x) < 1e-9
    assert abs(y) < 1e-9
    assert abs(z - 1.5) < 1e-9


def test_pixel_horizontal_offset():
    # u 偏右 100 像素, depth=2, fx=500 → x = 100*2/500 = 0.4
    x, y, z = pixel_to_camera(420, 240, 2.0, fx=500, fy=500, cx=320, cy=240)
    assert abs(x - 0.4) < 1e-9
    assert abs(y) < 1e-9


def test_pixel_vertical_offset_is_positive_down():
    # v 偏下 60 像素 → y 为正(光学系 y 向下)
    x, y, z = pixel_to_camera(320, 300, 1.0, fx=500, fy=500, cx=320, cy=240)
    assert y > 0
    assert abs(y - (60 * 1.0 / 500)) < 1e-9


def test_pixel_negative_offset():
    x, _, _ = pixel_to_camera(220, 240, 1.0, fx=500, fy=500, cx=320, cy=240)
    assert abs(x - (-100 / 500)) < 1e-9


def test_zero_focal_raises():
    with pytest.raises(ValueError):
        pixel_to_camera(1, 1, 1, fx=0.0, fy=500, cx=0, cy=0)


def test_fov_to_focal_90deg():
    # hfov=90°, W=640 → fx = 320 / tan(45°) = 320
    fx = fov_to_focal(640, math.radians(90))
    assert abs(fx - 320.0) < 1e-6


def test_camera_to_base_identity():
    # 单位旋转 + 平移 → 仅平移
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    p = camera_to_base((0.4, 0.0, 1.5), (0.62, 0.0, 2.0), R)
    assert abs(p[0] - 1.02) < 1e-9
    assert abs(p[1] - 0.0) < 1e-9
    assert abs(p[2] - 3.5) < 1e-9


def test_camera_to_base_rotation_z90():
    # 绕 z 轴 +90°: x->y, y->-x
    R = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    p = camera_to_base((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), R)
    assert abs(p[0] - 0.0) < 1e-9
    assert abs(p[1] - 1.0) < 1e-9
