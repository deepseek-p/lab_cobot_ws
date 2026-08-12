"""Map/world contract tests: every StationSpec pose must be free in committed map."""
import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

from lab_cobot_navigation.waypoints import STATION_SPECS, Pose2D

_MAP_DIR = Path(__file__).resolve().parents[1] / "maps"
_MAP_META = _MAP_DIR / "map.yaml"
_MAP_IMAGE = None
_MAP_METADATA = None

FREE_THRESHOLD = 250


def _load_map():
    global _MAP_IMAGE, _MAP_METADATA
    if _MAP_IMAGE is not None:
        return _MAP_IMAGE, _MAP_METADATA
    meta = yaml.safe_load(_MAP_META.read_text(encoding="utf-8"))
    img = np.array(Image.open(_MAP_DIR / meta["image"]))
    if img.ndim == 3:
        img = img[:, :, 0]
    _MAP_IMAGE = img
    _MAP_METADATA = meta
    return img, meta


def _world_to_pixel(x, y, meta):
    resolution = float(meta["resolution"])
    origin_x, origin_y = float(meta["origin"][0]), float(meta["origin"][1])
    height = _MAP_IMAGE.shape[0]
    width = _MAP_IMAGE.shape[1]
    px = int((x - origin_x) / resolution)
    py = height - 1 - int((y - origin_y) / resolution)
    if not (0 <= px < width and 0 <= py < height):
        return None
    return px, py


def _is_free_window(img, px, py, window_px=2):
    h, w = img.shape
    y0 = max(0, py - window_px)
    y1 = min(h, py + window_px + 1)
    x0 = max(0, px - window_px)
    x1 = min(w, px + window_px + 1)
    window = img[y0:y1, x0:x1]
    return int(window.min()) >= FREE_THRESHOLD


def _all_spec_poses():
    """Yield (label, x, y) for every declared pose in every StationSpec."""
    for name, spec in STATION_SPECS.items():
        yield (f"{name}_nav", spec.nav_pose.x, spec.nav_pose.y)
        yield (f"{name}_dock", spec.dock_pose.x, spec.dock_pose.y)
        for leg in spec.nav_legs:
            yield (f"{name}_{leg.name}", leg.pose.x, leg.pose.y)


@pytest.fixture(scope="module")
def loaded_map():
    img, meta = _load_map()
    return img, meta


@pytest.mark.parametrize(
    "label,x,y",
    list(_all_spec_poses()),
)
def test_all_declared_route_poses_are_free(label, x, y, loaded_map):
    img, meta = loaded_map
    px_py = _world_to_pixel(x, y, meta)
    assert px_py is not None, (
        f"{label} ({x:.2f}, {y:.2f}) 超出地图边界"
    )
    px, py = px_py
    assert _is_free_window(img, px, py), (
        f"{label} ({x:.2f},{y:.2f}) -> 像素({px},{py}) "
        f"值={img[py,px]}, 5x5 min={img[max(0,py-2):py+3,max(0,px-2):px+3].min()} "
        f"— 不是可靠 free 区域"
    )


def test_all_station_nav_poses_in_bounds(loaded_map):
    _img, meta = loaded_map
    resolution = float(meta["resolution"])
    origin_x = float(meta["origin"][0])
    origin_y = float(meta["origin"][1])
    height = _img.shape[0]
    width = _img.shape[1]
    map_x_max = origin_x + width * resolution
    map_y_max = origin_y + height * resolution

    for name, spec in STATION_SPECS.items():
        assert origin_x <= spec.nav_pose.x <= map_x_max, (
            f"{name} nav_pose.x={spec.nav_pose.x} 超出地图 X 范围"
        )
        assert origin_y <= spec.nav_pose.y <= map_y_max, (
            f"{name} nav_pose.y={spec.nav_pose.y} 超出地图 Y 范围"
        )
        assert origin_x <= spec.dock_pose.x <= map_x_max, (
            f"{name} dock_pose.x={spec.dock_pose.x} 超出地图 X 范围"
        )
        assert origin_y <= spec.dock_pose.y <= map_y_max, (
            f"{name} dock_pose.y={spec.dock_pose.y} 超出地图 Y 范围"
        )


def test_worktable_dock_standoff_from_table_edge():
    """Dock poses for worktable stations must be outside table footprint."""
    half_length = 0.55 / 2.0
    half_width = 0.50 / 2.0
    for name, spec in STATION_SPECS.items():
        if spec.work_surface is None:
            continue
        ws = spec.work_surface
        dp = spec.dock_pose

        # Check dock pose is not inside table footprint (map frame)
        table_x_min = ws.center_x - ws.size_x / 2.0
        table_x_max = ws.center_x + ws.size_x / 2.0
        table_y_min = ws.center_y - ws.size_y / 2.0
        table_y_max = ws.center_y + ws.size_y / 2.0

        chassis_overlaps = (
            (dp.x - half_width < table_x_max and dp.x + half_width > table_x_min)
            and (dp.y - half_length < table_y_max
                 and dp.y + half_length > table_y_min)
        )
        assert not chassis_overlaps, (
            f"{name}: dock_pose chassis footprint ({dp.x},{dp.y}) "
            f"与台面碰撞体重叠"
        )


def test_inspection_poses_outside_high_voltage_fence():
    """All inspection zone poses must be outside the high-voltage fence."""
    spec = STATION_SPECS["inspection_zone"]
    # high_voltage_zone fence: center (4.36, 2.90), half (1.015, 0.855)
    hv_x_min = 4.36 - 1.015
    hv_x_max = 4.36 + 1.015
    hv_y_min = 2.90 - 0.855
    hv_y_max = 2.90 + 0.855

    poses = [spec.nav_pose, spec.dock_pose]
    for leg in spec.nav_legs:
        poses.append(leg.pose)

    for pose in poses:
        inside_fence = (
            hv_x_min <= pose.x <= hv_x_max
            and hv_y_min <= pose.y <= hv_y_max
        )
        assert not inside_fence, (
            f"inspection pose ({pose.x},{pose.y}) 在高压围栏内"
        )


def test_home_dock_faces_east():
    spec = STATION_SPECS["home"]
    assert abs(spec.dock_pose.yaw - 0.0) < 1e-6, (
        f"home dock yaw={spec.dock_pose.yaw}, expected 0 (east)"
    )


def test_all_worktable_stations_face_north():
    for name in ("station_a", "tooling_zone", "aging_zone", "station_b"):
        spec = STATION_SPECS[name]
        assert abs(spec.dock_pose.yaw - math.pi / 2.0) < 1e-6, (
            f"{name} dock yaw={spec.dock_pose.yaw}, expected π/2 (north)"
        )
