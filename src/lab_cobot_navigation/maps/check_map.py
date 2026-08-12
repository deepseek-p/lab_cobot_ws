#!/usr/bin/env python3
"""Check static map wall coverage, obstacle noise, and key free points."""
from collections import deque
from pathlib import Path
import sys

import numpy as np
import yaml
from PIL import Image

# Allow importing from workspace when run standalone
_WS_SRC = Path(__file__).resolve().parents[3] / "src"
if str(_WS_SRC) not in sys.path:
    sys.path.insert(0, str(_WS_SRC))

from lab_cobot_navigation.waypoints import STATION_SPECS

BASE = Path(__file__).resolve().parent
WALL_LIMIT = 3.4
MAX_OBSTACLE_CLUSTERS = 350
FREE_THRESHOLD = 250


def _station_free_points():
    """Generate free-point check list from StationSpec nav/dock poses."""
    points = []
    for name, spec in STATION_SPECS.items():
        points.append((f"{name}_nav", spec.nav_pose.x, spec.nav_pose.y))
        points.append((f"{name}_dock", spec.dock_pose.x, spec.dock_pose.y))
        for leg in spec.nav_legs:
            points.append((f"{name}_leg_{leg.name}", leg.pose.x, leg.pose.y))
    return tuple(points)


def world_to_pixel(x, y, width, height, resolution, origin_x, origin_y):
    px = int((x - origin_x) / resolution)
    py = height - 1 - int((y - origin_y) / resolution)
    if not (0 <= px < width and 0 <= py < height):
        raise AssertionError(f"点({x}, {y}) 超出地图范围 -> 像素({px}, {py})")
    return px, py


def count_components(mask):
    visited = np.zeros(mask.shape, dtype=bool)
    height, width = mask.shape
    components = 0
    for start_y, start_x in np.argwhere(mask):
        if visited[start_y, start_x]:
            continue
        components += 1
        queue = deque([(int(start_y), int(start_x))])
        visited[start_y, start_x] = True
        while queue:
            y, x = queue.popleft()
            for ny in range(max(0, y - 1), min(height, y + 2)):
                for nx in range(max(0, x - 1), min(width, x + 2)):
                    if mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        queue.append((ny, nx))
    return components


def assert_free_window(img, px, py, name):
    window = img[max(0, py - 2):py + 3, max(0, px - 2):px + 3]
    value = int(img[py, px])
    print(f"{name} -> 像素({px},{py}), 值={value}, 5x5 min={int(window.min())}")
    if value < FREE_THRESHOLD or int(window.min()) < FREE_THRESHOLD:
        raise AssertionError(f"{name} 不是可靠 free 区域")


def main():
    meta = yaml.safe_load((BASE / "map.yaml").read_text(encoding="utf-8"))
    img = np.array(Image.open(BASE / meta["image"]))
    if img.ndim == 3:
        img = img[:, :, 0]

    height, width = img.shape
    resolution = float(meta["resolution"])
    origin_x, origin_y = meta["origin"][0], meta["origin"][1]
    print(f"地图 {width}x{height}, origin({origin_x},{origin_y}), res {resolution}")

    obstacle = img < 50
    obstacle_count = int(obstacle.sum())
    print(f"障碍像素总数: {obstacle_count}")
    if obstacle_count == 0:
        raise AssertionError("地图没有障碍像素")

    ys, xs = np.where(obstacle)
    world_x = origin_x + (xs + 0.5) * resolution
    world_y = origin_y + (height - ys - 0.5) * resolution
    x_min, x_max = float(world_x.min()), float(world_x.max())
    y_min, y_max = float(world_y.min()), float(world_y.max())
    print(f"占用范围: x=[{x_min:.3f}, {x_max:.3f}], y=[{y_min:.3f}, {y_max:.3f}]")
    if x_min > -WALL_LIMIT or x_max < WALL_LIMIT:
        raise AssertionError("东西墙覆盖不足")
    if y_min > -WALL_LIMIT or y_max < WALL_LIMIT:
        raise AssertionError("南北墙覆盖不足")

    components = count_components(obstacle)
    print(f"连通域(障碍簇)数: {components}")
    if components > MAX_OBSTACLE_CLUSTERS:
        raise AssertionError(f"障碍噪点簇过多: {components}")

    for name, x, y in _station_free_points():
        px, py = world_to_pixel(x, y, width, height, resolution, origin_x, origin_y)
        assert_free_window(img, px, py, name)

    print("PASS: map covers four walls, has low obstacle noise, and key points are free")


if __name__ == "__main__":
    main()
