#!/usr/bin/env python3
"""清理 SLAM 地图孤立噪点,并把机器人起点附近强制标为 free."""
import argparse
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


BASE = Path(__file__).resolve().parent
DEFAULT_MAP_YAML = BASE / "map.yaml"
DEFAULT_CLEAR_RADIUS = 14
DEFAULT_MIN_CLUSTER_SIZE = 20


def world_to_pixel(x, y, width, height, resolution, origin_x, origin_y):
    px = int((x - origin_x) / resolution)
    py = height - 1 - int((y - origin_y) / resolution)
    if not (0 <= px < width and 0 <= py < height):
        raise ValueError(f"点({x}, {y}) 超出地图范围 -> 像素({px}, {py})")
    return px, py


def load_map(map_yaml):
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image_path = Path(meta["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    img = np.array(Image.open(image_path))
    if img.ndim == 3:
        img = img[:, :, 0]
    resolution = float(meta["resolution"])
    origin_x, origin_y = float(meta["origin"][0]), float(meta["origin"][1])
    return img, image_path, resolution, origin_x, origin_y


def default_output_path(image_path):
    return image_path.with_name(f"{image_path.stem}_denoised{image_path.suffix}")


def remove_small_obstacle_clusters(out, obstacle, min_cluster_size):
    if min_cluster_size <= 0:
        print("跳过连通域去噪")
        return

    try:
        from scipy import ndimage

        labeled, count = ndimage.label(obstacle)
        sizes = ndimage.sum(obstacle, labeled, range(1, count + 1))
        removed = 0
        for index, size in enumerate(sizes, start=1):
            if size < min_cluster_size:
                out[labeled == index] = 254
                removed += 1
        print(f"连通域 {count} 个,移除 {removed} 个小噪点簇(保留大簇=墙)")
    except Exception as exc:
        print(f"无 scipy({exc}),跳过连通域去噪,仅清起点")


def denoise_map(map_yaml, output=None, clear_radius=DEFAULT_CLEAR_RADIUS,
                min_cluster_size=DEFAULT_MIN_CLUSTER_SIZE):
    img, image_path, resolution, origin_x, origin_y = load_map(map_yaml)
    height, width = img.shape
    obstacle = img < 50
    before_count = int(obstacle.sum())
    print(f"地图 {width}x{height}, 去噪前障碍像素 {before_count}")

    out = img.copy()
    remove_small_obstacle_clusters(out, obstacle, min_cluster_size)

    px, py = world_to_pixel(
        0.0,
        0.0,
        width,
        height,
        resolution,
        origin_x,
        origin_y,
    )
    yy, xx = np.ogrid[:height, :width]
    mask = (xx - px) ** 2 + (yy - py) ** 2 <= clear_radius ** 2
    freed = int(((out < 50) & mask).sum())
    out[mask & (out < 50)] = 254
    print(f"起点周围清除 {freed} 个占用像素")

    after_count = int((out < 50).sum())
    print(f"去噪后障碍像素 {after_count} (减少 {before_count - after_count})")
    print(f"起点({px},{py})值: {out[py, px]} (需为 254=free)")

    output_path = Path(output) if output is not None else default_output_path(image_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(out).save(output_path)
    print(f"已保存去噪后 {output_path}")
    return output_path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", type=Path, default=DEFAULT_MAP_YAML)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--clear-radius", type=int, default=DEFAULT_CLEAR_RADIUS)
    parser.add_argument(
        "--min-cluster-size",
        type=int,
        default=DEFAULT_MIN_CLUSTER_SIZE,
    )
    return parser.parse_args()


def main():
    args = parse_args()
    denoise_map(
        args.map_yaml,
        output=args.output,
        clear_radius=args.clear_radius,
        min_cluster_size=args.min_cluster_size,
    )


if __name__ == "__main__":
    main()
