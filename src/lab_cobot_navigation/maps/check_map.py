#!/usr/bin/env python3
"""Check map origin occupancy and obstacle noise."""
import numpy as np
import yaml
from PIL import Image

base = "/home/THW22/projects/lab_cobot_ws/src/lab_cobot_navigation/maps/"
m = yaml.safe_load(open(base + "map.yaml"))
img = np.array(Image.open(base + m["image"]))
h, w = img.shape
res = m["resolution"]
ox, oy = m["origin"][0], m["origin"][1]
px = int((0 - ox) / res)
py = h - int((0 - oy) / res)
print(f"地图 {w}x{h}, origin({ox},{oy}), res {res}")
print(f"机器人起点(0,0) -> 像素({px},{py}), 值={img[py, px]} (0占用/254free/205未知)")
print("起点周围 9x9:")
print(img[py - 4:py + 5, px - 4:px + 5])
obstacle = img < 50
print(f"障碍像素总数: {int(obstacle.sum())}")
try:
    from scipy import ndimage
    _, n = ndimage.label(obstacle)
    print(f"连通域(障碍簇)数: {n}  (越多=噪点越多;干净图应只有几簇=墙)")
except Exception:
    pass
# 检查 (1.5,0) 目标点附近
gx = int((1.5 - ox) / res)
gy = h - int((0 - oy) / res)
print(f"目标(1.5,0) -> 像素({gx},{gy}), 值={img[gy, gx]}")
