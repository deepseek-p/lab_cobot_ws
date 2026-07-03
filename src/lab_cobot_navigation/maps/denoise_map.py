#!/usr/bin/env python3
"""Remove isolated SLAM map noise while preserving real walls."""
import numpy as np
from PIL import Image

MAP = "/home/THW22/projects/lab_cobot_ws/src/lab_cobot_navigation/maps/map.pgm"

img = np.array(Image.open(MAP))
h, w = img.shape
obstacle = img < 50
n0 = int(obstacle.sum())
print(f"地图 {w}x{h}, 去噪前障碍像素 {n0}")

out = img.copy()
try:
    from scipy import ndimage
    labeled, n = ndimage.label(obstacle)
    sizes = ndimage.sum(obstacle, labeled, range(1, n + 1))
    removed = 0
    for i, s in enumerate(sizes, start=1):
        if s < 20:           # 小于 20 像素的簇视为噪点
            out[labeled == i] = 254
            removed += 1
    print(f"连通域 {n} 个,移除 {removed} 个小噪点簇(保留大簇=墙)")
except Exception as e:
    print(f"无 scipy({e}),跳过连通域去噪,仅清起点")

# 机器人起点 (0,0) -> 像素 (81,118),周围 R=14 强制 free(初始位置必为空地)
px, py = 81, 118
yy, xx = np.ogrid[:h, :w]
mask = (xx - px) ** 2 + (yy - py) ** 2 <= 14 ** 2
freed = int(((out < 50) & mask).sum())
out[mask & (out < 50)] = 254
print(f"起点周围清除 {freed} 个占用像素")

n1 = int((out < 50).sum())
print(f"去噪后障碍像素 {n1} (减少 {n0 - n1})")
print(f"起点(81,118)值: {out[118, 81]} (需为 254=free)")
Image.fromarray(out).save(MAP)
print("已保存去噪后 map.pgm")
