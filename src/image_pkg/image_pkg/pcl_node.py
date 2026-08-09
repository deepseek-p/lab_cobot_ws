"""ROS-independent organized point-cloud geometry helpers."""
import math

import numpy as np


def centroid_from_box(
    points,
    width,
    height,
    x1,
    y1,
    x2,
    y2,
    min_points=8,
):
    """Return a robust 3D centroid for a two-dimensional detection box."""
    cloud = np.asarray(points, dtype=np.float32).reshape(
        int(height), int(width), 3
    )
    x1 = max(0, min(int(width), int(math.floor(x1))))
    y1 = max(0, min(int(height), int(math.floor(y1))))
    x2 = max(x1, min(int(width), int(math.ceil(x2))))
    y2 = max(y1, min(int(height), int(math.ceil(y2))))
    patch = cloud[y1:y2, x1:x2].reshape(-1, 3)
    patch = patch[np.isfinite(patch).all(axis=1)]
    if len(patch) < int(min_points):
        return None
    median = np.median(patch, axis=0)
    distances = np.linalg.norm(patch - median, axis=1)
    limit = max(float(np.quantile(distances, 0.80)), 0.003)
    inliers = patch[distances <= limit]
    if len(inliers) < int(min_points):
        return None
    return np.mean(inliers, axis=0, dtype=np.float32)
