"""ROS-independent helpers for associating image boxes with organized clouds."""
from __future__ import annotations

import math

import numpy as np


def compute_centroid_from_patch(points, width, height, center_u, center_v, window):
    """Return a finite-point centroid from a square organized-cloud patch.

    ``points`` is a flattened ``height * width`` by 3 array.  Invalid (NaN,
    Inf) samples are ignored.  The helper deliberately has no ROS dependency
    so that the core RGB-D geometry remains easy to test.
    """
    values = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    # Unit tests and offline tools may provide a sparse list rather than all
    # width*height pixels.  There is no reliable pixel indexing in that form;
    # use every valid supplied point instead of inventing coordinates.
    if len(values) != int(width) * int(height):
        values = values[np.isfinite(values).all(axis=1)]
        return None if len(values) == 0 else np.mean(values, axis=0, dtype=np.float32)
    return centroid_from_box(
        points, width, height,
        int(center_u) - int(window), int(center_v) - int(window),
        int(center_u) + int(window) + 1, int(center_v) + int(window) + 1,
        min_points=1,
    )


def centroid_from_box(points, width, height, x1, y1, x2, y2, min_points=8):
    """Compute a robust 3D centroid for a 2D detection box.

    A median-distance inlier pass rejects table edges and background samples
    which otherwise leak into a YOLO rectangle around an object.
    """
    cloud = np.asarray(points, dtype=np.float32).reshape(int(height), int(width), 3)
    x1 = max(0, min(int(width), int(math.floor(x1))))
    y1 = max(0, min(int(height), int(math.floor(y1))))
    x2 = max(x1, min(int(width), int(math.ceil(x2))))
    y2 = max(y1, min(int(height), int(math.ceil(y2))))
    patch = cloud[y1:y2, x1:x2].reshape(-1, 3)
    patch = patch[np.isfinite(patch).all(axis=1)]
    if len(patch) < int(min_points):
        return None
    if int(min_points) <= 1:
        return np.mean(patch, axis=0, dtype=np.float32)
    median = np.median(patch, axis=0)
    distances = np.linalg.norm(patch - median, axis=1)
    # Retain the closest 80 percent, with a small absolute floor for a sparse
    # cloud.  This is deterministic and does not require a model template.
    limit = max(float(np.quantile(distances, 0.80)), 0.003)
    inliers = patch[distances <= limit]
    if len(inliers) < int(min_points):
        return None
    return np.mean(inliers, axis=0, dtype=np.float32)


def pca_quaternion(points):
    """Return a right-handed PCA orientation quaternion (x, y, z, w)."""
    values = np.asarray(points, dtype=np.float64)
    values = values[np.isfinite(values).all(axis=1)]
    if len(values) < 3:
        return (0.0, 0.0, 0.0, 1.0)
    _, _, vectors = np.linalg.svd(values - values.mean(axis=0), full_matrices=False)
    rotation = vectors.T
    if np.linalg.det(rotation) < 0.0:
        rotation[:, 2] *= -1.0
    return quaternion_from_matrix(rotation)


def quaternion_from_matrix(matrix):
    """Convert a 3x3 rotation matrix into an ``(x, y, z, w)`` quaternion."""
    m = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((m[2, 1] - m[1, 2]) / scale, (m[0, 2] - m[2, 0]) / scale,
                (m[1, 0] - m[0, 1]) / scale, 0.25 * scale)
    index = int(np.argmax(np.diag(m)))
    if index == 0:
        scale = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return (0.25 * scale, (m[0, 1] + m[1, 0]) / scale,
                (m[0, 2] + m[2, 0]) / scale, (m[2, 1] - m[1, 2]) / scale)
    if index == 1:
        scale = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return ((m[0, 1] + m[1, 0]) / scale, 0.25 * scale,
                (m[1, 2] + m[2, 1]) / scale, (m[0, 2] - m[2, 0]) / scale)
    scale = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return ((m[0, 2] + m[2, 0]) / scale, (m[1, 2] + m[2, 1]) / scale,
            0.25 * scale, (m[1, 0] - m[0, 1]) / scale)
