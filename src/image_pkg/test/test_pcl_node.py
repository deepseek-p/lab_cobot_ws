import numpy as np

from image_pkg.pcl_node import compute_centroid_from_patch


def test_compute_centroid_from_patch():
    points = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 2.0, 0.0],
        [1.0, 2.0, 0.0],
        [2.0, 1.0, 0.0],
    ], dtype=np.float32)

    centroid = compute_centroid_from_patch(
        points, width=3, height=3, center_u=1, center_v=1, window=1)

    assert centroid is not None
    assert np.allclose(centroid, np.array([0.8, 1.0, 0.0], dtype=np.float32))
