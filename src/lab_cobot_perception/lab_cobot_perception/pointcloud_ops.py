"""Point cloud processing helpers."""
import numpy as np


def depth_to_points(depth, fx, fy, cx, cy, z_min, z_max):
    """Convert a depth image to clipped camera-frame points."""
    z = np.asarray(depth, dtype=np.float32)
    finite = np.isfinite(z)
    valid = np.zeros(z.shape, dtype=bool)
    valid[finite] = (z[finite] >= float(z_min)) & (z[finite] <= float(z_max))
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float32)

    v_coords, u_coords = np.indices(z.shape, dtype=np.float32)
    z_valid = z[valid]
    x = (u_coords[valid] - float(cx)) * z_valid / float(fx)
    y = (v_coords[valid] - float(cy)) * z_valid / float(fy)
    return np.column_stack((x, y, z_valid)).astype(np.float32, copy=False)


def segment_objects(points, voxel_size, plane_dist, eps, min_points):
    """Segment tabletop object clusters from camera-frame points."""
    import open3d as o3d

    points_array = np.asarray(points, dtype=np.float64)
    if points_array.size == 0:
        return []
    if points_array.ndim != 2 or points_array.shape[1] != 3:
        raise ValueError("points must be an Nx3 array")

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points_array)
    if float(voxel_size) > 0.0:
        cloud = cloud.voxel_down_sample(voxel_size=float(voxel_size))
    if len(cloud.points) == 0:
        return []

    object_cloud = cloud
    if len(cloud.points) >= 3:
        try:
            if hasattr(o3d.utility, "random"):
                o3d.utility.random.seed(0)
            _plane, inliers = cloud.segment_plane(
                distance_threshold=float(plane_dist),
                ransac_n=3,
                num_iterations=200,
            )
            object_cloud = cloud.select_by_index(inliers, invert=True)
        except RuntimeError:
            object_cloud = cloud
    if len(object_cloud.points) == 0:
        return []

    labels = np.asarray(object_cloud.cluster_dbscan(
        eps=float(eps),
        min_points=int(min_points),
        print_progress=False,
    ))
    clusters = []
    for label in sorted(label for label in set(labels.tolist()) if label >= 0):
        indices = np.where(labels == label)[0]
        cluster_cloud = object_cloud.select_by_index(indices)
        cluster_points = np.asarray(cluster_cloud.points)
        bbox = cluster_cloud.get_axis_aligned_bounding_box()
        clusters.append({
            "centroid": cluster_points.mean(axis=0),
            "extent": np.asarray(bbox.get_extent(), dtype=np.float64),
            "n_points": int(len(cluster_points)),
        })
    return clusters


def project_to_pixel(xyz, fx, fy, cx, cy):
    """Project a 3D point to image pixel coordinates."""
    x, y, z = [float(value) for value in xyz]
    if z <= 0.0 or not np.isfinite(z):
        return (float("inf"), float("inf"))
    return (
        float(fx) * x / z + float(cx),
        float(fy) * y / z + float(cy),
    )


def associate(clusters, dets_2d, fx, fy, cx, cy, expand=0.1):
    """Associate clusters with 2D detections by centroid reprojection."""
    matches = []
    for cluster in clusters:
        u, v = project_to_pixel(cluster["centroid"], fx, fy, cx, cy)
        best_index = None
        best_conf = float("-inf")
        for index, det in enumerate(dets_2d):
            x1, y1, x2, y2 = _det_xyxy(det)
            width = x2 - x1
            height = y2 - y1
            x1 -= width * float(expand)
            x2 += width * float(expand)
            y1 -= height * float(expand)
            y2 += height * float(expand)
            if x1 <= u <= x2 and y1 <= v <= y2:
                conf = _det_conf(det)
                if conf > best_conf:
                    best_index = index
                    best_conf = conf
        matches.append(best_index)
    return matches


def match_aruco(clusters, aruco_xyz, gate_m=0.06):
    """Return the nearest cluster index inside the ArUco gate."""
    if aruco_xyz is None:
        return None
    aruco = np.asarray(aruco_xyz, dtype=np.float64)
    best_index = None
    best_distance = float(gate_m)
    for index, cluster in enumerate(clusters):
        distance = float(np.linalg.norm(np.asarray(cluster["centroid"]) - aruco))
        if distance <= best_distance:
            best_index = index
            best_distance = distance
    return best_index


def _det_xyxy(det):
    if isinstance(det, dict):
        return [float(value) for value in det["xyxy"]]
    return [float(value) for value in det.xyxy]


def _det_conf(det):
    if isinstance(det, dict):
        return float(det.get("conf", 0.0))
    return float(det.conf)
