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

    The centre of a YOLO rectangle is more likely to contain the target than
    its border.  We therefore first crop the outer 15 %, then retain the near
    depth layer before the spatial inlier pass.  This rejects the tabletop or
    floor behind a small object instead of averaging it into the pose.
    """
    cloud = np.asarray(points, dtype=np.float32).reshape(int(height), int(width), 3)
    x1 = max(0, min(int(width), int(math.floor(x1))))
    y1 = max(0, min(int(height), int(math.floor(y1))))
    x2 = max(x1, min(int(width), int(math.ceil(x2))))
    y2 = max(y1, min(int(height), int(math.ceil(y2))))
    box_width, box_height = x2 - x1, y2 - y1
    if box_width >= 12 and box_height >= 12:
        margin_x = max(1, int(round(box_width * 0.15)))
        margin_y = max(1, int(round(box_height * 0.15)))
        if box_width - 2 * margin_x >= 4 and box_height - 2 * margin_y >= 4:
            x1, x2 = x1 + margin_x, x2 - margin_x
            y1, y2 = y1 + margin_y, y2 - margin_y
    patch_grid = cloud[y1:y2, x1:x2]
    valid_grid = np.isfinite(patch_grid).all(axis=2)
    patch = patch_grid[valid_grid]
    if len(patch) < int(min_points):
        return None
    if int(min_points) <= 1:
        return np.mean(patch, axis=0, dtype=np.float32)
    ranges = np.linalg.norm(patch, axis=1)
    # Prefer the depth-connected component around the box centre.  A YOLO box
    # around a thin/oblique object often contains more table pixels than
    # object pixels; range quantiles alone can then select the wrong surface.
    # Connectivity keeps the central foreground surface while remaining
    # independent of object colour and lighting.
    component = _centered_depth_component(
        patch_grid, valid_grid, int(min_points))
    if component is not None:
        patch = component
        ranges = np.linalg.norm(patch, axis=1)
    # A box may include both an object and the support surface.  Retaining the
    # closest half of valid depths is conservative for an overhead camera;
    # keep a small metric allowance so a thick object is not truncated.
    near_limit = float(np.quantile(ranges, 0.50)) + 0.015
    near_patch = patch[ranges <= near_limit]
    if len(near_patch) >= int(min_points):
        patch = near_patch
    median = np.median(patch, axis=0)
    distances = np.linalg.norm(patch - median, axis=1)
    # Retain the closest 80 percent, with a small absolute floor for a sparse
    # cloud.  This is deterministic and does not require a model template.
    limit = max(float(np.quantile(distances, 0.80)), 0.003)
    inliers = patch[distances <= limit]
    if len(inliers) < int(min_points):
        return None
    return np.mean(inliers, axis=0, dtype=np.float32)


def foreground_centroid_above_support_plane(
    points, width, height, x1, y1, x2, y2, min_points=8,
    plane_distance_m=0.004,
):
    """Estimate the visible object surface after removing its support plane.

    A detection rectangle around glassware or a thin tool can contain more
    table pixels than object pixels.  Fit the dominant plane in a ring around
    the rectangle, remove points on that plane, retain the centre-nearest
    connected foreground component, and finally average its camera-nearest
    surface.  The routine falls back to :func:`centroid_from_box` whenever a
    reliable surrounding plane is unavailable.
    """
    feature = foreground_feature_above_support_plane(
        points, width, height, x1, y1, x2, y2,
        min_points=min_points, plane_distance_m=plane_distance_m,
    )
    if feature is not None:
        return feature["centroid"]
    return centroid_from_box(
        points, width, height, x1, y1, x2, y2,
        min_points=min_points,
    )


def foreground_feature_above_support_plane(
    points, width, height, x1, y1, x2, y2, min_points=8,
    plane_distance_m=0.004, combine_disconnected_components=False,
):
    """Return a support-referenced 6-D feature frame for one image box.

    The old API returned only a visible-surface centroid.  That point changes
    with camera azimuth and is generally *not* the Gazebo/CAD model origin.
    This routine additionally returns the local support-plane anchor and an
    orthonormal ``camera_from_feature`` basis.  Consumers can therefore apply
    a per-object ``T_feature_model`` instead of adding a world-axis Z value.

    The feature origin is the visible foreground centroid projected onto the
    surrounding support plane.  Feature +Z points from the plane toward the
    object, +X follows the dominant in-plane foreground axis, and +Y completes
    a right-handed frame.  When no reliable support plane exists, ``None`` is
    returned so callers can explicitly use their documented centroid fallback.
    """
    cloud = np.asarray(points, dtype=np.float32).reshape(
        int(height), int(width), 3)
    x1 = max(0, min(int(width), int(math.floor(x1))))
    y1 = max(0, min(int(height), int(math.floor(y1))))
    x2 = max(x1, min(int(width), int(math.ceil(x2))))
    y2 = max(y1, min(int(height), int(math.ceil(y2))))
    box_width, box_height = x2 - x1, y2 - y1
    if box_width < 4 or box_height < 4:
        return None
    expand_x = max(12, int(round(box_width * 0.30)))
    expand_y = max(12, int(round(box_height * 0.30)))
    left, right = max(0, x1 - expand_x), min(int(width), x2 + expand_x)
    top, bottom = max(0, y1 - expand_y), min(int(height), y2 + expand_y)
    ring_grid = cloud[top:bottom, left:right]
    ring_mask = np.ones(ring_grid.shape[:2], dtype=bool)
    ring_mask[y1 - top:y2 - top, x1 - left:x2 - left] = False
    ring_mask &= np.isfinite(ring_grid).all(axis=2)
    ring = ring_grid[ring_mask]
    if len(ring) < max(40, int(min_points) * 3):
        return None
    # Reject distant walls and very near robot pixels before fitting the
    # dominant local plane.
    ranges = np.linalg.norm(ring, axis=1)
    low, high = np.quantile(ranges, (0.10, 0.90))
    support = ring[(ranges >= low) & (ranges <= high)].astype(np.float64)
    if len(support) < 30:
        return None
    plane_point = plane_normal = None
    for _ in range(4):
        plane_point = np.median(support, axis=0)
        try:
            _, _, vectors = np.linalg.svd(
                support - plane_point, full_matrices=False)
        except np.linalg.LinAlgError:
            return None
        plane_normal = vectors[-1]
        residual = np.abs((support - plane_point) @ plane_normal)
        limit = max(0.002, min(0.008, float(np.quantile(residual, 0.65))))
        inliers = support[residual <= limit]
        if len(inliers) < 30 or len(inliers) == len(support):
            break
        support = inliers
    if plane_point is None or plane_normal is None:
        return None
    patch_grid = cloud[y1:y2, x1:x2]
    valid = np.isfinite(patch_grid).all(axis=2)
    signed = np.zeros(valid.shape, dtype=np.float64)
    signed[valid] = (patch_grid[valid] - plane_point) @ plane_normal
    foreground_mask = valid & (
        np.abs(signed) >= max(0.002, float(plane_distance_m)))
    component = _nearest_mask_component(
        patch_grid, foreground_mask, int(min_points))
    if component is None:
        return None
    # Keep the centre-connected component for the visible-surface centroid.
    # The support footprint can be assembled later from all above-plane
    # pixels for explicitly composite models.  The default remains the centre
    # component so a bottle box cannot absorb a neighbouring item.
    component_ranges = np.linalg.norm(component, axis=1)
    near_limit = float(np.quantile(component_ranges, 0.35)) + 0.012
    near = component[component_ranges <= near_limit]
    if len(near) >= int(min_points):
        component = near
    median = np.median(component, axis=0)
    distances = np.linalg.norm(component - median, axis=1)
    limit = max(0.003, float(np.quantile(distances, 0.85)))
    inliers = component[distances <= limit]
    if len(inliers) < int(min_points):
        return None
    centroid = np.mean(inliers, axis=0, dtype=np.float64)

    # Resolve the plane-normal sign from the actual foreground rather than a
    # camera convention.  This remains valid for oblique and inverted views.
    normal = np.asarray(plane_normal, dtype=np.float64)
    signed_height = float(np.dot(centroid - plane_point, normal))
    if signed_height < 0.0:
        normal = -normal
        signed_height = -signed_height
    if signed_height < max(0.002, float(plane_distance_m)):
        return None
    full_component = (
        patch_grid[foreground_mask].astype(np.float64)
        if bool(combine_disconnected_components)
        else component.astype(np.float64)
    )
    # Estimate a stable in-plane axis from the *complete* object footprint.
    # Only the translation component is
    # needed by rotationally symmetric objects, while elongated CAD models
    # can use this full basis for a non-zero feature-to-origin XY transform.
    full_heights = (full_component - plane_point) @ normal
    footprint_points = full_component[
        full_heights >= max(0.002, float(plane_distance_m))]
    footprint_heights = full_heights[
        full_heights >= max(0.002, float(plane_distance_m))]
    if len(footprint_points) < int(min_points):
        return None
    footprint = footprint_points - np.outer(footprint_heights, normal)
    footprint_centre = np.median(footprint, axis=0)
    projected = footprint - footprint_centre
    try:
        _, singular, vectors = np.linalg.svd(projected, full_matrices=False)
    except np.linalg.LinAlgError:
        return None
    tangent_x = vectors[0]
    tangent_x -= float(np.dot(tangent_x, normal)) * normal
    norm_x = float(np.linalg.norm(tangent_x))
    if norm_x < 1e-8 or len(singular) == 0:
        return None
    tangent_x /= norm_x
    # PCA axes have a 180-degree ambiguity.  Make the sign deterministic in
    # the optical frame; multi-view fusion then cannot flip the local offset.
    if (tangent_x[0] < 0.0
            or (abs(tangent_x[0]) < 1e-8 and tangent_x[1] < 0.0)):
        tangent_x = -tangent_x
    tangent_y = np.cross(normal, tangent_x)
    norm_y = float(np.linalg.norm(tangent_y))
    if norm_y < 1e-8:
        return None
    tangent_y /= norm_y
    tangent_x = np.cross(tangent_y, normal)
    # The midpoint of robust projected extents approximates the CAD footprint
    # centre without depending on triangle/pixel density.  It is invariant to
    # which side is nearer to the camera and therefore suitable for multiview
    # fusion of untagged objects.
    relative = footprint - plane_point
    coordinate_x = relative @ tangent_x
    coordinate_y = relative @ tangent_y
    centre_x = 0.5 * sum(np.quantile(coordinate_x, (0.03, 0.97)))
    centre_y = 0.5 * sum(np.quantile(coordinate_y, (0.03, 0.97)))
    support_anchor = plane_point + centre_x * tangent_x + centre_y * tangent_y
    basis = np.column_stack((tangent_x, tangent_y, normal))
    return {
        "centroid": centroid.astype(np.float32),
        "support_anchor": support_anchor.astype(np.float32),
        "camera_from_feature": basis.astype(np.float32),
        "support_height_m": signed_height,
        "foreground_points": int(len(inliers)),
        "footprint_points": int(len(footprint_points)),
    }


def _nearest_mask_component(point_grid, mask, min_points):
    """Return the connected masked component nearest the crop centre."""
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    centre_row, centre_col = (height - 1) / 2.0, (width - 1) / 2.0
    candidates = []
    for row in range(height):
        for column in range(width):
            if not mask[row, column] or visited[row, column]:
                continue
            stack = [(row, column)]
            visited[row, column] = True
            pixels = []
            while stack:
                current_row, current_column = stack.pop()
                pixels.append((current_row, current_column))
                for delta_row in (-1, 0, 1):
                    for delta_column in (-1, 0, 1):
                        if delta_row == 0 and delta_column == 0:
                            continue
                        next_row = current_row + delta_row
                        next_column = current_column + delta_column
                        if (0 <= next_row < height and 0 <= next_column < width
                                and mask[next_row, next_column]
                                and not visited[next_row, next_column]):
                            visited[next_row, next_column] = True
                            stack.append((next_row, next_column))
            if len(pixels) < int(min_points):
                continue
            distance = min(
                (r - centre_row) ** 2 + (c - centre_col) ** 2
                for r, c in pixels)
            candidates.append((distance, -len(pixels), pixels))
    if not candidates:
        return None
    pixels = min(candidates, key=lambda value: value[:2])[2]
    return np.asarray([point_grid[row, column] for row, column in pixels])


def _centered_depth_component(patch_grid, valid_grid, min_points):
    """Return the centre-nearest connected range layer from a box crop."""
    height, width = valid_grid.shape
    if height == 0 or width == 0:
        return None
    ranges = np.linalg.norm(patch_grid, axis=2)
    cy, cx = (height - 1) / 2.0, (width - 1) / 2.0
    radius_y, radius_x = max(1, height // 5), max(1, width // 5)
    top, bottom = max(0, int(cy) - radius_y), min(height, int(cy) + radius_y + 1)
    left, right = max(0, int(cx) - radius_x), min(width, int(cx) + radius_x + 1)
    centre_values = ranges[top:bottom, left:right][
        valid_grid[top:bottom, left:right]]
    all_values = ranges[valid_grid]
    if len(all_values) < int(min_points):
        return None
    seed_range = float(
        np.median(centre_values) if len(centre_values) >= max(3, min_points // 3)
        else np.quantile(all_values, 0.35))
    # Depth sensors become noisier with range.  Keep at least 20 mm, but do
    # not merge surfaces separated by more than 60 mm.
    band = max(0.020, min(0.060, seed_range * 0.04))
    mask = valid_grid & (np.abs(ranges - seed_range) <= band)
    visited = np.zeros_like(mask, dtype=bool)
    components = []
    for row in range(height):
        for column in range(width):
            if not mask[row, column] or visited[row, column]:
                continue
            stack = [(row, column)]
            visited[row, column] = True
            pixels = []
            while stack:
                current_row, current_column = stack.pop()
                pixels.append((current_row, current_column))
                for delta_row in (-1, 0, 1):
                    for delta_column in (-1, 0, 1):
                        if delta_row == 0 and delta_column == 0:
                            continue
                        next_row = current_row + delta_row
                        next_column = current_column + delta_column
                        if (0 <= next_row < height and 0 <= next_column < width
                                and mask[next_row, next_column]
                                and not visited[next_row, next_column]):
                            visited[next_row, next_column] = True
                            stack.append((next_row, next_column))
            if len(pixels) >= int(min_points):
                distance = min(
                    (row_value - cy) ** 2 + (column_value - cx) ** 2
                    for row_value, column_value in pixels)
                components.append((distance, -len(pixels), pixels))
    if not components:
        return None
    pixels = min(components, key=lambda item: (item[0], item[1]))[2]
    return np.asarray(
        [patch_grid[row, column] for row, column in pixels],
        dtype=np.float32)


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
