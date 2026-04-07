import numpy as np
from scipy.spatial import KDTree


def find_correspondences(source_points, target_tree, max_dist):
    """
    For each point in source, find the nearest neighbor in target.
    """
    distances, indices = target_tree.query(source_points)

    valid = np.isfinite(distances) & (distances <= max_dist)
    src_idx = np.where(valid)[0]
    tgt_idx = indices[valid]

    return src_idx, tgt_idx


def compute_rigid_transform(src_points, tgt_points):
    """
    Compute optimal rotation R and translation t using SVD.
    Minimizes: sum ||R @ src_i + t - tgt_i||^2
    """
    if src_points.shape != tgt_points.shape:
        raise ValueError("src_points and tgt_points must have the same shape")
    if src_points.shape[0] < 3:
        raise ValueError("Need at least 3 correspondences")

    # Step 1: centroids
    centroid_src = np.mean(src_points, axis=0)
    centroid_tgt = np.mean(tgt_points, axis=0)

    # Step 2: center points
    src_centered = src_points - centroid_src
    tgt_centered = tgt_points - centroid_tgt

    # Step 3: cross-covariance
    H = src_centered.T @ tgt_centered

    # Step 4: SVD
    U, S, Vt = np.linalg.svd(H)

    # Step 5: rotation
    R = Vt.T @ U.T

    # reflection fix
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T

    # Step 6: translation
    t = centroid_tgt - R @ centroid_src

    return R, t


def icp(source, target, max_iterations=50, tolerance=1e-6, max_correspondence_dist=0.5):
    """
    Point-to-point ICP algorithm.
    """
    if source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source must have shape (N, 3)")
    if target.ndim != 2 or target.shape[1] != 3:
        raise ValueError("target must have shape (M, 3)")
    if len(source) == 0 or len(target) == 0:
        raise ValueError("source and target must be non-empty")

    src = source.copy()
    target_tree = KDTree(target)

    T_total = np.eye(4)
    errors = []

    for i in range(max_iterations):
        # Step 1: correspondences
        src_idx, tgt_idx = find_correspondences(src, target_tree, max_correspondence_dist)

        if len(src_idx) < 3:
            print(f"Stopped: not enough correspondences ({len(src_idx)})")
            break

        matched_src = src[src_idx]
        matched_tgt = target[tgt_idx]

        # Step 2: rigid transform
        R, t = compute_rigid_transform(matched_src, matched_tgt)

        # Step 3: apply transform
        src = (R @ src.T).T + t

        # Step 4: accumulate transform
        T_inc = np.eye(4)
        T_inc[:3, :3] = R
        T_inc[:3, 3] = t
        T_total = T_inc @ T_total

        # Step 5: mean error + convergence
        transformed_matched = (R @ matched_src.T).T + t
        mean_error = np.mean(np.linalg.norm(transformed_matched - matched_tgt, axis=1))
        errors.append(mean_error)

        print(f"iter={i+1:02d} pairs={len(src_idx)} mean_error={mean_error:.6f}")

        if len(errors) > 1 and abs(errors[-2] - errors[-1]) < tolerance:
            print(f"Converged at iteration {i+1}")
            break

    return T_total, errors


def apply_transform(points, T):
    return (T[:3, :3] @ points.T).T + T[:3, 3]
