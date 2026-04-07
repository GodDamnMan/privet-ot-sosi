import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from icp_numpy import icp, apply_transform


def load_cloud(path):
    pcd = o3d.io.read_point_cloud(str(path))
    pts = np.asarray(pcd.points)
    if pts.shape[0] == 0:
        raise ValueError(f"Empty cloud: {path}")
    return pcd, pts


def ensure_normals(pcd):
    if not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1,
                max_nn=30,
            )
        )


def pairwise_register(source_pcd, target_pcd, method="numpy", max_dist=0.3, max_iter=80):
    source = np.asarray(source_pcd.points)
    target = np.asarray(target_pcd.points)

    if method == "numpy":
        T, errors = icp(
            source,
            target,
            max_iterations=max_iter,
            tolerance=1e-6,
            max_correspondence_dist=max_dist,
        )
        return T, {"method": "numpy", "final_error": errors[-1] if errors else None, "iterations": len(errors)}

    elif method == "open3d_p2p":
        result = o3d.pipelines.registration.registration_icp(
            source_pcd,
            target_pcd,
            max_correspondence_distance=max_dist,
            init=np.eye(4),
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        )
        return result.transformation, {
            "method": "open3d_p2p",
            "fitness": result.fitness,
            "rmse": result.inlier_rmse,
        }

    elif method == "open3d_p2l":
        ensure_normals(source_pcd)
        ensure_normals(target_pcd)
        result = o3d.pipelines.registration.registration_icp(
            source_pcd,
            target_pcd,
            max_correspondence_distance=max_dist,
            init=np.eye(4),
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )
        return result.transformation, {
            "method": "open3d_p2l",
            "fitness": result.fitness,
            "rmse": result.inlier_rmse,
        }

    else:
        raise ValueError(f"Unknown method: {method}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", help="Ordered list of snapshot .ply files")
    parser.add_argument("--method", default="numpy", choices=["numpy", "open3d_p2p", "open3d_p2l"])
    parser.add_argument("--max_dist", type=float, default=0.3)
    parser.add_argument("--max_iter", type=int, default=80)
    parser.add_argument("--voxel", type=float, default=0.03, help="Optional final downsample voxel")
    parser.add_argument("--output", default="room_map_3d.ply")
    args = parser.parse_args()

    clouds = []
    for path in args.inputs:
        pcd, pts = load_cloud(path)
        clouds.append(pcd)
        print(f"Loaded {path}: {len(pts)} points")

    # первая поза = базовая карта
    global_transforms = [np.eye(4)]
    aligned_clouds = [clouds[0]]

    # pairwise: i -> i-1, потом накапливаем в global
    for i in range(1, len(clouds)):
        print(f"\nRegistering cloud {i} to cloud {i-1} with {args.method} ...")
        T_pair, info = pairwise_register(
            clouds[i], clouds[i - 1],
            method=args.method,
            max_dist=args.max_dist,
            max_iter=args.max_iter,
        )
        print("Pairwise info:", info)

        T_global = global_transforms[i - 1] @ T_pair
        global_transforms.append(T_global)

        aligned_pts = apply_transform(np.asarray(clouds[i].points), T_global)
        aligned_pcd = o3d.geometry.PointCloud()
        aligned_pcd.points = o3d.utility.Vector3dVector(aligned_pts)
        aligned_clouds.append(aligned_pcd)

    colors = [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        [1, 1, 0],
        [1, 0, 1],
        [0, 1, 1],
    ]

    combined = o3d.geometry.PointCloud()
    for i, cloud in enumerate(aligned_clouds):
        cloud.paint_uniform_color(colors[i % len(colors)])
        combined += cloud

    if args.voxel > 0:
        combined = combined.voxel_down_sample(args.voxel)

    o3d.io.write_point_cloud(args.output, combined)
    print(f"\nSaved stitched map to {args.output}")

    for i, T in enumerate(global_transforms):
        print(f"\nGlobal transform for cloud {i}:")
        print(T)

    o3d.visualization.draw_geometries([combined])


if __name__ == "__main__":
    main()
