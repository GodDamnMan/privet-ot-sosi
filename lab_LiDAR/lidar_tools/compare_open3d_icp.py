import argparse
import time
import numpy as np
import open3d as o3d


def ensure_normals(pcd):
    if not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1,
                max_nn=30,
            )
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source_ply")
    parser.add_argument("target_ply")
    parser.add_argument("--max_dist", type=float, default=0.3)
    args = parser.parse_args()

    source = o3d.io.read_point_cloud(args.source_ply)
    target = o3d.io.read_point_cloud(args.target_ply)

    init = np.eye(4)

    # Point-to-Point
    t0 = time.time()
    result_p2p = o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_correspondence_distance=args.max_dist,
        init=init,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    )
    dt_p2p = time.time() - t0

    print("\n=== Open3D Point-to-Point ===")
    print(f"fitness = {result_p2p.fitness:.6f}")
    print(f"inlier_rmse = {result_p2p.inlier_rmse:.6f}")
    print(f"time = {dt_p2p:.3f} s")
    print("T =")
    print(result_p2p.transformation)

    # Point-to-Plane
    ensure_normals(source)
    ensure_normals(target)

    t1 = time.time()
    result_p2l = o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_correspondence_distance=args.max_dist,
        init=init,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    dt_p2l = time.time() - t1

    print("\n=== Open3D Point-to-Plane ===")
    print(f"fitness = {result_p2l.fitness:.6f}")
    print(f"inlier_rmse = {result_p2l.inlier_rmse:.6f}")
    print(f"time = {dt_p2l:.3f} s")
    print("T =")
    print(result_p2l.transformation)

    aligned_p2p = source.transform(result_p2p.transformation.copy())
    o3d.io.write_point_cloud("aligned_open3d_p2p.ply", aligned_p2p)


if __name__ == "__main__":
    main()
