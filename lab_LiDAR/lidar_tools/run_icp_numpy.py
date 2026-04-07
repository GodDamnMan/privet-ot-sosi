import argparse
import time

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

from icp_numpy import icp, apply_transform


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source_ply")
    parser.add_argument("target_ply")
    parser.add_argument("--max_dist", type=float, default=0.3)
    parser.add_argument("--max_iter", type=int, default=50)
    args = parser.parse_args()

    source_pcd = o3d.io.read_point_cloud(args.source_ply)
    target_pcd = o3d.io.read_point_cloud(args.target_ply)

    source = np.asarray(source_pcd.points)
    target = np.asarray(target_pcd.points)

    print(f"Source points: {len(source)}")
    print(f"Target points: {len(target)}")

    t0 = time.time()
    T, errors = icp(
        source,
        target,
        max_iterations=args.max_iter,
        tolerance=1e-6,
        max_correspondence_dist=args.max_dist,
    )
    dt = time.time() - t0

    aligned = apply_transform(source, T)

    aligned_pcd = o3d.geometry.PointCloud()
    aligned_pcd.points = o3d.utility.Vector3dVector(aligned)
    o3d.io.write_point_cloud("aligned_numpy_icp.ply", aligned_pcd)

    print("\nFinal transform:")
    print(T)
    print(f"Time: {dt:.3f} s")
    print(f"Final error: {errors[-1] if errors else 'n/a'}")

    plt.plot(errors)
    plt.xlabel("Iteration")
    plt.ylabel("Mean error")
    plt.title("NumPy ICP convergence")
    plt.grid(True)
    plt.savefig("numpy_icp_errors.png", dpi=150)
    print("Saved aligned cloud to aligned_numpy_icp.ply")
    print("Saved convergence plot to numpy_icp_errors.png")


if __name__ == "__main__":
    main()
