import argparse
from pathlib import Path

import open3d as o3d


def preprocess_cloud(input_file, output_file, voxel_size=0.05):
    pcd = o3d.io.read_point_cloud(str(input_file))
    print(f"Loaded {len(pcd.points)} points from {input_file}")

    if len(pcd.points) == 0:
        print("[ERROR] Empty point cloud")
        return

    # 1. Downsample
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"After voxel downsample: {len(pcd_down.points)} points")

    # 2. Remove outliers
    pcd_clean, ind = pcd_down.remove_statistical_outlier(
        nb_neighbors=20,
        std_ratio=2.0
    )
    print(f"After outlier removal: {len(pcd_clean.points)} points")

    # 3. Estimate normals
    pcd_clean.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1,
            max_nn=30,
        )
    )

    ok = o3d.io.write_point_cloud(str(output_file), pcd_clean)
    if ok:
        print(f"Saved processed cloud to {output_file}")
    else:
        print(f"[ERROR] Failed to save {output_file}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_file", help="Input .ply file")
    parser.add_argument("output_file", help="Output processed .ply file")
    parser.add_argument("--voxel", type=float, default=0.05, help="Voxel size in meters")
    args = parser.parse_args()

    preprocess_cloud(args.input_file, args.output_file, voxel_size=args.voxel)


if __name__ == "__main__":
    main()
