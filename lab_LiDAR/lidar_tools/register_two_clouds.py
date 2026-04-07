import argparse
import copy
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


def run_registration(source, target, method="p2p", max_dist=0.2, init=None):
    if init is None:
        init = np.eye(4)

    if method == "p2p":
        result = o3d.pipelines.registration.registration_icp(
            source, target,
            max_correspondence_distance=max_dist,
            init=init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        )
    elif method == "p2l":
        ensure_normals(source)
        ensure_normals(target)
        result = o3d.pipelines.registration.registration_icp(
            source, target,
            max_correspondence_distance=max_dist,
            init=init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )
    else:
        raise ValueError("method must be p2p or p2l")

    return result


def visualize_pair(source, target, T, title="result"):
    src = copy.deepcopy(source)
    tgt = copy.deepcopy(target)

    src.transform(T)
    src.paint_uniform_color([1, 0, 0])  # red
    tgt.paint_uniform_color([0, 1, 0])  # green

    print(f"Showing: {title}")
    o3d.visualization.draw_geometries([src, tgt])


def save_merged(source, target, T, output_path, voxel=0.03):
    src = copy.deepcopy(source)
    tgt = copy.deepcopy(target)

    src.transform(T)
    src.paint_uniform_color([1, 0, 0])
    tgt.paint_uniform_color([0, 1, 0])

    merged = src + tgt
    if voxel > 0:
        merged = merged.voxel_down_sample(voxel)

    o3d.io.write_point_cloud(output_path, merged)
    print(f"Saved merged map to {output_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source_ply")
    parser.add_argument("target_ply")
    parser.add_argument("--max_dist", type=float, default=0.2)
    parser.add_argument("--method", choices=["p2p", "p2l", "p2p_then_p2l"], default="p2p")
    parser.add_argument("--voxel", type=float, default=0.03)
    parser.add_argument("--output", default="room_map_3d.ply")

    parser.add_argument("--tx", type=float, default=0.0)
    parser.add_argument("--ty", type=float, default=0.0)
    parser.add_argument("--tz", type=float, default=0.0)
    parser.add_argument("--yaw_deg", type=float, default=0.0)

    args = parser.parse_args()

    source = o3d.io.read_point_cloud(args.source_ply)
    target = o3d.io.read_point_cloud(args.target_ply)

    print(f"Loaded source: {len(source.points)} points")
    print(f"Loaded target: {len(target.points)} points")

    yaw = np.deg2rad(args.yaw_deg)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1],
    ])

    init = np.eye(4)
    init[:3, :3] = Rz
    init[:3, 3] = [args.tx, args.ty, args.tz]

    print("Initial guess:")
    print(init)

    t0 = time.time()

    if args.method == "p2p_then_p2l":
        coarse = run_registration(
            source, target,
            method="p2p",
            max_dist=max(args.max_dist, 0.2),
            init=init,
        )
        print("\nCoarse p2p:")
        print(f"fitness = {coarse.fitness:.6f}")
        print(f"inlier_rmse = {coarse.inlier_rmse:.6f}")
        print(coarse.transformation)

        refined = run_registration(
            source, target,
            method="p2l",
            max_dist=args.max_dist,
            init=coarse.transformation,
        )
        result = refined
    else:
        result = run_registration(
            source, target,
            method=args.method,
            max_dist=args.max_dist,
            init=init,
        )

    dt = time.time() - t0

    print(f"\nMethod: {args.method}")
    print(f"fitness = {result.fitness:.6f}")
    print(f"inlier_rmse = {result.inlier_rmse:.6f}")
    print(f"time = {dt:.3f} s")
    print("Transformation =")
    print(result.transformation)

    visualize_pair(source, target, result.transformation, title=f"{args.method}, d={args.max_dist}")
    save_merged(source, target, result.transformation, args.output, voxel=args.voxel)
if __name__ == "__main__":
    main()
