import argparse
import struct
from pathlib import Path

import numpy as np
import open3d as o3d
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def msg_to_numpy(msg):
    """Convert deserialized PointCloud2 message to Nx3 numpy array."""
    field_map = {f.name: f for f in msg.fields}
    required = ("x", "y", "z")
    if not all(name in field_map for name in required):
        return np.empty((0, 3), dtype=np.float64)

    ox = field_map["x"].offset
    oy = field_map["y"].offset
    oz = field_map["z"].offset

    step = msg.point_step
    data = bytes(msg.data)
    n = msg.width * msg.height

    points = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        points[i, 0] = struct.unpack_from("<f", data, base + ox)[0]
        points[i, 1] = struct.unpack_from("<f", data, base + oy)[0]
        points[i, 2] = struct.unpack_from("<f", data, base + oz)[0]

    valid = (
        np.isfinite(points).all(axis=1)
        & (np.linalg.norm(points, axis=1) > 0.1)
    )
    return points[valid].astype(np.float64)


def extract_bag(bag_path, topic):
    """Read all PointCloud2 messages from bag and accumulate into one array."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    all_points = []

    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == topic]

        if not conns:
            print(f"[ERROR] Topic '{topic}' not found in bag: {bag_path}")
            print("Available topics:")
            for c in reader.connections:
                print(f"  - {c.topic} [{c.msgtype}]")
            return np.empty((0, 3), dtype=np.float64)

        msg_count = 0
        for conn, timestamp, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            pts = msg_to_numpy(msg)
            if len(pts) > 0:
                all_points.append(pts)
                msg_count += 1

        print(f"Read {msg_count} messages from topic {topic}")

    if not all_points:
        return np.empty((0, 3), dtype=np.float64)

    points = np.vstack(all_points)
    print(f"Accumulated {len(points)} raw points")
    return points


def save_cloud(points, filename):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    ok = o3d.io.write_point_cloud(str(filename), pcd)
    if ok:
        print(f"Saved {len(points)} points to {filename}")
    else:
        print(f"[ERROR] Failed to save cloud to {filename}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", help="Path to rosbag folder")
    parser.add_argument(
        "--topic",
        default="/livox/lidar",
        help="PointCloud2 topic name"
    )
    parser.add_argument(
        "--output",
        default="snapshot.ply",
        help="Output .ply filename"
    )
    args = parser.parse_args()

    points = extract_bag(args.bag_path, args.topic)

    if len(points) == 0:
        print("[ERROR] No valid points extracted")
        return

    save_cloud(points, args.output)


if __name__ == "__main__":
    main()
