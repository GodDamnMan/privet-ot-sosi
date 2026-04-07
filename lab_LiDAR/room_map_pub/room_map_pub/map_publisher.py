from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def pack_rgb_uint32(r, g, b):
    return (int(r) << 16) | (int(g) << 8) | int(b)


class MapPublisher(Node):
    def __init__(self):
        super().__init__("map_3d_publisher")

        self.declare_parameter("ply_path", "/home/fabian/ros2_ws/src/processed_clouds/room_map_3d.ply")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/map_3d")
        self.declare_parameter("publish_rate", 1.0)

        ply_path = self.get_parameter("ply_path").value
        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.publisher_ = self.create_publisher(PointCloud2, topic, 10)
        self.points = self.load_colored_points(ply_path)

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_cloud)

        self.get_logger().info(f"Loaded {len(self.points)} points from {ply_path}")
        self.get_logger().info(f"Publishing to {topic} with frame_id={self.frame_id}")

    def load_colored_points(self, ply_path):
        pcd = o3d.io.read_point_cloud(str(Path(ply_path)))
        xyz = np.asarray(pcd.points, dtype=np.float32)

        if xyz.shape[0] == 0:
            raise RuntimeError(f"Empty cloud: {ply_path}")

        if pcd.has_colors():
            colors = (np.asarray(pcd.colors) * 255.0).clip(0, 255).astype(np.uint8)
        else:
            colors = np.full((xyz.shape[0], 3), 255, dtype=np.uint8)

        points = []
        for i in range(xyz.shape[0]):
            x, y, z = xyz[i]
            r, g, b = colors[i]
            rgb = pack_rgb_uint32(r, g, b)
            points.append([float(x), float(y), float(z), int(rgb)])

        return points

    def publish_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]

        msg = point_cloud2.create_cloud(header, fields, self.points)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
