import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs_py import point_cloud2


# ── PointCloud2 helpers ───────────────────────────────────────────────────────

def msg_to_numpy(msg):
    """Convert PointCloud2 message to (N, 3) float64 array."""
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float64)

    ox = field_map['x'].offset
    oy = field_map['y'].offset
    oz = field_map['z'].offset
    step = msg.point_step
    data = bytes(msg.data)
    n = msg.width * msg.height

    pts = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        pts[i, 0] = struct.unpack_from('<f', data, base + ox)[0]
        pts[i, 1] = struct.unpack_from('<f', data, base + oy)[0]
        pts[i, 2] = struct.unpack_from('<f', data, base + oz)[0]

    valid = np.isfinite(pts).all(axis=1) & (np.linalg.norm(pts, axis=1) > 0.1)
    return pts[valid].astype(np.float64)


def numpy_to_pc2(header, points):
    """Convert (N, 3) numpy array to PointCloud2 message."""
    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    pts_list = points.astype(np.float32).tolist()
    return point_cloud2.create_cloud(header, fields, pts_list)


# ── Voxel downsampling (NumPy) ────────────────────────────────────────────────

def voxel_downsample(points, voxel_size):
    if len(points) == 0:
        return points
    mins = points.min(axis=0)
    indices = np.floor((points - mins) / voxel_size).astype(np.int32)
    keys = indices[:, 0] * 1_000_003 + indices[:, 1] * 1_009 + indices[:, 2]
    _, first = np.unique(keys, return_index=True)
    return points[first]


# ── RANSAC plane fitting ──────────────────────────────────────────────────────

def ransac_plane(points, n_iterations=1000, distance_threshold=0.02):
    """
    Fit the dominant plane in `points` using RANSAC.

    Returns
    -------
    normal : (3,) unit normal  or None if fitting failed
    d      : plane offset so that  normal @ p + d = 0  for inlier p
    mask   : (N,) bool  True = inlier
    """
    n = len(points)
    if n < 3:
        return None, None, np.zeros(n, dtype=bool)

    best_count = 0
    best_normal = None
    best_d = None

    for _ in range(n_iterations):
        idx = np.random.choice(n, 3, replace=False)
        p0, p1, p2 = points[idx]

        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)
        length = np.linalg.norm(normal)
        if length < 1e-10:
            continue
        normal /= length
        d = -normal @ p0

        dist = np.abs(points @ normal + d)
        count = np.sum(dist < distance_threshold)

        if count > best_count:
            best_count = count
            best_normal = normal.copy()
            best_d = d

    if best_normal is None:
        return None, None, np.zeros(n, dtype=bool)

    mask = np.abs(points @ best_normal + best_d) < distance_threshold
    return best_normal, best_d, mask


# ── Plane classification ──────────────────────────────────────────────────────

def classify_plane(normal):
    """
    Returns 'floor' if normal is nearly vertical,
            'wall'  if normal is nearly horizontal,
            'other' otherwise.
    """
    vertical = abs(normal[2])       # |cos(angle with Z)|
    if vertical > 0.85:
        return 'floor'
    if vertical < 0.35:
        return 'wall'
    return 'other'


# ── ROS 2 node ────────────────────────────────────────────────────────────────

PLANE_COLORS = [
    (0.2, 0.6, 1.0),   # blue
    (1.0, 0.4, 0.2),   # orange
    (0.2, 1.0, 0.4),   # green
    (1.0, 0.9, 0.1),   # yellow
    (0.9, 0.2, 0.9),   # magenta
]

FLOOR_COLOR = (0.2, 0.8, 1.0)   # cyan
WALL_COLOR  = (1.0, 0.5, 0.1)   # orange


class PlaneDetectorNode(Node):

    def __init__(self):
        super().__init__('plane_detector')

        self.declare_parameter('voxel_size',          0.05)
        self.declare_parameter('distance_threshold',  0.02)
        self.declare_parameter('num_iterations',      1000)
        self.declare_parameter('max_planes',          5)
        self.declare_parameter('min_inlier_ratio',    0.05)

        self.sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self._cloud_cb, 10)

        self.pub_markers = self.create_publisher(MarkerArray, '/planes/markers', 10)
        self.pub_floor   = self.create_publisher(PointCloud2, '/planes/floor',   10)
        self.pub_walls   = self.create_publisher(PointCloud2, '/planes/walls',   10)
        self.pub_objects = self.create_publisher(PointCloud2, '/planes/objects', 10)

        self.get_logger().info('PlaneDetectorNode ready, subscribing to /livox/lidar')

    # ── callback ──────────────────────────────────────────────────────────────

    def _cloud_cb(self, msg):
        t_start = time.time()

        pts = msg_to_numpy(msg)
        if len(pts) == 0:
            return

        # Downsample
        voxel_size = self.get_parameter('voxel_size').value
        pts = voxel_downsample(pts, voxel_size)
        t_down = time.time()

        # Iterative RANSAC
        dist_thresh     = self.get_parameter('distance_threshold').value
        n_iter          = self.get_parameter('num_iterations').value
        max_planes      = self.get_parameter('max_planes').value
        min_ratio       = self.get_parameter('min_inlier_ratio').value

        remaining = pts.copy()
        planes = []   # list of (normal, d, inlier_points, label)

        for _ in range(max_planes):
            if len(remaining) < 10:
                break

            normal, d, mask = ransac_plane(remaining, n_iter, dist_thresh)
            if normal is None:
                break

            ratio = mask.sum() / len(remaining)
            if ratio < min_ratio:
                break

            inliers  = remaining[mask]
            remaining = remaining[~mask]
            label    = classify_plane(normal)
            planes.append((normal, d, inliers, label))

        t_ransac = time.time()

        # Publish
        header = Header()
        header.stamp    = msg.header.stamp
        header.frame_id = msg.header.frame_id

        self._publish_markers(header, planes)
        self._publish_typed_clouds(header, planes, remaining)

        t_pub = time.time()

        self.get_logger().info(
            f'clouds={len(pts)} planes={len(planes)} '
            f'down={1000*(t_down-t_start):.1f}ms '
            f'ransac={1000*(t_ransac-t_down):.1f}ms '
            f'pub={1000*(t_pub-t_ransac):.1f}ms'
        )

    # ── marker publishing ─────────────────────────────────────────────────────

    def _publish_markers(self, header, planes):
        marker_array = MarkerArray()

        # Delete old markers first
        delete_all = Marker()
        delete_all.header  = header
        delete_all.action  = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for i, (normal, d, inliers, label) in enumerate(planes):
            centroid = inliers.mean(axis=0)

            color = FLOOR_COLOR if label == 'floor' else (
                    WALL_COLOR  if label == 'wall'  else
                    PLANE_COLORS[i % len(PLANE_COLORS)])

            # Flat disc to represent the plane extent
            m = Marker()
            m.header    = header
            m.ns        = 'planes'
            m.id        = i
            m.type      = Marker.CUBE
            m.action    = Marker.ADD

            m.pose.position.x = centroid[0]
            m.pose.position.y = centroid[1]
            m.pose.position.z = centroid[2]

            # Orient cube so its Z-face aligns with the plane normal
            # Build quaternion from normal
            z = np.array([0.0, 0.0, 1.0])
            axis = np.cross(z, normal)
            axis_len = np.linalg.norm(axis)
            if axis_len < 1e-6:
                # Already aligned (or anti-aligned)
                qw, qx, qy, qz = (1.0, 0.0, 0.0, 0.0) if normal[2] > 0 else (0.0, 1.0, 0.0, 0.0)
            else:
                axis /= axis_len
                angle = np.arccos(np.clip(np.dot(z, normal), -1.0, 1.0))
                s = np.sin(angle / 2.0)
                qw = np.cos(angle / 2.0)
                qx, qy, qz = axis * s

            m.pose.orientation.w = float(qw)
            m.pose.orientation.x = float(qx)
            m.pose.orientation.y = float(qy)
            m.pose.orientation.z = float(qz)

            # Scale: span of inliers, thin in normal direction
            span = inliers.max(axis=0) - inliers.min(axis=0)
            m.scale.x = max(float(span[0]), 0.1)
            m.scale.y = max(float(span[1]), 0.1)
            m.scale.z = 0.02   # thin slab

            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = 0.5

            # Text label above plane
            t = Marker()
            t.header    = header
            t.ns        = 'plane_labels'
            t.id        = i
            t.type      = Marker.TEXT_VIEW_FACING
            t.action    = Marker.ADD
            t.pose.position.x = centroid[0]
            t.pose.position.y = centroid[1]
            t.pose.position.z = centroid[2] + 0.3
            t.pose.orientation.w = 1.0
            t.scale.z   = 0.15
            t.color.r   = 1.0
            t.color.g   = 1.0
            t.color.b   = 1.0
            t.color.a   = 1.0
            t.text      = f'{label} ({len(inliers)} pts)'

            marker_array.markers.append(m)
            marker_array.markers.append(t)

        self.pub_markers.publish(marker_array)

    # ── typed point cloud publishing ──────────────────────────────────────────

    def _publish_typed_clouds(self, header, planes, remaining):
        floor_pts = []
        wall_pts  = []

        for normal, d, inliers, label in planes:
            if label == 'floor':
                floor_pts.append(inliers)
            elif label == 'wall':
                wall_pts.append(inliers)

        if floor_pts:
            self.pub_floor.publish(numpy_to_pc2(header, np.vstack(floor_pts)))
        if wall_pts:
            self.pub_walls.publish(numpy_to_pc2(header, np.vstack(wall_pts)))
        if len(remaining) > 0:
            self.pub_objects.publish(numpy_to_pc2(header, remaining))


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PlaneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
