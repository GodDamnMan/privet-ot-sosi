# lidar_tools

Offline processing scripts for the Livox MID-70 lab. Run these from inside the Docker container (or any environment with Open3D, rosbags, NumPy, SciPy installed).

All scripts must be run from **this directory**:
```bash
cd ~/ros2_ws/src/lidar_tools   # or wherever you cloned the repo
```

---

## Scripts

### 1. `extract_snapshot.py` — bag → `.ply`

Reads all `PointCloud2` messages from a ROS 2 bag and accumulates them into a single dense point cloud saved as a `.ply` file.

```bash
python3 extract_snapshot.py <bag_folder> [--topic TOPIC] [--output OUTPUT]
```

| Argument | Default | Description |
|---|---|---|
| `bag_folder` | required | Path to the rosbag directory (e.g. `snapshot_1/`) |
| `--topic` | `/livox/lidar` | PointCloud2 topic name |
| `--output` | `snapshot.ply` | Output `.ply` filename |

**Example:**
```bash
python3 extract_snapshot.py ../bag_files/snapshot_1 --output ../processed_clouds/snapshot_01.ply
python3 extract_snapshot.py ../bag_files/snapshot_2 --output ../processed_clouds/snapshot_02.ply
```

---

### 2. `preprocess_cloud.py` — voxel downsample + outlier removal + normals

Preprocesses a raw `.ply` cloud: voxel downsampling → statistical outlier removal → normal estimation. Output is ready for ICP.

```bash
python3 preprocess_cloud.py <input.ply> <output.ply> [--voxel SIZE]
```

| Argument | Default | Description |
|---|---|---|
| `input.ply` | required | Raw point cloud |
| `output.ply` | required | Processed output |
| `--voxel` | `0.05` | Voxel grid size in meters |

**Example:**
```bash
python3 preprocess_cloud.py ../processed_clouds/snapshot_01.ply ../processed_clouds/snapshot_01_clean.ply
python3 preprocess_cloud.py ../processed_clouds/snapshot_02.ply ../processed_clouds/snapshot_02_clean.ply
```

---

### 3. `icp_numpy.py` — ICP implementation (library, not a script)

Contains the NumPy/SciPy point-to-point ICP implementation used by other scripts:
- `find_correspondences(source, target_tree, max_dist)`
- `compute_rigid_transform(src_points, tgt_points)` — SVD-based (Arun method)
- `icp(source, target, ...)` — full iterative loop, returns `(T_4x4, errors_list)`
- `apply_transform(points, T)`

Import it in your own scripts:
```python
from icp_numpy import icp, apply_transform
```

---

### 4. `run_icp_numpy.py` — run NumPy ICP on two clouds

Registers `source.ply` to `target.ply` using the custom NumPy ICP, saves the aligned cloud and a convergence plot.

```bash
python3 run_icp_numpy.py <source.ply> <target.ply> [--max_dist D] [--max_iter N]
```

| Argument | Default | Description |
|---|---|---|
| `source.ply` | required | Source cloud (will be moved) |
| `target.ply` | required | Target (fixed) cloud |
| `--max_dist` | `0.3` | Max correspondence distance (m) |
| `--max_iter` | `50` | Max ICP iterations |

**Outputs:** `aligned_numpy_icp.ply`, `numpy_icp_errors.png`

**Example:**
```bash
python3 run_icp_numpy.py ../processed_clouds/snapshot_01_clean.ply \
                         ../processed_clouds/snapshot_02_clean.ply \
                         --max_dist 0.3 --max_iter 50
```

---

### 5. `compare_open3d_icp.py` — Open3D P2P vs P2L comparison

Runs both Open3D ICP variants on the same pair and prints fitness, RMSE, and time for each.

```bash
python3 compare_open3d_icp.py <source.ply> <target.ply> [--max_dist D]
```

**Example:**
```bash
python3 compare_open3d_icp.py ../processed_clouds/snapshot_01_clean.ply \
                               ../processed_clouds/snapshot_02_clean.ply
```

**Output (example):**
```
=== Open3D Point-to-Point ===
fitness       = 0.982100
inlier_rmse   = 0.000794
time          = 0.14 s

=== Open3D Point-to-Plane ===
fitness       = 0.987400
inlier_rmse   = 0.000612
time          = 0.21 s
```

Saves `aligned_open3d_p2p.ply` (P2P result).

---

### 6. `register_two_clouds.py` — register, visualize, and save merged map

Registers two clouds, opens an Open3D visualization window (red = source, green = target), and saves the merged colored map.

```bash
python3 register_two_clouds.py <source.ply> <target.ply> \
    [--method p2p|p2l|p2p_then_p2l] \
    [--max_dist D] [--voxel V] [--output FILE] \
    [--tx X] [--ty Y] [--tz Z] [--yaw_deg A]
```

| Argument | Default | Description |
|---|---|---|
| `--method` | `p2p` | `p2p`, `p2l`, or `p2p_then_p2l` (coarse+fine) |
| `--max_dist` | `0.2` | Max correspondence distance (m) |
| `--voxel` | `0.03` | Final merged map voxel size (m) |
| `--output` | `room_map_3d.ply` | Output merged cloud |
| `--tx/ty/tz` | `0` | Initial translation guess (m) |
| `--yaw_deg` | `0` | Initial yaw rotation guess (degrees) |

**Example — P2L with a 90° initial guess:**
```bash
python3 register_two_clouds.py ../processed_clouds/snapshot_01_clean.ply \
                                ../processed_clouds/snapshot_02_clean.ply \
                                --method p2l --yaw_deg 90 \
                                --output ../processed_clouds/room_map_3d.ply
```

---

### 7. `stitch_snapshots.py` — stitch 2+ clouds into a 3D map

Pairwise-registers an ordered list of clouds (cloud 1→0, 2→1, …), accumulates global transforms, paints each cloud a different color, and saves the combined map.

```bash
python3 stitch_snapshots.py <cloud1.ply> <cloud2.ply> [cloud3.ply ...] \
    [--method numpy|open3d_p2p|open3d_p2l] \
    [--max_dist D] [--max_iter N] [--voxel V] [--output FILE]
```

| Argument | Default | Description |
|---|---|---|
| `--method` | `numpy` | Registration backend |
| `--max_dist` | `0.3` | Max correspondence distance (m) |
| `--max_iter` | `80` | Max iterations (NumPy ICP only) |
| `--voxel` | `0.03` | Final map voxel size (0 = skip) |
| `--output` | `room_map_3d.ply` | Output file |

**Example:**
```bash
python3 stitch_snapshots.py \
    ../processed_clouds/snapshot_01_clean.ply \
    ../processed_clouds/snapshot_02_clean.ply \
    --method open3d_p2l \
    --output ../processed_clouds/room_map_3d.ply
```

Colors assigned per snapshot: red → green → blue → yellow → magenta → cyan.

---

## Typical full workflow

```bash
# 1. Extract raw clouds from bags
python3 extract_snapshot.py ../bag_files/snapshot_1 --output ../processed_clouds/snapshot_01.ply
python3 extract_snapshot.py ../bag_files/snapshot_2 --output ../processed_clouds/snapshot_02.ply

# 2. Preprocess
python3 preprocess_cloud.py ../processed_clouds/snapshot_01.ply ../processed_clouds/snapshot_01_clean.ply
python3 preprocess_cloud.py ../processed_clouds/snapshot_02.ply ../processed_clouds/snapshot_02_clean.ply

# 3. (Optional) compare ICP methods
python3 compare_open3d_icp.py ../processed_clouds/snapshot_01_clean.ply \
                               ../processed_clouds/snapshot_02_clean.ply

# 4. Stitch into final map
python3 stitch_snapshots.py \
    ../processed_clouds/snapshot_01_clean.ply \
    ../processed_clouds/snapshot_02_clean.ply \
    --method open3d_p2l \
    --output ../processed_clouds/room_map_3d.ply
```
