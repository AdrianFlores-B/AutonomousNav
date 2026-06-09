#!/usr/bin/env python3
"""Live 3D point cloud viewer for the Livox Mid-360.

Streams point cloud data from the LidarDriver and renders it in real time
using Open3D. Points are colored by height (z-axis) using a turbo colormap.

Controls (Open3D window):
    Mouse drag      - rotate view
    Scroll          - zoom
    Ctrl+drag       - pan
    R               - reset camera to default viewpoint
    Q / Esc / close - quit

Terminal controls:
    Ctrl+C          - quit
"""

import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import open3d as o3d
from matplotlib.cm import turbo

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from nav import LidarDriver

SDK_LIB = (
    Path(__file__).resolve().parent.parent
    / "Livox-SDK2" / "build" / "sdk_core" / "liblivox_lidar_sdk_shared.so"
)
CONFIG = Path(__file__).resolve().parent.parent / "mid360_config.json"

ACCUMULATE_SEC = 1.5
VOXEL_SIZE = 0.02  # meters, for downsampling when accumulated cloud is large
MAX_DISPLAY_POINTS = 600_000
TARGET_FPS = 30


def colorize_by_height(xyz: np.ndarray) -> np.ndarray:
    """Map z (height) values to RGB via turbo colormap."""
    z = xyz[:, 2]
    z_min, z_max = z.min(), z.max()
    span = z_max - z_min
    if span < 0.01:
        norm = np.full(len(z), 0.5)
    else:
        norm = (z - z_min) / span
    return turbo(norm)[:, :3]


def main():
    if not SDK_LIB.exists():
        print(f"SDK library not found: {SDK_LIB}")
        sys.exit(1)
    if not CONFIG.exists():
        print(f"Config not found: {CONFIG}")
        sys.exit(1)

    driver = LidarDriver(SDK_LIB)
    driver.start(CONFIG)
    print("SDK initialized, waiting for LiDAR data...")

    vis = o3d.visualization.Visualizer()
    vis.create_window("Mid-360 Live", width=1280, height=720)

    pcd = o3d.geometry.PointCloud()
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(pcd)
    vis.add_geometry(axes)

    opt = vis.get_render_option()
    opt.background_color = np.array([0.05, 0.05, 0.1])
    opt.point_size = 1.5

    ring_buf = deque()  # (wall_time, xyz_array, color_array)
    total_points = 0
    t0 = time.monotonic()
    last_stats = t0
    need_initial_viewpoint = True
    frame_dt = 1.0 / TARGET_FPS

    try:
        while True:
            t_frame = time.monotonic()

            clouds = driver.get_point_clouds()
            for _ts_ns, _data_type, pts in clouds:
                valid = (pts["x"] != 0) | (pts["y"] != 0) | (pts["z"] != 0)
                pts = pts[valid]
                if len(pts) == 0:
                    continue
                xyz = np.column_stack([
                    pts["x"], pts["y"], pts["z"]
                ]).astype(np.float64) / 1000.0
                colors = colorize_by_height(xyz)
                ring_buf.append((t_frame, xyz, colors))
                total_points += len(xyz)

            cutoff = t_frame - ACCUMULATE_SEC
            while ring_buf and ring_buf[0][0] < cutoff:
                ring_buf.popleft()

            if ring_buf:
                all_xyz = np.concatenate([b[1] for b in ring_buf])
                all_clr = np.concatenate([b[2] for b in ring_buf])

                if len(all_xyz) > MAX_DISPLAY_POINTS:
                    idx = np.random.choice(len(all_xyz), MAX_DISPLAY_POINTS, replace=False)
                    all_xyz = all_xyz[idx]
                    all_clr = all_clr[idx]

                pcd.points = o3d.utility.Vector3dVector(all_xyz)
                pcd.colors = o3d.utility.Vector3dVector(all_clr)
                vis.update_geometry(pcd)

                if need_initial_viewpoint and len(all_xyz) > 100:
                    vis.reset_view_point(True)
                    need_initial_viewpoint = False

            if not vis.poll_events():
                break
            vis.update_renderer()

            if t_frame - last_stats >= 3.0:
                elapsed = t_frame - t0
                n_display = len(pcd.points)
                n_buf = sum(len(b[1]) for b in ring_buf)
                rate = total_points / elapsed if elapsed > 0 else 0
                devs = driver.devices
                dev_str = ", ".join(
                    f"{d['sn']} ({d['ip']})" for d in devs.values()
                ) or "waiting..."
                print(
                    f"[{elapsed:5.0f}s] {rate:,.0f} pts/s | "
                    f"display: {n_display:,} | buffer: {n_buf:,} | "
                    f"devices: {dev_str}"
                )
                last_stats = t_frame

            sleep_for = frame_dt - (time.monotonic() - t_frame)
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        vis.destroy_window()
        driver.stop()
        elapsed = time.monotonic() - t0
        print(f"Total: {total_points:,} points in {elapsed:.1f}s")


if __name__ == "__main__":
    main()
