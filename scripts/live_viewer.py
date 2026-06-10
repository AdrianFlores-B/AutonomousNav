#!/usr/bin/env python3
"""Live 3D point cloud viewer for the Livox Mid-360.

Streams point cloud data from the LidarDriver and renders it in real time
using Open3D. A tkinter control panel provides sliders for spatial filtering
(sphere or box mode) and color mode selection.

Controls (Open3D window):
    Mouse drag      - rotate view
    Scroll          - zoom
    Ctrl+drag       - pan
    C               - cycle color mode
    R               - reset camera to default viewpoint
    Q / Esc / close - quit

Control panel:
    Filter Mode     - sphere (radius) or box (6-axis)
    Radius slider   - max distance from sensor (sphere mode)
    X/Y/Z sliders   - independent min/max per axis (box mode)
    Color Mode      - height or distance coloring
"""

import sys
import time
import tkinter as tk
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
MAX_DISPLAY_POINTS = 600_000
TARGET_FPS = 30


# ---------------------------------------------------------------------------
#  Colorization
# ---------------------------------------------------------------------------

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


def colorize_by_distance(xyz: np.ndarray) -> np.ndarray:
    """Map distance from sensor (vector magnitude) to RGB via turbo colormap."""
    dist = np.linalg.norm(xyz, axis=1)
    d_min, d_max = dist.min(), dist.max()
    span = d_max - d_min
    if span < 0.01:
        norm = np.full(len(dist), 0.5)
    else:
        norm = (dist - d_min) / span
    return turbo(norm)[:, :3]


COLOR_MODES = [
    ("height", colorize_by_height),
    ("distance", colorize_by_distance),
]


# ---------------------------------------------------------------------------
#  Control Panel (tkinter)
# ---------------------------------------------------------------------------

class ControlPanel:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Mid-360 Controls")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._closed = False

        # --- Filter Mode ---
        mode_frame = tk.LabelFrame(self.root, text="Filter Mode",
                                   padx=10, pady=5)
        mode_frame.pack(fill="x", padx=10, pady=(10, 5))

        self._filter_mode = tk.StringVar(value="sphere")
        tk.Radiobutton(mode_frame, text="Sphere (radius)",
                       variable=self._filter_mode, value="sphere",
                       command=self._on_mode_change).pack(anchor="w")
        tk.Radiobutton(mode_frame, text="Box (X / Y / Z)",
                       variable=self._filter_mode, value="box",
                       command=self._on_mode_change).pack(anchor="w")

        # --- Sphere ---
        self._sphere_frame = tk.LabelFrame(self.root, text="Sphere",
                                           padx=10, pady=5)
        self._sphere_frame.pack(fill="x", padx=10, pady=5)

        self._radius = tk.DoubleVar(value=50.0)
        self._radius_scale = tk.Scale(
            self._sphere_frame, label="Radius (m)",
            from_=0.5, to=50.0, resolution=0.1,
            orient="horizontal", variable=self._radius, length=260,
        )
        self._radius_scale.pack(fill="x")

        # --- Box ---
        self._box_frame = tk.LabelFrame(self.root, text="Box",
                                        padx=10, pady=5)
        self._box_frame.pack(fill="x", padx=10, pady=5)

        slider_defs = [
            ("X min (m)", "x_min", -50.0, 50.0, -20.0),
            ("X max (m)", "x_max", -50.0, 50.0,  20.0),
            ("Y min (m)", "y_min", -50.0, 50.0, -20.0),
            ("Y max (m)", "y_max", -50.0, 50.0,  20.0),
            ("Z min (m)", "z_min", -10.0, 10.0,  -2.0),
            ("Z max (m)", "z_max", -10.0, 10.0,  10.0),
        ]
        self._box_vars: dict[str, tk.DoubleVar] = {}
        self._box_scales: list[tk.Scale] = []
        for label, key, lo, hi, default in slider_defs:
            var = tk.DoubleVar(value=default)
            self._box_vars[key] = var
            s = tk.Scale(
                self._box_frame, label=label,
                from_=lo, to=hi, resolution=0.1,
                orient="horizontal", variable=var, length=260,
            )
            s.pack(fill="x")
            self._box_scales.append(s)

        # --- Color Mode ---
        color_frame = tk.LabelFrame(self.root, text="Color Mode",
                                    padx=10, pady=5)
        color_frame.pack(fill="x", padx=10, pady=5)

        self._color_idx = tk.IntVar(value=0)
        for i, (name, _) in enumerate(COLOR_MODES):
            tk.Radiobutton(color_frame, text=name.capitalize(),
                           variable=self._color_idx, value=i).pack(anchor="w")

        # --- Stats ---
        stats_frame = tk.Frame(self.root, padx=10, pady=5)
        stats_frame.pack(fill="x")
        self._stats_var = tk.StringVar(value="Waiting for data...")
        tk.Label(stats_frame, textvariable=self._stats_var,
                 font=("monospace", 9), anchor="w",
                 justify="left").pack(fill="x")

        self._on_mode_change()

    # --- state enable/disable ---

    def _on_mode_change(self):
        is_sphere = self._filter_mode.get() == "sphere"
        self._radius_scale.config(state="normal" if is_sphere else "disabled")
        for s in self._box_scales:
            s.config(state="disabled" if is_sphere else "normal")

    def _on_close(self):
        self._closed = True

    # --- public API ---

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def filter_mode(self) -> str:
        return self._filter_mode.get()

    @property
    def radius(self) -> float:
        return self._radius.get()

    @property
    def box_bounds(self) -> tuple:
        v = self._box_vars
        return (
            v["x_min"].get(), v["x_max"].get(),
            v["y_min"].get(), v["y_max"].get(),
            v["z_min"].get(), v["z_max"].get(),
        )

    @property
    def color_index(self) -> int:
        return self._color_idx.get()

    def set_color_index(self, idx: int):
        self._color_idx.set(idx)

    def update_stats(self, total: int, displayed: int, rate: float):
        self._stats_var.set(
            f"Buffer:    {total:>9,} pts\n"
            f"Displayed: {displayed:>9,} pts\n"
            f"Rate:      {rate:>9,.0f} pts/s"
        )

    def update(self):
        if not self._closed:
            try:
                self.root.update()
            except tk.TclError:
                self._closed = True

    def destroy(self):
        if not self._closed:
            try:
                self.root.destroy()
            except tk.TclError:
                pass
            self._closed = True


# ---------------------------------------------------------------------------
#  Spatial filtering
# ---------------------------------------------------------------------------

def filter_points(xyz: np.ndarray, panel: ControlPanel) -> np.ndarray:
    """Return boolean mask of points that pass the active filter."""
    if panel.filter_mode == "sphere":
        dist = np.linalg.norm(xyz, axis=1)
        return dist <= panel.radius
    else:
        x_min, x_max, y_min, y_max, z_min, z_max = panel.box_bounds
        return (
            (xyz[:, 0] >= x_min) & (xyz[:, 0] <= x_max) &
            (xyz[:, 1] >= y_min) & (xyz[:, 1] <= y_max) &
            (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
        )


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

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

    panel = ControlPanel()

    def on_key_c(vis):
        idx = (panel.color_index + 1) % len(COLOR_MODES)
        panel.set_color_index(idx)
        name = COLOR_MODES[idx][0]
        print(f"[viewer] Color mode: {name}")
        return False

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.register_key_callback(ord("C"), on_key_c)
    vis.create_window("Mid-360 Live", width=1280, height=720)

    pcd = o3d.geometry.PointCloud()
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(pcd)
    vis.add_geometry(axes)

    opt = vis.get_render_option()
    opt.background_color = np.array([0.05, 0.05, 0.1])
    opt.point_size = 2.0

    ring_buf = deque()  # (wall_time, xyz_array) — colors computed each frame
    total_points = 0
    t0 = time.monotonic()
    last_stats = t0
    need_initial_viewpoint = True
    frame_dt = 1.0 / TARGET_FPS

    try:
        while True:
            t_frame = time.monotonic()

            panel.update()
            if panel.closed:
                break

            # --- ingest new data ---
            clouds = driver.get_point_clouds()
            for _ts_ns, _data_type, pts in clouds:
                valid = (pts["x"] != 0) | (pts["y"] != 0) | (pts["z"] != 0)
                pts = pts[valid]
                if len(pts) == 0:
                    continue
                xyz = np.column_stack([
                    pts["x"], pts["y"], pts["z"]
                ]).astype(np.float64) / 1000.0
                ring_buf.append((t_frame, xyz))
                total_points += len(xyz)

            # --- expire old batches ---
            cutoff = t_frame - ACCUMULATE_SEC
            while ring_buf and ring_buf[0][0] < cutoff:
                ring_buf.popleft()

            # --- filter, colorize, render ---
            n_buf = 0
            n_displayed = 0

            if ring_buf:
                all_xyz = np.concatenate([b[1] for b in ring_buf])
                n_buf = len(all_xyz)

                mask = filter_points(all_xyz, panel)
                filtered = all_xyz[mask]

                if len(filtered) > MAX_DISPLAY_POINTS:
                    idx = np.random.choice(
                        len(filtered), MAX_DISPLAY_POINTS, replace=False)
                    filtered = filtered[idx]

                if len(filtered) > 0:
                    _, colorize_fn = COLOR_MODES[panel.color_index]
                    colors = colorize_fn(filtered)
                    pcd.points = o3d.utility.Vector3dVector(filtered)
                    pcd.colors = o3d.utility.Vector3dVector(colors)
                else:
                    pcd.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
                    pcd.colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))

                n_displayed = len(filtered)
                vis.update_geometry(pcd)

                if need_initial_viewpoint and n_displayed > 100:
                    vis.reset_view_point(True)
                    need_initial_viewpoint = False

            if not vis.poll_events():
                break
            vis.update_renderer()

            # --- stats ---
            if t_frame - last_stats >= 3.0:
                elapsed = t_frame - t0
                rate = total_points / elapsed if elapsed > 0 else 0
                devs = driver.devices
                dev_str = ", ".join(
                    f"{d['sn']} ({d['ip']})" for d in devs.values()
                ) or "waiting..."
                print(
                    f"[{elapsed:5.0f}s] {rate:,.0f} pts/s | "
                    f"display: {n_displayed:,} / {n_buf:,} | "
                    f"devices: {dev_str}"
                )
                panel.update_stats(n_buf, n_displayed, rate)
                last_stats = t_frame

            # --- frame rate limit ---
            sleep_for = frame_dt - (time.monotonic() - t_frame)
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        vis.destroy_window()
        panel.destroy()
        driver.stop()
        elapsed = time.monotonic() - t0
        print(f"Total: {total_points:,} points in {elapsed:.1f}s")


if __name__ == "__main__":
    main()
