#!/usr/bin/env python3
"""Live capture test: prints point cloud + IMU stats from the Livox Mid-360."""

import sys
import time
from pathlib import Path

import numpy as np

# add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from nav import LidarDriver

SDK_LIB = Path(__file__).resolve().parent.parent / "Livox-SDK2" / "build" / "sdk_core" / "liblivox_lidar_sdk_shared.so"
CONFIG = Path(__file__).resolve().parent.parent / "mid360_config.json"


def main():
    if not SDK_LIB.exists():
        print(f"SDK library not found: {SDK_LIB}")
        sys.exit(1)
    if not CONFIG.exists():
        print(f"Config not found: {CONFIG}")
        sys.exit(1)

    driver = LidarDriver(SDK_LIB)
    driver.start(CONFIG)
    print(f"SDK initialized, waiting for LiDAR data...\n")

    total_points = 0
    total_imu = 0
    t0 = time.monotonic()
    last_print = 0.0

    try:
        while True:
            clouds = driver.get_point_clouds()
            imus = driver.get_imu_samples()

            for ts_ns, data_type, pts in clouds:
                total_points += len(pts)

            for ts_ns, samples in imus:
                total_imu += len(samples)

            elapsed = time.monotonic() - t0

            if elapsed - last_print >= 2.0:
                last_print = elapsed
                pts_rate = total_points / elapsed if elapsed > 0 else 0
                imu_rate = total_imu / elapsed if elapsed > 0 else 0

                info = ""
                if clouds:
                    last_ts, last_dt, last_pts = clouds[-1]
                    valid = last_pts[(last_pts["x"] != 0) | (last_pts["y"] != 0) | (last_pts["z"] != 0)]
                    if len(valid) > 0:
                        s = valid[0]
                        info = (
                            f"  sample point: x={s['x']}mm  y={s['y']}mm  "
                            f"z={s['z']}mm  refl={s['reflectivity']}"
                        )
                        dists = np.sqrt(valid["x"].astype(np.float64)**2 +
                                        valid["y"].astype(np.float64)**2 +
                                        valid["z"].astype(np.float64)**2) / 1000.0
                        info += f"  range: {dists.min():.2f}-{dists.max():.2f}m"

                if imus:
                    last_ts, last_s = imus[-1]
                    s = last_s[0]
                    info += (
                        f"\n  IMU: gyro=({s['gyro_x']:.4f}, {s['gyro_y']:.4f}, {s['gyro_z']:.4f}) rad/s  "
                        f"acc=({s['acc_x']:.4f}, {s['acc_y']:.4f}, {s['acc_z']:.4f}) g"
                    )

                devices = driver.devices
                dev_str = ", ".join(
                    f"{d['sn']} ({d['ip']})" for d in devices.values()
                ) or "none yet"

                print(
                    f"[{elapsed:6.1f}s]  points: {total_points:>10,} ({pts_rate:>8,.0f}/s)  "
                    f"imu: {total_imu:>8,} ({imu_rate:>6,.0f}/s)  devices: {dev_str}"
                )
                if info:
                    print(info)
                print()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        driver.stop()
        elapsed = time.monotonic() - t0
        print(f"\nTotal: {total_points:,} points, {total_imu:,} IMU samples in {elapsed:.1f}s")


if __name__ == "__main__":
    main()
