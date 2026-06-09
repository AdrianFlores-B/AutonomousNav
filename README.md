# AutonomousNav

Custom from-scratch LiDAR-Inertial-Visual SLAM system for autonomous navigation, built without ROS/ROS2 dependencies. Streams and visualizes real-time 3D point clouds and IMU data from a Livox Mid-360 solid-state LiDAR.

## Hardware

| Component | Model | Notes |
|-----------|-------|-------|
| LiDAR | Livox Mid-360 | 360° × 59° FOV, non-repetitive scan, built-in IMU |
| Camera | Orbbec Astra+ | RGB stream (integration planned) |
| Host | Ubuntu 24.04 LTS | Ethernet connection to LiDAR |

### Mid-360 Specifications

- **Point cloud**: ~200,000 points/sec, Cartesian coordinates (mm), reflectivity, 8-bit confidence tags
- **IMU**: 200 Hz, 3-axis accelerometer + 3-axis gyroscope
- **Detection range**: 0.1 m – 100 m
- **Connection**: Ethernet (UDP), subnet `192.168.1.0/24`
- **Time sync**: IEEE 1588 PTP and GPS PPS supported

## Architecture

```
Mid-360 LiDAR
    │  UDP packets (point cloud port 56300, IMU port 56400)
    ▼
Livox-SDK2 (C shared library)
    │  Parses packets, invokes registered callbacks
    ▼
nav.LidarDriver (Python, ctypes)
    │  Zero-copy-ish numpy arrays via memmove + frombuffer
    │  Thread-safe deque buffers (SDK callbacks → main thread)
    ▼
Application layer
    ├── scripts/live_viewer.py   — real-time Open3D 3D viewer
    ├── scripts/test_capture.py  — terminal stats + data validation
    └── (future: SLAM pipeline, navigation, recording)
```

**No ROS/ROS2.** The entire stack uses vendor SDKs and standard Python/C++ libraries directly. This keeps the build simple, the latency low, and the dependency tree small.

**Python-first.** The data acquisition layer, visualization, and (planned) high-level SLAM logic are all Python. C++ is reserved for real-time critical paths only (via pybind11 where needed). At 200K points/sec + 200 Hz IMU, Python with numpy handles the data rates comfortably.

## Project Structure

```
AutonomousNav/
├── nav/                          # Core Python package
│   ├── __init__.py
│   ├── data_types.py             # ctypes structs + numpy dtypes matching SDK headers
│   └── lidar_driver.py           # LidarDriver: wraps Livox-SDK2 via ctypes
├── scripts/
│   ├── live_viewer.py            # Real-time 3D point cloud viewer (Open3D)
│   └── test_capture.py           # Terminal-based data capture + stats
├── patches/
│   └── livox-sdk2-fixes.patch    # Required patches for SDK2 (GCC 13+, bind fix)
├── Livox-SDK2/                   # Git submodule → github.com/Livox-SDK/Livox-SDK2
├── mid360_config.json            # Network config (host IP, ports)
├── Livox_Mid-360_User_Manual_EN.pdf
└── Livox_Viewer_2_User_Manual_EN_v1.2.pdf
```

## Setup

### Prerequisites

- Ubuntu 24.04 (or similar Linux with GCC 13+)
- CMake 3.10+
- Python 3.10+
- Livox Mid-360 connected via Ethernet

### 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/AdrianFlores-B/AutonomousNav.git
cd AutonomousNav
```

If you already cloned without `--recurse-submodules`:

```bash
git submodule update --init
```

### 2. Patch and Build Livox-SDK2

The SDK requires patches for GCC 13+ compatibility and a socket bind fix:

```bash
cd Livox-SDK2
git apply ../patches/livox-sdk2-fixes.patch
mkdir build && cd build
cmake ..
make -j$(nproc)
cd ../..
```

This produces `Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so`.

### 3. Create Python Environment

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy open3d
```

### 4. Configure Network

Set a static IP on the Ethernet interface connected to the Mid-360:

```bash
# Replace eno1 with your interface name
sudo ip addr add 192.168.1.2/24 dev eno1
sudo ip link set eno1 up
```

If using `firewalld`, trust the interface:

```bash
sudo firewall-cmd --zone=trusted --change-interface=eno1
sudo firewall-cmd --zone=trusted --change-interface=eno1 --permanent
```

The default `mid360_config.json` expects host IP `192.168.1.2`. Edit it if your network differs.

### 5. Run

**Live 3D viewer** (requires a display):

```bash
sudo .venv/bin/python3 -u scripts/live_viewer.py
```

**Terminal stats only** (headless-friendly):

```bash
sudo .venv/bin/python3 -u scripts/test_capture.py
```

Both scripts need `sudo` because the SDK uses raw sockets for device discovery.

## Live Viewer Controls

| Input | Action |
|-------|--------|
| Mouse drag | Rotate view |
| Scroll wheel | Zoom in/out |
| Ctrl + drag | Pan |
| R | Reset camera viewpoint |
| Q / Esc | Quit |

Points are colored by height (z-axis) using the turbo colormap: blue (floor) → green (mid) → red (ceiling). The RGB axes at the origin show the LiDAR coordinate frame (X = red, Y = green, Z = blue).

## SDK Patches

The `patches/livox-sdk2-fixes.patch` file contains fixes required to build and run the SDK on modern Linux:

| Fix | File | Issue |
|-----|------|-------|
| Add `#include <cstdint>` | `define.h`, `livox_lidar_def.h`, `file_manager.h` | GCC 13 removed implicit `<cstdint>` from other headers |
| `INADDR_ANY` bind fallback | `network_util.cpp` | SDK passed `255.255.255.255` as bind address when no interface matched, causing socket bind failures |
| Epoll min-size guard | `multiple_io_epoll.cpp` | `epoll_wait` with `maxevents=0` returns an error |

## Roadmap

- [x] Native Livox-SDK2 data acquisition (point cloud + IMU)
- [x] Real-time 3D point cloud visualization
- [ ] Data recording and playback (binary format)
- [ ] Orbbec Astra+ camera integration (RGB texturing)
- [ ] IMU pre-integration and bias estimation
- [ ] Point cloud registration (ICP / feature-based)
- [ ] Tightly-coupled LiDAR-Inertial odometry
- [ ] Loop closure and global optimization
- [ ] Occupancy grid mapping
- [ ] Path planning and autonomous navigation

## License

This project is for academic research at CIC. Livox-SDK2 is subject to its own [MIT license](https://github.com/Livox-SDK/Livox-SDK2/blob/master/LICENSE.txt).
