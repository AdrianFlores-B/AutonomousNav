# AutonomousNav — Complete Project Guide

> **Audience**: You (Adrian) and your boss. This document explains every layer of the
> system we've built so far — from the physical laser hitting the wall, all the way
> to the colored 3D point cloud spinning on your screen.

---

## Table of Contents

1. [What Are We Building?](#1-what-are-we-building)
2. [The Hardware — Livox Mid-360](#2-the-hardware--livox-mid-360)
3. [What Is an SDK and Why Do We Need One?](#3-what-is-an-sdk-and-why-do-we-need-one)
4. [The Bridge — How Python Talks to a C Library](#4-the-bridge--how-python-talks-to-a-c-library)
5. [Our Code — File by File](#5-our-code--file-by-file)
6. [The Complete Data Pipeline](#6-the-complete-data-pipeline)
7. [The Live 3D Viewer](#7-the-live-3d-viewer)
8. [What We Fixed Along the Way](#8-what-we-fixed-along-the-way)
9. [Glossary](#9-glossary)

---

## 1. What Are We Building?

### The Goal

We are building a **self-driving navigation system** for a mobile cart that will move
autonomously through the CIC building. To do this, the cart needs to:

1. **See** the world around it (walls, doors, obstacles, furniture)
2. **Build a map** of the building in 3D
3. **Know where it is** on that map at all times
4. **Plan paths** to navigate from point A to point B without hitting anything

This is called **SLAM** — Simultaneous Localization and Mapping. The cart builds the
map and figures out its position *at the same time*, in real-time, as it moves.

### Where We Are Now

We're at step 1: **making the cart see**. Specifically, we've built the software that:

- Connects to the LiDAR sensor
- Reads 200,000 3D points per second from it
- Reads 200 motion measurements per second from its built-in IMU
- Displays all of it live in a 3D viewer you can rotate and zoom

```
┌─────────────────────────────────────────────────────────┐
│                    FULL ROADMAP                          │
│                                                         │
│   ██████████  Step 1: See the world        ← WE ARE    │
│   ░░░░░░░░░░  Step 2: Record & replay        HERE      │
│   ░░░░░░░░░░  Step 3: Add camera (RGB)                  │
│   ░░░░░░░░░░  Step 4: Build 3D map (SLAM)               │
│   ░░░░░░░░░░  Step 5: Navigate autonomously             │
│                                                         │
│   ██ = done   ░░ = planned                              │
└─────────────────────────────────────────────────────────┘
```

### Why No ROS?

Most robotics projects use **ROS** (Robot Operating System), a framework that provides
tools for sensor communication, data processing, visualization, etc. We deliberately
chose **not** to use it because:

- **Tight coupling**: We want every component to work together seamlessly, not through
  ROS's message-passing system which adds latency and complexity
- **Simplicity**: No need for `catkin`, `colcon`, launch files, or the entire ROS
  ecosystem just to read a sensor
- **Understanding**: By building from scratch, we understand every byte flowing through
  the system — critical for debugging a real-time SLAM pipeline
- **Deployment**: No ROS installation needed on the final cart — just our code

Instead, we talk directly to the sensor hardware using the manufacturer's SDK.

---

## 2. The Hardware — Livox Mid-360

### What Is LiDAR?

LiDAR stands for **Li**ght **D**etection **A**nd **R**anging. It works like this:

```
    How ONE laser measurement works:
    ================================

    1. Laser fires         2. Light hits wall       3. Light bounces back
                                │
    ┌──────┐  ─ ─ ─ ─ ─ ─ ─ ─▶│                    ┌──────┐
    │LiDAR │    laser pulse     │ wall               │LiDAR │◀─ ─ ─ ─ ─ ─
    │sensor │                   │                    │sensor │  reflected
    └──────┘                    │                    └──────┘   light

    4. Sensor measures the TIME it took:

       distance = (speed of light × time) ÷ 2

       Light travels at 300,000 km/s, so if the round-trip took 20 nanoseconds:
       distance = (300,000 km/s × 20 ns) ÷ 2 = 3.0 meters
```

The sensor does this **hundreds of thousands of times per second**, pointing the laser
in different directions each time. Each measurement becomes a **3D point** — an
(x, y, z) coordinate in space.

### The Mid-360 Specifically

The Livox Mid-360 is a **solid-state** LiDAR. Unlike traditional spinning LiDARs
(like the ones on Waymo cars that physically rotate 360°), the Mid-360 uses an
**internal rotating prism** that bends the laser beam in a flower-like pattern:

```
    Traditional spinning LiDAR         Livox Mid-360 (non-repetitive)
    ─────────────────────────         ────────────────────────────────

    Same lines every rotation:         Fills in gaps over time:

    ────────────────────────           ╲  ╱ ──── ╲  ╱
    ────────────────────────            ╳      ──  ╳
    ────────────────────────           ╱  ╲ ──── ╱  ╲
    ────────────────────────              ── ╲╱──── ╲╱
    ────────────────────────           ╲╱ ── ╱╲ ────╱╲

    After 1 second: same lines         After 1 second: dense coverage
    After 10 seconds: same lines       After 10 seconds: very dense
```

This "non-repetitive" pattern is why our viewer accumulates 1.5 seconds of data —
the longer we wait, the denser and more complete the 3D picture becomes.

### What Data Comes Out

The Mid-360 produces two streams of data:

```
    ┌────────────────────────────────────────────────────────┐
    │               LIVOX MID-360 OUTPUTS                     │
    │                                                         │
    │  STREAM 1: Point Cloud                                  │
    │  ~~~~~~~~~~~~~~~~~~~~~~                                 │
    │  200,000 points per second                              │
    │  Each point = { x, y, z, reflectivity, tag }            │
    │                                                         │
    │     x, y, z       → position in millimeters             │
    │     reflectivity  → how shiny the surface is (0-255)    │
    │     tag           → confidence level of the measurement │
    │                                                         │
    │  Arrives in PACKETS of 96 points each                   │
    │  That's ~2,083 packets per second                       │
    │                                                         │
    │  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─  │
    │                                                         │
    │  STREAM 2: IMU (Inertial Measurement Unit)              │
    │  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~              │
    │  200 samples per second                                 │
    │  Each sample = { gyro_x/y/z, acc_x/y/z }               │
    │                                                         │
    │     gyro_x/y/z  → rotation speed (rad/s)                │
    │     acc_x/y/z   → acceleration including gravity (g)    │
    │                                                         │
    │  The IMU chip sits INSIDE the LiDAR housing             │
    │  It measures how the LiDAR is moving and tilting        │
    │  Critical for SLAM: tells us cart motion between scans  │
    └────────────────────────────────────────────────────────┘
```

### How It Connects

The Mid-360 connects to the computer via a **single cable** that splits into three:

```
    ┌─────────┐      M12 aviation       ┌─────────────────────┐
    │ Mid-360 │──────connector──────────▶│ 1-to-3 splitter     │
    │  LiDAR  │    (single cable)        │                     │
    └─────────┘                          │  ├── Ethernet ──────▶ Computer (192.168.1.2)
                                         │  ├── Power (12V) ◀── Power supply
                                         │  └── (spare)        │
                                         └─────────────────────┘

    Network setup:
    ┌──────────────────────────────────────────────────────┐
    │  LiDAR IP:    192.168.1.109  (set by serial number)  │
    │  Computer IP: 192.168.1.2    (static, set by us)     │
    │  Subnet:      255.255.255.0  (same local network)    │
    │  Protocol:    UDP (fast, no handshake overhead)       │
    └──────────────────────────────────────────────────────┘
```

The LiDAR and computer talk using **UDP** (User Datagram Protocol) — it's like
sending postcards instead of making phone calls. Fast, no waiting for acknowledgment,
but packets could theoretically get lost. For 200K points/sec on a direct cable, loss
is essentially zero.

---

## 3. What Is an SDK and Why Do We Need One?

### The Problem

The Mid-360 sends raw **UDP packets** over Ethernet. These are just blobs of bytes —
ones and zeros packed in a specific format. We could write code to listen on the
network ports and decode these bytes ourselves, but:

1. The packet format is complex and undocumented publicly
2. There's a handshake protocol to discover and activate the LiDAR
3. There's timestamp synchronization logic
4. We'd need to handle packet ordering, device states, error recovery, etc.

This would take weeks to reverse-engineer and get right.

### The Solution: SDK

**SDK** stands for **Software Development Kit**. Think of it as a **translator** that
Livox (the manufacturer) provides:

```
    WITHOUT SDK (we'd have to do this ourselves):
    ═══════════════════════════════════════════════

    ┌─────────┐   raw UDP bytes    ┌──────────────────────────────┐
    │ Mid-360 │ ──────────────────▶│ Our code would need to:      │
    │         │ ◀──────────────────│   • Listen on 5 UDP ports    │
    └─────────┘   raw UDP bytes    │   • Parse binary headers     │
                                   │   • Handle device discovery  │
                                   │   • Manage state machine     │
                                   │   • Decode point coordinates │
                                   │   • Handle timestamps        │
                                   │   • Error recovery           │
                                   │   • ... weeks of work        │
                                   └──────────────────────────────┘

    WITH SDK (what we actually do):
    ════════════════════════════════

    ┌─────────┐  raw UDP   ┌───────────┐  clean data   ┌──────────┐
    │ Mid-360 │ ─────────▶ │ Livox     │ ────────────▶ │ Our code │
    │         │ ◀───────── │ SDK2      │               │          │
    └─────────┘  raw UDP   │ (C library│               │ "Hey SDK,│
                           │  .so file)│ ◀──────────── │  give me │
                           └───────────┘  simple calls │  points" │
                                                       └──────────┘
```

### What the SDK Actually Is

The SDK is a **compiled C library** — a `.so` file (shared object) on Linux:

```
    Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                          ↑
            This single file IS the SDK

    It's compiled from C++ source code (the Livox-SDK2/ folder).
    Once compiled, it's a ~2MB binary file containing machine code
    that knows how to talk to Livox LiDARs.
```

The SDK provides **functions** we can call:

```
    Functions the SDK gives us:                What they do:
    ───────────────────────────                ─────────────

    LivoxLidarSdkInit(config)             →   "Start up, here's the network config"
    SetLivoxLidarPointCloudCallBack(fn)   →   "Call THIS function when points arrive"
    SetLivoxLidarImuDataCallback(fn)      →   "Call THIS function when IMU data arrives"
    SetLivoxLidarInfoChangeCallback(fn)   →   "Call THIS function when a device appears"
    SetLivoxLidarWorkMode(handle, mode)   →   "Tell LiDAR #X to start scanning"
    LivoxLidarSdkUninit()                 →   "Shut everything down"
    DisableLivoxSdkConsoleLogger()        →   "Stop printing debug messages"
```

### How the SDK Works Internally

When we call `LivoxLidarSdkInit()`, the SDK spins up **background threads** that:

1. Open UDP sockets on the configured ports
2. Broadcast a discovery message on the network
3. Wait for the LiDAR to respond with its serial number and status
4. Start receiving data packets on the point cloud and IMU ports
5. Parse each packet and call **our callback functions** with the decoded data

```
    Timeline of what happens:
    ═════════════════════════

    Time 0.0s   LivoxLidarSdkInit() called
                SDK creates internal threads
                SDK opens UDP sockets on ports 56101-56501
                SDK sends discovery broadcast

    Time ~0.5s  Mid-360 responds: "I'm here, SN=47MDM6E0020509"
                SDK calls our InfoChange callback
                We call SetLivoxLidarWorkMode(NORMAL)
                SDK tells LiDAR: "start scanning"

    Time ~0.6s  LiDAR begins sending point cloud packets
                SDK receives first UDP packet (96 points)
                SDK calls our PointCloud callback with the data
                SDK receives IMU packet
                SDK calls our IMU callback

    Time ~0.6s+ This repeats ~2,083 times/sec for points
                and 200 times/sec for IMU
                UNTIL we call LivoxLidarSdkUninit()
```

### The Callback Pattern

This is the most important concept to understand. The SDK uses **callbacks** — we
give the SDK a function, and the SDK calls that function whenever new data arrives.
It's like saying "call me back when the pizza is ready":

```
    ANALOGY: Ordering Pizza
    ═══════════════════════

    1. You call the pizza shop:          →  LivoxLidarSdkInit()
    2. "Call me at 555-1234 when ready"  →  SetLivoxLidarPointCloudCallBack(my_func)
    3. You go do other things            →  Main loop runs (viewer, etc.)
    4. Phone rings: "Pizza is ready!"    →  SDK calls my_func(data) from its thread
    5. You receive the pizza             →  my_func copies the data to a buffer
    6. Repeat 2,083 times per second     →  (ok, that's a lot of pizza)
```

In code, it looks like this:

```python
    # Step 2: We define what to do when points arrive
    def on_points_received(handle, dev_type, data_ptr, client_data):
        # This function gets called by the SDK, ~2083 times per second
        # 'data_ptr' points to a packet with 96 points
        # We copy the points to our buffer
        points = read_points_from(data_ptr)
        buffer.append(points)

    # Step 2b: We register our function with the SDK
    sdk.SetLivoxLidarPointCloudCallBack(on_points_received)

    # Step 3: Meanwhile, in our main loop...
    while True:
        # We drain the buffer and display the points
        new_points = buffer.drain()
        show_in_3d_viewer(new_points)
```

### Threading

This is happening on **multiple threads simultaneously**:

```
    ┌─────────────────────────────────────────────────────────────┐
    │                    PROCESS: python3 live_viewer.py           │
    │                                                             │
    │  ┌─────────────────────┐   ┌─────────────────────────────┐  │
    │  │  MAIN THREAD        │   │  SDK THREAD 1 (network)     │  │
    │  │  (our Python code)  │   │  (created by SDK internally)│  │
    │  │                     │   │                             │  │
    │  │  while True:        │   │  while True:               │  │
    │  │    drain buffer ◀───┼───┼── buffer.append(points)     │  │
    │  │    update viewer    │   │    (calls our callback)     │  │
    │  │    poll events      │   │                             │  │
    │  │    render frame     │   │                             │  │
    │  │    sleep 33ms       │   │                             │  │
    │  └─────────────────────┘   └─────────────────────────────┘  │
    │                                                             │
    │  ┌─────────────────────────────┐                            │
    │  │  SDK THREAD 2 (IMU)         │                            │
    │  │  (created by SDK internally)│                            │
    │  │                             │                            │
    │  │  Receives IMU packets       │                            │
    │  │  Calls our IMU callback ────┼──▶ imu_buffer.append()     │
    │  └─────────────────────────────┘                            │
    └─────────────────────────────────────────────────────────────┘

    The DEQUE (buffer) is thread-safe in Python:
    - SDK threads APPEND to one end
    - Main thread POPS from the other end
    - No data corruption, no locks needed
```

---

## 4. The Bridge — How Python Talks to a C Library

### The Challenge

The SDK is written in **C/C++** and compiled to machine code (`.so` file). Our
application is written in **Python**. These are very different worlds:

```
    C world                              Python world
    ═══════                              ════════════
    • Compiled to machine code           • Interpreted line by line
    • Manual memory management           • Automatic garbage collection
    • Types: int32_t, uint8_t, float     • Types: int, float (flexible)
    • Structs with exact byte layout     • Objects with dynamic attributes
    • Runs at maximum CPU speed          • ~100x slower for raw computation
    • Pointers (raw memory addresses)    • References (managed by runtime)
```

### ctypes: The Bridge

Python includes a built-in module called **ctypes** that lets Python call functions
in compiled C libraries. It's like having a **bilingual translator** at the border:

```
    ┌──────────────┐    ctypes     ┌──────────────────┐
    │   Python     │  (translator) │   C library      │
    │              │               │   (.so file)     │
    │  "Call       │──────────────▶│                  │
    │   Init with  │  Converts     │  LivoxLidarSdk   │
    │   this       │  Python str   │  Init(           │
    │   config     │  to C char*   │    "/path/to/    │
    │   path"      │               │     config.json" │
    │              │               │  )               │
    │              │◀──────────────│                  │
    │  Got True    │  Converts     │  returns true    │
    │              │  C bool to    │                  │
    │              │  Python bool  │                  │
    └──────────────┘               └──────────────────┘
```

Here's what ctypes does concretely in our code:

```python
    # Load the compiled C library into Python's memory space
    lib = ctypes.CDLL("path/to/liblivox_lidar_sdk_shared.so")

    # Tell ctypes: "LivoxLidarSdkInit takes a string, a string, and a pointer,
    #               and returns a boolean"
    lib.LivoxLidarSdkInit.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_void_p]
    lib.LivoxLidarSdkInit.restype = ctypes.c_bool

    # Now we can call the C function from Python!
    ok = lib.LivoxLidarSdkInit(b"/path/to/config.json", b"", None)
    # Python converts these to C types automatically thanks to argtypes
```

### Data Types Must Match Exactly

The LiDAR sends points as tightly packed bytes. Each point is exactly **14 bytes**:

```
    One point in memory (14 bytes total):
    ══════════════════════════════════════

    Byte:  0  1  2  3  4  5  6  7  8  9  10 11 12 13
           ├────────┤  ├────────┤  ├────────┤  ├┤ ├┤
              x           y           z       refl tag
           (int32)     (int32)     (int32)  (u8)(u8)
           4 bytes     4 bytes     4 bytes   1   1

    x, y, z are in MILLIMETERS as signed 32-bit integers
    reflectivity is 0-255 (how shiny)
    tag is confidence (how reliable this measurement is)
```

In C, this is defined with `#pragma pack(1)` (no padding between fields). We must
match this **exactly** in Python, or the data will be garbage:

```python
    # In nav/data_types.py — this MUST match the C struct byte-for-byte
    CARTESIAN_HIGH_DTYPE = np.dtype([
        ("x", np.int32),          # 4 bytes, matches C int32_t
        ("y", np.int32),          # 4 bytes
        ("z", np.int32),          # 4 bytes
        ("reflectivity", np.uint8),  # 1 byte, matches C uint8_t
        ("tag", np.uint8),           # 1 byte
    ], align=False)               # align=False = packed, no padding
    # Total: 14 bytes per point — MUST equal C's sizeof(LivoxLidarCartHighRawPoint)
```

### Why numpy?

Each packet has 96 points × 14 bytes = 1,344 bytes. We get ~2,083 packets per second.
That's ~2.8 million bytes per second of raw point data.

**numpy** lets us treat this raw memory as a structured array — like a spreadsheet
where each row is a point and columns are x, y, z, reflectivity, tag. No loops, no
per-point Python overhead:

```python
    # WITHOUT numpy (slow — Python loop over 96 points):
    points = []
    for i in range(96):
        x = struct.unpack_from("<i", data, i * 14)      # ~1000 ns per call
        y = struct.unpack_from("<i", data, i * 14 + 4)
        z = struct.unpack_from("<i", data, i * 14 + 8)
        points.append((x, y, z))
    # ~100 microseconds per packet × 2083 packets/sec = 208 ms/sec of CPU time

    # WITH numpy (fast — zero-loop, reads directly from memory):
    points = np.frombuffer(raw_bytes, dtype=CARTESIAN_HIGH_DTYPE)
    # ~2 microseconds per packet × 2083 packets/sec = 4 ms/sec of CPU time
    # 50x faster!
```

---

## 5. Our Code — File by File

### Project Map

```
    AutonomousNav/
    │
    ├── nav/                        ◀── OUR CORE PYTHON PACKAGE
    │   ├── __init__.py                  (the "engine room")
    │   ├── data_types.py
    │   └── lidar_driver.py
    │
    ├── scripts/                    ◀── RUNNABLE APPLICATIONS
    │   ├── live_viewer.py               (what the user actually runs)
    │   └── test_capture.py
    │
    ├── Livox-SDK2/                 ◀── THE SDK (git submodule)
    │   ├── (source code)                Compiles to the .so library
    │   └── build/sdk_core/
    │       └── liblivox_lidar_sdk_shared.so  ◀── THE COMPILED SDK
    │
    ├── patches/                    ◀── FIXES WE MADE TO THE SDK
    │   └── livox-sdk2-fixes.patch
    │
    └── mid360_config.json          ◀── NETWORK CONFIGURATION
```

### File: `nav/data_types.py` — The Dictionary

**Purpose**: Defines the exact binary layout of every data structure the SDK uses.
Think of it as a **dictionary** that tells Python how to read the C library's data.

```
    This file answers: "When the SDK gives me a blob of bytes,
    what does each byte MEAN?"

    ┌─────────────────────────────────────────────────────────┐
    │  data_types.py contains:                                │
    │                                                         │
    │  STRUCTURES (ctypes)         DTYPES (numpy)             │
    │  ══════════════════         ════════════════             │
    │  LivoxLidarEthernetPacket   CARTESIAN_HIGH_DTYPE        │
    │  LivoxLidarInfo             CARTESIAN_LOW_DTYPE         │
    │  LivoxLidarAsyncControl...  SPHERICAL_DTYPE             │
    │                             IMU_DTYPE                   │
    │                                                         │
    │  ↑ These describe the       ↑ These describe the        │
    │    PACKET HEADER              POINT DATA inside         │
    │    (metadata: how many        the packet                │
    │     points, what type,                                  │
    │     timestamp, etc.)                                    │
    │                                                         │
    │  CALLBACKS (function signatures)                        │
    │  ═══════════════════════════════                        │
    │  PointCloudCB, ImuDataCB, InfoChangeCB, ...             │
    │                                                         │
    │  ↑ These tell ctypes the exact signature of each        │
    │    callback function (what parameters it receives)      │
    └─────────────────────────────────────────────────────────┘
```

The packet structure in detail:

```
    LivoxLidarEthernetPacket — the header of every data packet
    ═══════════════════════════════════════════════════════════

    ┌───────────┬────────┬─────────────┬─────────┬─────────┬──────────┐
    │ version   │ length │time_interval│ dot_num │ udp_cnt │frame_cnt │
    │ (1 byte)  │(2 byte)│  (2 bytes)  │(2 bytes)│(2 bytes)│ (1 byte) │
    ├───────────┴────────┴─────────────┴─────────┴─────────┴──────────┤
    │ data_type │ time_type │ reserved (12 bytes) │ crc32 (4 bytes)   │
    │ (1 byte)  │ (1 byte)  │                     │                   │
    ├───────────┴───────────┴─────────────────────┴───────────────────┤
    │ timestamp (8 bytes)                                             │
    ├─────────────────────────────────────────────────────────────────┤
    │ data[] ← the actual point data starts here (byte 36)           │
    │          96 points × 14 bytes = 1,344 bytes of point data      │
    └─────────────────────────────────────────────────────────────────┘

    dot_num tells us HOW MANY points (usually 96)
    data_type tells us WHAT FORMAT (1 = Cartesian high precision)
    timestamp tells us WHEN these points were captured (nanoseconds)
    data[] is WHERE the actual x,y,z coordinates live
```

### File: `nav/lidar_driver.py` — The Engine

**Purpose**: The `LidarDriver` class wraps the SDK. It's the only file that touches
the C library. Everything else in the project just uses `LidarDriver`.

```
    What LidarDriver does:
    ═════════════════════

    ┌──────────────────────────────────────────────────────────┐
    │                      LidarDriver                         │
    │                                                          │
    │  PUBLIC API (what scripts call):                         │
    │  ─────────────────────────────                           │
    │  driver = LidarDriver("path/to/sdk.so")  ← load library │
    │  driver.start("mid360_config.json")      ← start SDK    │
    │  clouds = driver.get_point_clouds()      ← get data     │
    │  imus = driver.get_imu_samples()         ← get IMU      │
    │  devs = driver.devices                   ← see devices  │
    │  driver.stop()                           ← shut down    │
    │                                                          │
    │  INTERNAL (hidden, automatic):                           │
    │  ─────────────────────────────                           │
    │  _on_point_cloud()  ← called by SDK when points arrive  │
    │  _on_imu()          ← called by SDK when IMU arrives    │
    │  _on_info_change()  ← called when LiDAR discovered      │
    │  _setup_signatures()← tells ctypes the function types   │
    │                                                          │
    │  BUFFERS:                                                │
    │  ────────                                                │
    │  cloud_buf (deque, max 4000 items)  ← ~2 sec of points  │
    │  imu_buf   (deque, max 500 items)   ← ~2.5 sec of IMU   │
    └──────────────────────────────────────────────────────────┘
```

The key flow inside `_on_point_cloud` (called ~2,083 times/sec by the SDK):

```
    SDK calls _on_point_cloud(handle, dev_type, data_ptr, client_data)
    │
    ├── 1. Read the packet header via data_ptr.contents
    │       → get dot_num (96), data_type (1 = Cartesian high), timestamp
    │
    ├── 2. Calculate data size: 96 points × 14 bytes = 1,344 bytes
    │
    ├── 3. Copy the raw bytes out of SDK's memory
    │       ctypes.memmove(our_buffer, sdk_address, 1344)
    │       ↑ This is critical! The SDK reuses its buffer — if we don't
    │         copy NOW, the data will be overwritten by the next packet
    │
    ├── 4. Interpret bytes as numpy structured array
    │       np.frombuffer(our_buffer, dtype=CARTESIAN_HIGH_DTYPE)
    │       → Now we have pts["x"], pts["y"], pts["z"], etc.
    │
    └── 5. Append to deque: (timestamp, data_type, points_array)
            → Main thread will pick this up later
```

### File: `nav/__init__.py` — The Front Door

Just one line — makes `from nav import LidarDriver` work:

```python
    from nav.lidar_driver import LidarDriver
```

### File: `scripts/test_capture.py` — The Terminal Monitor

**Purpose**: Prints stats to the terminal without any graphics. Useful for verifying
the sensor works, measuring data rates, checking if points are valid.

```
    Output looks like:
    ══════════════════

    SDK initialized, waiting for LiDAR data...
    [LidarDriver] Device discovered: SN=47MDM6E0020509  IP=192.168.1.109
    [LidarDriver] Work mode set to NORMAL for 192.168.1.109

    [  2.0s]  points:    400,128 (200,064/s)  imu:      400 (200/s)
      sample point: x=1234mm  y=-567mm  z=890mm  refl=45  range: 0.28-11.70m
      IMU: gyro=(0.0012, -0.0008, 0.0003) rad/s  acc=(0.01, -0.02, 0.99) g

    [  4.0s]  points:    800,256 (200,064/s)  ...
```

The IMU `acc_z ≈ 0.99g` confirms gravity is pointing straight down through the
z-axis — the LiDAR is sitting flat and level.

### File: `scripts/live_viewer.py` — The 3D Viewer

This is the main visualization tool. Explained in detail in [Section 7](#7-the-live-3d-viewer).

### File: `mid360_config.json` — Network Configuration

Tells the SDK what IP addresses and ports to use:

```json
    {
      "MID360": {
        "lidar_net_info": {          ← ports the LIDAR sends FROM
          "cmd_data_port"  : 56100,
          "push_msg_port"  : 56200,
          "point_data_port": 56300,  ← point cloud data
          "imu_data_port"  : 56400,  ← IMU data
          "log_data_port"  : 56500
        },
        "host_net_info": [{          ← our COMPUTER's settings
          "host_ip"        : "192.168.1.2",  ← our static IP
          "cmd_data_port"  : 56101,
          "push_msg_port"  : 56201,
          "point_data_port": 56301,  ← we listen here for points
          "imu_data_port"  : 56401,  ← we listen here for IMU
          "log_data_port"  : 56501
        }]
      }
    }
```

### File: `patches/livox-sdk2-fixes.patch`

The SDK source code had bugs that prevented it from compiling and running on our
system. This patch file contains our fixes (see [Section 8](#8-what-we-fixed-along-the-way)).

---

## 6. The Complete Data Pipeline

Here's **everything** that happens from laser to screen, in order:

```
    ┌─────────────────────────────────────────────────────────────────┐
    │ STEP 1: PHYSICAL                                                │
    │                                                                 │
    │   Mid-360 fires laser → light bounces off wall → sensor detects │
    │   return time → computes distance → converts to x,y,z (mm)     │
    │   This happens 200,000 times per second                         │
    └───────────────────────────────┬─────────────────────────────────┘
                                    │
                                    ▼
    ┌─────────────────────────────────────────────────────────────────┐
    │ STEP 2: NETWORK                                                 │
    │                                                                 │
    │   LiDAR batches 96 points into one UDP packet (1,380 bytes)     │
    │   Sends to 192.168.1.2:56301 at ~2,083 packets/sec              │
    │   Also sends IMU to port 56401 at 200 packets/sec               │
    └───────────────────────────────┬─────────────────────────────────┘
                                    │
                                    ▼
    ┌─────────────────────────────────────────────────────────────────┐
    │ STEP 3: SDK RECEIVES                                            │
    │                                                                 │
    │   SDK's internal thread receives UDP packet via epoll            │
    │   Validates CRC checksum                                        │
    │   Parses the 36-byte header (dot_num, data_type, timestamp)     │
    │   Calls our registered callback function with a pointer to data │
    └───────────────────────────────┬─────────────────────────────────┘
                                    │
                                    ▼
    ┌─────────────────────────────────────────────────────────────────┐
    │ STEP 4: OUR CALLBACK (lidar_driver.py → _on_point_cloud)       │
    │                                                                 │
    │   Runs on SDK thread, Python GIL acquired automatically (~1μs)  │
    │   Reads dot_num (96), data_type (1), timestamp from header      │
    │   memmove: copies 96 × 14 = 1,344 bytes from SDK → our buffer  │
    │   frombuffer: interprets bytes as numpy array with 96 rows      │
    │   Appends (timestamp, data_type, array) to thread-safe deque    │
    │   Returns — SDK can reuse its buffer now                        │
    └───────────────────────────────┬─────────────────────────────────┘
                                    │
                                    ▼
    ┌─────────────────────────────────────────────────────────────────┐
    │ STEP 5: MAIN LOOP DRAINS BUFFER (live_viewer.py)               │
    │                                                                 │
    │   Every ~33ms (30 fps), main loop calls driver.get_point_clouds │
    │   Drains all items from the deque (thread-safe popleft)         │
    │   Typically gets ~69 packets = ~6,624 points per frame          │
    └───────────────────────────────┬─────────────────────────────────┘
                                    │
                                    ▼
    ┌─────────────────────────────────────────────────────────────────┐
    │ STEP 6: DATA PROCESSING                                        │
    │                                                                 │
    │   Filter out zero-points (invalid measurements where x=y=z=0)  │
    │   Convert mm → meters: divide x, y, z by 1000                  │
    │   Stack into Nx3 float64 array: [[x₁,y₁,z₁], [x₂,y₂,z₂]...] │
    │   Add to ring buffer with wall-clock timestamp                  │
    │   Evict entries older than 1.5 seconds from ring buffer         │
    └───────────────────────────────┬─────────────────────────────────┘
                                    │
                                    ▼
    ┌─────────────────────────────────────────────────────────────────┐
    │ STEP 7: COLORIZATION                                           │
    │                                                                 │
    │   Concatenate all ring buffer entries → ~300,000 points         │
    │   Take each point's z (height) value                            │
    │   Normalize: z_norm = (z - z_min) / (z_max - z_min)            │
    │   Map to turbo colormap:                                        │
    │       0.0 (lowest)  → dark blue  (floor)                        │
    │       0.25          → cyan/teal                                 │
    │       0.50 (middle) → green      (walls, furniture)             │
    │       0.75          → yellow/orange                             │
    │       1.0 (highest) → dark red   (ceiling)                      │
    └───────────────────────────────┬─────────────────────────────────┘
                                    │
                                    ▼
    ┌─────────────────────────────────────────────────────────────────┐
    │ STEP 8: RENDERING                                               │
    │                                                                 │
    │   Upload points + colors to Open3D PointCloud object            │
    │   Open3D sends them to the GPU                                  │
    │   GPU renders each point as a tiny colored square               │
    │   Result: a 3D scene you can rotate, zoom, and pan              │
    └─────────────────────────────────────────────────────────────────┘
```

### Data Rates Summary

```
    Component             Rate              Volume
    ─────────────────     ─────────────     ──────────────
    Laser measurements    200,000 / sec     -
    Point packets (UDP)     2,083 / sec     2.8 MB/sec
    IMU packets (UDP)         200 / sec     4.8 KB/sec
    Callback invocations    2,283 / sec     -
    Viewer frame updates       30 / sec     -
    Points on screen      ~300,000          (1.5 sec window)
```

---

## 7. The Live 3D Viewer

### How `live_viewer.py` Works

The viewer is a **single main loop** that does three things each frame: read data,
process it, and render it.

```
    Frame loop (~30 fps):
    ═════════════════════

    ┌──────────────────────────────────────────────────────────┐
    │                                                          │
    │  ┌──────────────┐    ┌──────────────┐    ┌────────────┐ │
    │  │ 1. READ      │───▶│ 2. PROCESS   │───▶│ 3. RENDER  │ │
    │  │              │    │              │    │            │ │
    │  │ Drain deque  │    │ Filter zeros │    │ Upload to  │ │
    │  │ Get ~69      │    │ mm → meters  │    │ Open3D     │ │
    │  │ packets      │    │ Colorize     │    │ poll_events│ │
    │  │ (~6600 pts)  │    │ Ring buffer  │    │ update_    │ │
    │  │              │    │ management   │    │ renderer   │ │
    │  └──────────────┘    └──────────────┘    └────────────┘ │
    │                                                          │
    │  Total time per frame: 5-15 ms                           │
    │  Budget per frame at 30fps: 33 ms                        │
    │  Headroom: 18-28 ms (comfortable)                        │
    │                                                          │
    └──────────────────────────────────────────────────────────┘
```

### The Ring Buffer

The ring buffer keeps a **sliding window** of recent points:

```
    Time ──────────────────────────────────────────────▶

    At time T = 5.0s, accumulate_sec = 1.5:

    EXPIRED (discarded)         VISIBLE (displayed)
    ─────────────────          ─────────────────────
    ├── 0.0s ─── 3.5s ───├── 3.5s ──── 4.0s ──── 4.5s ──── 5.0s ──┤
                           ▲                                         ▲
                           cutoff = 5.0 - 1.5 = 3.5                 now


    Each entry in the ring buffer:
    (wall_time, xyz_array, color_array)
       │          │           │
       │          │           └── Nx3 RGB colors (one per point)
       │          └── Nx3 float64 (x,y,z in meters)
       └── when this batch arrived (for expiry)
```

### Why 1.5 Seconds?

```
    0.1 sec: ~20,000 points   → sparse, can see individual dots
    0.5 sec: ~100,000 points  → shapes recognizable
    1.5 sec: ~300,000 points  → good density, objects clearly visible  ← chosen
    5.0 sec: ~1,000,000 pts   → very dense but uses more memory/CPU
```

At 1.5 seconds, the non-repetitive scan pattern has filled in enough gaps to give a
clear picture of the environment, while still being responsive (old data leaves in
1.5 seconds as you move the cart).

### What You See in the Window

```
    ┌─────────────────────────────────────────────────────────┐
    │           Mid-360 Live (Open3D window)                   │
    │                                                         │
    │                   ·  ·   · ·                            │
    │              · · ·  ·   ·  · · ·    ← ceiling (red)    │
    │           ·····························                  │
    │          ·                           ·                  │
    │         ·   ····  ·····   ····  ···  ·  ← walls (green │
    │         ·   ·  ·  ·   ·  ·  ·  · ·  ·     /yellow)    │
    │         ·   ····  ·····   ····  ···  ·                  │
    │          ·                           ·                  │
    │           ···························                    │
    │          ··   ···  ····  ···   ···  ··  ← furniture     │
    │         ···························                      │
    │        ·································  ← floor (blue)│
    │                                                         │
    │   R,G,B                                                 │
    │   axes → ·  ← this is the LiDAR position (origin)      │
    │                                                         │
    └─────────────────────────────────────────────────────────┘
```

The coordinate axes at the origin tell you where the LiDAR is and which direction
each axis points:
- **Red axis (X)**: forward
- **Green axis (Y)**: left
- **Blue axis (Z)**: up

---

## 8. What We Fixed Along the Way

### SDK Patches (in `patches/livox-sdk2-fixes.patch`)

The Livox SDK2 source code had several issues that prevented it from working on our
Ubuntu 24.04 system with GCC 13:

```
    FIX 1: Missing #include <cstdint>
    ──────────────────────────────────
    Problem: GCC 13 removed implicit inclusion of <cstdint> from other
             standard headers. Types like uint8_t, int32_t were undefined.
    Where:   define.h, livox_lidar_def.h, file_manager.h
    Fix:     Added #include <cstdint> to each file

    FIX 2: INADDR_ANY bind fallback
    ────────────────────────────────
    Problem: When no network interface matched, SDK tried to bind a socket
             to "255.255.255.255" — this is the broadcast address and
             cannot be used as a bind address. Socket creation failed silently.
    Where:   network_util.cpp
    Fix:     If netif is empty or "255.255.255.255", bind to INADDR_ANY (0.0.0.0)
             which means "listen on all interfaces"

    FIX 3: Epoll maxevents guard
    ─────────────────────────────
    Problem: epoll_wait() called with maxevents=0 when descriptor list was
             empty, which is an error per the Linux API.
    Where:   multiple_io_epoll.cpp
    Fix:     Use max(nfds, 1) so epoll_wait always gets a valid count
```

---

## 9. Glossary

| Term | Meaning |
|------|---------|
| **LiDAR** | Light Detection And Ranging — measures distance by timing laser reflections |
| **IMU** | Inertial Measurement Unit — measures rotation (gyroscope) and acceleration |
| **SLAM** | Simultaneous Localization And Mapping — building a map while tracking position |
| **SDK** | Software Development Kit — manufacturer's library for talking to their hardware |
| **ctypes** | Python built-in module for calling compiled C/C++ library functions |
| **numpy** | Python library for fast numerical array operations |
| **Open3D** | Python library for 3D visualization and point cloud processing |
| **UDP** | User Datagram Protocol — fast, connectionless network communication |
| **Callback** | A function you give to another system, which calls it when events happen |
| **Deque** | Double-ended queue — efficient append/pop from both ends, thread-safe in CPython |
| **GIL** | Global Interpreter Lock — Python mechanism ensuring one thread runs Python at a time |
| **.so file** | Shared Object — compiled C/C++ library on Linux (like .dll on Windows) |
| **Submodule** | Git feature for including one repository inside another |
| **Point cloud** | A set of 3D points representing the surface of objects |
| **Reflectivity** | How much laser light a surface reflects back (shiny = high, dark = low) |
| **FOV** | Field of View — the angular range the sensor can see |
| **Voxel** | A 3D pixel — a small cube used to downsample point clouds |
| **Non-repetitive scan** | Scan pattern that covers different areas each rotation, filling gaps over time |
| **Ring buffer** | Fixed-size buffer that overwrites oldest data when full |
| **epoll** | Linux kernel mechanism for efficiently monitoring many network connections |
| **pybind11** | C++ library for creating Python bindings (planned for future performance-critical code) |
