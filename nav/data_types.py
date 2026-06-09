"""ctypes structures and numpy dtypes matching Livox SDK2 livox_lidar_def.h.

All structs use _pack_ = 1 to match the SDK's #pragma pack(1).
"""

import ctypes
import numpy as np

# --- ctypes structures (packed, matching C header) ---

class LivoxLidarEthernetPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("version", ctypes.c_uint8),
        ("length", ctypes.c_uint16),
        ("time_interval", ctypes.c_uint16),  # unit: 0.1 us
        ("dot_num", ctypes.c_uint16),
        ("udp_cnt", ctypes.c_uint16),
        ("frame_cnt", ctypes.c_uint8),
        ("data_type", ctypes.c_uint8),
        ("time_type", ctypes.c_uint8),
        ("rsvd", ctypes.c_uint8 * 12),
        ("crc32", ctypes.c_uint32),
        ("timestamp", ctypes.c_uint8 * 8),
        ("data", ctypes.c_uint8 * 1),
    ]

PACKET_DATA_OFFSET = 36  # byte offset of data[] within the packet

class LivoxLidarInfo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("dev_type", ctypes.c_uint8),
        ("sn", ctypes.c_char * 16),
        ("lidar_ip", ctypes.c_char * 16),
    ]

class LivoxLidarAsyncControlResponse(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("ret_code", ctypes.c_uint8),
        ("error_key", ctypes.c_uint16),
    ]

# --- Data type enums ---

IMU_DATA = 0
CARTESIAN_HIGH = 1
CARTESIAN_LOW = 2
SPHERICAL = 3

# Work mode enum
WORK_MODE_NORMAL = 0x01

# --- numpy dtypes for efficient array operations ---

CARTESIAN_HIGH_DTYPE = np.dtype([
    ("x", np.int32),       # mm
    ("y", np.int32),       # mm
    ("z", np.int32),       # mm
    ("reflectivity", np.uint8),
    ("tag", np.uint8),
], align=False)  # 14 bytes per point

CARTESIAN_LOW_DTYPE = np.dtype([
    ("x", np.int16),       # cm
    ("y", np.int16),       # cm
    ("z", np.int16),       # cm
    ("reflectivity", np.uint8),
    ("tag", np.uint8),
], align=False)  # 8 bytes per point

SPHERICAL_DTYPE = np.dtype([
    ("depth", np.uint32),
    ("theta", np.uint16),
    ("phi", np.uint16),
    ("reflectivity", np.uint8),
    ("tag", np.uint8),
], align=False)  # 10 bytes per point

IMU_DTYPE = np.dtype([
    ("gyro_x", np.float32),  # rad/s
    ("gyro_y", np.float32),
    ("gyro_z", np.float32),
    ("acc_x", np.float32),   # g
    ("acc_y", np.float32),
    ("acc_z", np.float32),
], align=False)  # 24 bytes per sample

POINT_DTYPE_MAP = {
    CARTESIAN_HIGH: (CARTESIAN_HIGH_DTYPE, 14),
    CARTESIAN_LOW: (CARTESIAN_LOW_DTYPE, 8),
    SPHERICAL: (SPHERICAL_DTYPE, 10),
    IMU_DATA: (IMU_DTYPE, 24),
}

# --- ctypes callback signatures ---

PointCloudCB = ctypes.CFUNCTYPE(
    None, ctypes.c_uint32, ctypes.c_uint8,
    ctypes.POINTER(LivoxLidarEthernetPacket), ctypes.c_void_p,
)

ImuDataCB = ctypes.CFUNCTYPE(
    None, ctypes.c_uint32, ctypes.c_uint8,
    ctypes.POINTER(LivoxLidarEthernetPacket), ctypes.c_void_p,
)

InfoChangeCB = ctypes.CFUNCTYPE(
    None, ctypes.c_uint32,
    ctypes.POINTER(LivoxLidarInfo), ctypes.c_void_p,
)

InfoCB = ctypes.CFUNCTYPE(
    None, ctypes.c_uint32, ctypes.c_uint8,
    ctypes.c_char_p, ctypes.c_void_p,
)

AsyncControlCB = ctypes.CFUNCTYPE(
    None, ctypes.c_int32, ctypes.c_uint32,
    ctypes.POINTER(LivoxLidarAsyncControlResponse), ctypes.c_void_p,
)
