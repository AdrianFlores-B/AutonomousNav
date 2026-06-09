"""Pure-Python driver for the Livox Mid-360 via Livox-SDK2 (ctypes).

Usage:
    driver = LidarDriver("path/to/liblivox_lidar_sdk_shared.so")
    driver.start("path/to/mid360_config.json")

    while running:
        clouds = driver.get_point_clouds()  # list of (timestamp_ns, points_ndarray)
        imus   = driver.get_imu_samples()   # list of (timestamp_ns, imu_ndarray)

    driver.stop()
"""

import ctypes
import struct
import socket
import threading
from collections import deque
from pathlib import Path

import numpy as np

from nav.data_types import (
    LivoxLidarEthernetPacket,
    LivoxLidarInfo,
    LivoxLidarAsyncControlResponse,
    PointCloudCB,
    ImuDataCB,
    InfoChangeCB,
    InfoCB,
    AsyncControlCB,
    PACKET_DATA_OFFSET,
    POINT_DTYPE_MAP,
    CARTESIAN_HIGH,
    IMU_DATA,
    WORK_MODE_NORMAL,
)


def _handle_to_ip(handle: int) -> str:
    return socket.inet_ntoa(struct.pack("<I", handle))


class LidarDriver:
    """Wraps Livox-SDK2 shared library, exposes point cloud + IMU as numpy."""

    def __init__(self, sdk_path: str | Path):
        self._sdk_path = str(sdk_path)
        self._lib = ctypes.CDLL(self._sdk_path)
        self._setup_signatures()

        self._cloud_buf: deque = deque(maxlen=4000)  # ~2 sec at ~2k pkt/s
        self._imu_buf: deque = deque(maxlen=500)     # ~2.5 sec at 200 Hz

        self._devices: dict[int, dict] = {}
        self._running = False

        # prevent garbage collection of ctypes callbacks
        self._cb_pointcloud = PointCloudCB(self._on_point_cloud)
        self._cb_imu = ImuDataCB(self._on_imu)
        self._cb_info_change = InfoChangeCB(self._on_info_change)
        self._cb_info = InfoCB(self._on_info)
        self._cb_work_mode = AsyncControlCB(self._on_work_mode_set)

    # ------------------------------------------------------------------ #
    #  public API                                                         #
    # ------------------------------------------------------------------ #

    def start(self, config_path: str | Path) -> None:
        config_path = str(Path(config_path).resolve())
        self._lib.DisableLivoxSdkConsoleLogger()
        ok = self._lib.LivoxLidarSdkInit(
            config_path.encode(),
            b"",
            None,
        )
        if not ok:
            self._lib.LivoxLidarSdkUninit()
            raise RuntimeError("LivoxLidarSdkInit failed")
        self._lib.SetLivoxLidarPointCloudCallBack(self._cb_pointcloud, None)
        self._lib.SetLivoxLidarImuDataCallback(self._cb_imu, None)
        self._lib.SetLivoxLidarInfoChangeCallback(self._cb_info_change, None)
        self._lib.SetLivoxLidarInfoCallback(self._cb_info, None)
        self._running = True

    def stop(self) -> None:
        if self._running:
            self._lib.LivoxLidarSdkUninit()
            self._running = False

    def get_point_clouds(self) -> list[tuple[int, int, np.ndarray]]:
        """Drain point cloud buffer.

        Returns list of (timestamp_ns, data_type, points_ndarray).
        Each points_ndarray uses the dtype matching data_type
        (CARTESIAN_HIGH_DTYPE / CARTESIAN_LOW_DTYPE / SPHERICAL_DTYPE).
        """
        items = []
        while self._cloud_buf:
            try:
                items.append(self._cloud_buf.popleft())
            except IndexError:
                break
        return items

    def get_imu_samples(self) -> list[tuple[int, np.ndarray]]:
        """Drain IMU buffer.

        Returns list of (timestamp_ns, imu_ndarray) where imu_ndarray
        has dtype IMU_DTYPE with fields gyro_x/y/z (rad/s) and acc_x/y/z (g).
        """
        items = []
        while self._imu_buf:
            try:
                items.append(self._imu_buf.popleft())
            except IndexError:
                break
        return items

    @property
    def devices(self) -> dict[int, dict]:
        """Discovered devices: {handle: {"sn": str, "ip": str, "dev_type": int}}."""
        return dict(self._devices)

    # ------------------------------------------------------------------ #
    #  SDK callbacks (called from SDK internal threads)                    #
    # ------------------------------------------------------------------ #

    def _on_point_cloud(self, handle, dev_type, data_ptr, client_data):
        if not data_ptr:
            return
        pkt = data_ptr.contents
        dot_num = pkt.dot_num
        data_type = pkt.data_type
        if data_type not in POINT_DTYPE_MAP or data_type == IMU_DATA:
            return
        dtype, point_size = POINT_DTYPE_MAP[data_type]
        nbytes = dot_num * point_size
        ts_ns = struct.unpack_from("<Q", bytes(pkt.timestamp))[0]
        data_addr = ctypes.addressof(pkt) + PACKET_DATA_OFFSET
        buf = (ctypes.c_uint8 * nbytes)()
        ctypes.memmove(buf, data_addr, nbytes)
        points = np.frombuffer(buf, dtype=dtype).copy()
        self._cloud_buf.append((ts_ns, data_type, points))

    def _on_imu(self, handle, dev_type, data_ptr, client_data):
        if not data_ptr:
            return
        pkt = data_ptr.contents
        dot_num = pkt.dot_num
        dtype, sample_size = POINT_DTYPE_MAP[IMU_DATA]
        nbytes = dot_num * sample_size
        ts_ns = struct.unpack_from("<Q", bytes(pkt.timestamp))[0]
        data_addr = ctypes.addressof(pkt) + PACKET_DATA_OFFSET
        buf = (ctypes.c_uint8 * nbytes)()
        ctypes.memmove(buf, data_addr, nbytes)
        samples = np.frombuffer(buf, dtype=dtype).copy()
        self._imu_buf.append((ts_ns, samples))

    def _on_info_change(self, handle, info_ptr, client_data):
        if not info_ptr:
            return
        info = info_ptr.contents
        sn = info.sn.decode("utf-8", errors="replace").rstrip("\x00")
        ip = _handle_to_ip(handle)
        self._devices[handle] = {
            "sn": sn,
            "ip": ip,
            "dev_type": info.dev_type,
        }
        print(f"[LidarDriver] Device discovered: SN={sn}  IP={ip}  type={info.dev_type}")
        self._lib.SetLivoxLidarWorkMode(
            ctypes.c_uint32(handle),
            ctypes.c_int(WORK_MODE_NORMAL),
            self._cb_work_mode,
            None,
        )

    def _on_info(self, handle, dev_type, info_str, client_data):
        pass

    def _on_work_mode_set(self, status, handle, response_ptr, client_data):
        if response_ptr and response_ptr.contents.ret_code == 0:
            ip = _handle_to_ip(handle)
            print(f"[LidarDriver] Work mode set to NORMAL for {ip}")

    # ------------------------------------------------------------------ #
    #  ctypes function signatures                                         #
    # ------------------------------------------------------------------ #

    def _setup_signatures(self):
        lib = self._lib

        lib.LivoxLidarSdkInit.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_void_p]
        lib.LivoxLidarSdkInit.restype = ctypes.c_bool

        lib.LivoxLidarSdkUninit.argtypes = []
        lib.LivoxLidarSdkUninit.restype = None

        lib.DisableLivoxSdkConsoleLogger.argtypes = []
        lib.DisableLivoxSdkConsoleLogger.restype = None

        lib.SetLivoxLidarPointCloudCallBack.argtypes = [PointCloudCB, ctypes.c_void_p]
        lib.SetLivoxLidarPointCloudCallBack.restype = None

        lib.SetLivoxLidarImuDataCallback.argtypes = [ImuDataCB, ctypes.c_void_p]
        lib.SetLivoxLidarImuDataCallback.restype = None

        lib.SetLivoxLidarInfoChangeCallback.argtypes = [InfoChangeCB, ctypes.c_void_p]
        lib.SetLivoxLidarInfoChangeCallback.restype = None

        lib.SetLivoxLidarInfoCallback.argtypes = [InfoCB, ctypes.c_void_p]
        lib.SetLivoxLidarInfoCallback.restype = None

        lib.SetLivoxLidarWorkMode.argtypes = [
            ctypes.c_uint32, ctypes.c_int, AsyncControlCB, ctypes.c_void_p,
        ]
        lib.SetLivoxLidarWorkMode.restype = ctypes.c_int32
