#!/usr/bin/env python3
"""
virtual_remote.py — USB 虚拟遥控器 0x10 协议
                    IMU 姿态数据 0x11 协议

实现 USB虚拟遥控器.md 定义的 VIRTUAL_RC_SETPOINT 协议：
  40 字节帧，A5 5A 01 10 头部，28 字节 payload，CRC-16/CCITT-FALSE
  九宫格模式 (main_switch*3+sub_switch)，DEADMAN/MOTION_ENABLE/SMOOTH_STOP 标志

同时扩展 IMU_ORIENTATION 0x11 协议，发送处理后的 IMU 姿态：
  float32[6] (roll, pitch, yaw, gyro_x, gyro_y, gyro_z) + uint32 timestamp
"""

import struct, time, threading, os
import serial
import serial.tools.list_ports

VIRTUAL_RC_FORMAT = "<IIBBHhhhhhHI"
assert struct.calcsize(VIRTUAL_RC_FORMAT) == 28

MAGIC      = b"\xA5\x5A"
MSG_TYPE   = b"\x01\x10"
IMU_MSG_TYPE = b"\x01\x11"
FRAME_LEN  = 40
PAYLOAD_LEN = 28

IMU_FRAME_LEN = 40
IMU_PAYLOAD_LEN = 28
IMU_FORMAT = "<6fI"
assert struct.calcsize(IMU_FORMAT) == 28

# command_flags 位定义
DEADMAN_HELD   = 1 << 0
MOTION_ENABLE  = 1 << 1
SMOOTH_STOP    = 1 << 2

# 九宫格模式编号
MODE_IDLE       = 0   # LOW+LOW
MODE_LOW_WHEEL  = 1
MODE_SAFE_HOLD  = 2
MODE_STAND_HOLD = 3   # MID+LOW
MODE_STAND_WHEEL= 4
MODE_STAND_ARM  = 5   # MID+HIGH
MODE_GAIT_ONLY  = 6   # HIGH+LOW
MODE_GAIT_WHEEL = 7
MODE_HIGH_HOLD  = 8

# 速度档位 → speed_permille
SPEED_LOW  = -800
SPEED_MID  = 0
SPEED_HIGH = 800


def _crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def _float_to_permille(value: float) -> int:
    """-1.0~1.0 → -1000~1000"""
    return max(-1000, min(1000, int(round(value * 1000))))


class VirtualRemoteOutput:
    """USB 虚拟遥控器输出接口（唯一写入点）"""

    def __init__(self, port: str = "auto", timeout: float = 0.05):
        self.port = port
        self.timeout = timeout
        self._serial = None
        self._running = False
        self._frame_seq = 0
        self._counter = 0
        self._lock = threading.Lock()

        # — 当前样本 —
        self._session_id = 0
        self._main_switch = 0
        self._sub_switch = 0
        self._flags = 0
        self._yaw = 0
        self._forward = 0
        self._speed = 0
        self._arm_j0 = 0
        self._arm_j1 = 0
        self._host_start = 0
        # — IMU 数据 —
        self._imu_data = [0.0]*6   # roll, pitch, yaw, gx, gy, gz
        self._imu_updated = False

    # ------------------------------------------------------------------
    def connect(self) -> bool:
        if self._serial and self._serial.is_open:
            return True
        if self.port and self.port != "auto":
            try:
                self._serial = serial.Serial(self.port, 115200, timeout=self.timeout)
                return True
            except serial.SerialException as e:
                print(f"[VR] 无法打开 {self.port}: {e}")
                return False
        for p in serial.tools.list_ports.comports():
            if "ACM" in p.device or "USB" in p.device:
                try:
                    self._serial = serial.Serial(p.device, 115200, timeout=self.timeout)
                    self.port = p.device
                    return True
                except serial.SerialException:
                    continue
        print("[VR] 未找到 USB CDC 设备")
        return False

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()

    # ------------------------------------------------------------------
    # 导航/任务层唯一接口
    # ------------------------------------------------------------------
    def set_mode(self, main_switch: int, sub_switch: int):
        """设置九宫格模式 (0..2 各)"""
        with self._lock:
            self._main_switch = max(0, min(2, main_switch))
            self._sub_switch = max(0, min(2, sub_switch))

    def set_motion(self, forward: float, yaw: float):
        """设置运动量 (-1.0~1.0)"""
        with self._lock:
            self._forward = _float_to_permille(forward)
            self._yaw = _float_to_permille(yaw)

    def set_speed_axis(self, value: float):
        """设置速度轴 (-1.0~1.0)"""
        with self._lock:
            self._speed = _float_to_permille(value)

    def set_arm_jog(self, j0: float, j1: float):
        """设置机械臂微调 (-1.0~1.0)"""
        with self._lock:
            self._arm_j0 = _float_to_permille(j0)
            self._arm_j1 = _float_to_permille(j1)

    def refresh_autonomy_permit(self):
        """置位 DEADMAN + MOTION_ENABLE"""
        with self._lock:
            self._flags |= DEADMAN_HELD | MOTION_ENABLE
            self._host_start = time.time()

    def smooth_stop(self):
        """请求平稳收步 (CH1/CH2 归零)"""
        with self._lock:
            self._flags |= SMOOTH_STOP
            self._forward = 0
            self._yaw = 0

    def emergency_stop(self, reason: int = 0):
        """紧急停止 → 安全零帧 + 清 DEADMAN"""
        with self._lock:
            self._flags = 0
            self._main_switch = 0
            self._sub_switch = 0
            self._forward = 0
            self._yaw = 0
            self._speed = -1000
            self._arm_j0 = 0
            self._arm_j1 = 0

    def send_safety_zero(self):
        """发送安全零帧 (模式切换/暂停/释放时)"""
        with self._lock:
            old_flags = self._flags
            self._flags = SMOOTH_STOP if (old_flags & MOTION_ENABLE) else 0
        self._write_frame(self._build_frame())
        with self._lock:
            self._flags = old_flags

    # ------------------------------------------------------------------
    # IMU 姿态接口
    # ------------------------------------------------------------------
    def set_imu_orientation(self, roll_deg: float, pitch_deg: float, yaw_deg: float,
                             gyro_x: float, gyro_y: float, gyro_z: float):
        """设置 IMU 处理后的欧拉角 (度) 和角速度 (rad/s)"""
        with self._lock:
            self._imu_data = [roll_deg, pitch_deg, yaw_deg, gyro_x, gyro_y, gyro_z]
            self._imu_updated = True

    # ------------------------------------------------------------------
    def _build_frame(self) -> bytes:
        with self._lock:
            payload = struct.pack(
                VIRTUAL_RC_FORMAT,
                self._session_id,
                int(time.time() * 1000) & 0xFFFFFFFF,
                self._main_switch,
                self._sub_switch,
                self._flags,
                self._yaw,
                self._forward,
                self._speed,
                self._arm_j0,
                self._arm_j1,
                0x0067,           # valid_mask: CH1/2/3/6/7
                self._counter,
            )
            self._counter += 1

        header = MAGIC + MSG_TYPE
        wrapper = struct.pack("<HH", self._frame_seq, PAYLOAD_LEN)
        pre_crc = header + b"\x00\x00" + wrapper + payload
        crc = _crc16_ccitt(pre_crc)
        self._frame_seq += 1
        return pre_crc + struct.pack("<H", crc)

    def _build_imu_frame(self) -> bytes | None:
        """构建 0x11 IMU 姿态帧。若无新数据返回 None"""
        with self._lock:
            if not self._imu_updated:
                return None
            data = list(self._imu_data)
        payload = struct.pack(IMU_FORMAT,
            data[0], data[1], data[2],   # roll, pitch, yaw
            data[3], data[4], data[5],   # gyro x/y/z
            int(time.time() * 1000) & 0xFFFFFFFF)
        header = MAGIC + IMU_MSG_TYPE
        wrapper = struct.pack("<HH", self._frame_seq, IMU_PAYLOAD_LEN)
        pre_crc = header + b"\x00\x00" + wrapper + payload
        crc = _crc16_ccitt(pre_crc)
        self._frame_seq += 1
        return pre_crc + struct.pack("<H", crc)

    def _write_frame(self, frame: bytes):
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(frame)
            except serial.SerialException:
                pass

    def start_loop(self, freq: float = 50.0):
        if self._running:
            return
        self._running = True
        self._tick = 0
        interval = 1.0 / freq
        def _loop():
            while self._running:
                # 0x10 控制帧：每 tick 都发
                ctrl = self._build_frame()
                self._write_frame(ctrl)
                # 0x11 IMU 帧：每 2 个 tick 发一次（25Hz）
                if self._tick % 2 == 0:
                    imu = self._build_imu_frame()
                    if imu:
                        self._write_frame(imu)
                self._tick += 1
                time.sleep(interval)
        threading.Thread(target=_loop, daemon=True).start()

    def stop_loop(self):
        self._running = False
