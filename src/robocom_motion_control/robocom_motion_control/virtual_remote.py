#!/usr/bin/env python3
"""
virtual_remote.py — USB 虚拟遥控器（基于下位机 host 协议库）
"""

import struct, time, threading, os
import serial
import serial.tools.list_ports

from .host.quadruped_protocol import (
    ControlStream, VirtualRemoteCommand, RobotMode,
    VirtualImuFrame, DEADMAN_HELD, MOTION_ENABLE, SMOOTH_STOP,
)

SPEED_LOW  = 300
SPEED_MID  = 600
SPEED_HIGH = 1000


class VirtualRemoteOutput:
    """虚拟遥控器：内部使用 host 库 ControlStream 编码 + 管理串口与 50Hz 循环"""

    def __init__(self, port="auto", timeout=0.1, tx_callback=None):
        self.port = port
        self.timeout = timeout
        self.tx_callback = tx_callback
        self._serial = None
        self._stream = ControlStream()
        self._cmd = VirtualRemoteCommand.safe_zero()
        self._lock = threading.Lock()
        self._running = False
        self._imu_data = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._imu_updated = False

    # ------------------------------------------------------------------
    def connect(self) -> bool:
        if self._serial and self._serial.is_open:
            return True
        if self.port and self.port != "auto":
            try:
                self._serial = serial.Serial(self.port, 115200, timeout=self.timeout)
                self._stream = ControlStream()
                return True
            except serial.SerialException as e:
                print(f"[VR] 无法打开 {self.port}: {e}")
                return False
        for p in serial.tools.list_ports.comports():
            if "ACM" in p.device or "USB" in p.device:
                try:
                    self._serial = serial.Serial(p.device, 115200, timeout=self.timeout)
                    self.port = p.device
                    self._stream = ControlStream()
                    return True
                except serial.SerialException:
                    continue
        print("[VR] 未找到 USB CDC 设备")
        return False

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self._running = False

    # ------------------------------------------------------------------
    def set_mode(self, main_switch: int, sub_switch: int):
        mode_idx = main_switch * 3 + sub_switch
        modes = list(RobotMode)
        if 0 <= mode_idx < len(modes):
            with self._lock:
                self._cmd = VirtualRemoteCommand(
                    mode=modes[mode_idx],
                    forward=self._cmd.forward,
                    yaw=self._cmd.yaw,
                    speed_axis=self._cmd.speed_axis,
                    motion_enable=self._cmd.motion_enable,
                    deadman=self._cmd.deadman,
                    smooth_stop=self._cmd.smooth_stop,
                )

    def set_motion(self, forward: float, yaw: float):
        with self._lock:
            self._cmd = VirtualRemoteCommand(
                mode=self._cmd.mode, forward=forward, yaw=yaw,
                speed_axis=self._cmd.speed_axis,
                motion_enable=self._cmd.motion_enable,
                deadman=self._cmd.deadman, smooth_stop=self._cmd.smooth_stop,
            )

    def set_speed_axis(self, speed: float):
        with self._lock:
            self._cmd = VirtualRemoteCommand(
                mode=self._cmd.mode, forward=self._cmd.forward, yaw=self._cmd.yaw,
                speed_axis=speed,
                motion_enable=self._cmd.motion_enable,
                deadman=self._cmd.deadman, smooth_stop=self._cmd.smooth_stop,
            )

    def set_imu_orientation(self, roll, pitch, yaw, gx=0.0, gy=0.0, gz=0.0):
        with self._lock:
            self._imu_data = (roll, pitch, yaw, gx, gy, gz)
            self._imu_updated = True

    # ------------------------------------------------------------------
    def emergency_stop(self):
        with self._lock:
            self._cmd = VirtualRemoteCommand(deadman=False, motion_enable=False, smooth_stop=True)

    def send_safety_zero(self):
        with self._lock:
            self._cmd = VirtualRemoteCommand(
                mode=self._cmd.mode,
                motion_enable=self._cmd.motion_enable,
                deadman=self._cmd.deadman,
                smooth_stop=self._cmd.smooth_stop,
                speed_axis=self._cmd.speed_axis,
            )

    def smooth_stop(self):
        with self._lock:
            self._cmd = VirtualRemoteCommand(
                mode=self._cmd.mode,
                motion_enable=self._cmd.motion_enable,
                deadman=False, smooth_stop=True,
                speed_axis=self._cmd.speed_axis,
            )

    def refresh_autonomy_permit(self):
        with self._lock:
            self._cmd = VirtualRemoteCommand(
                mode=self._cmd.mode, forward=self._cmd.forward, yaw=self._cmd.yaw,
                speed_axis=self._cmd.speed_axis,
                motion_enable=self._cmd.motion_enable, deadman=True,
            )

    # ------------------------------------------------------------------
    def start_loop(self, freq: float = 50.0):
        if self._running:
            return
        self._running = True
        interval = 1.0 / freq
        tick = 0

        def _loop():
            nonlocal tick
            while self._running and self._serial and self._serial.is_open:
                try:
                    with self._lock:
                        cmd = self._cmd
                    host_ms = int(time.time() * 1000)
                    frame = self._stream.encode(cmd, host_ms)
                    # 控制帧
                    if self.tx_callback:
                        self.tx_callback(frame)
                    self._serial.write(frame)
                    # IMU 帧（每 2 tick 一次 = 25Hz）
                    if tick % 2 == 0:
                        with self._lock:
                            imu = self._imu_data
                        imu_frame = VirtualImuFrame(*imu).encode(self._stream.frame_seq)
                        self._stream.frame_seq = (self._stream.frame_seq + 1) & 0xFFFF
                        if self.tx_callback:
                            self.tx_callback(imu_frame)
                        self._serial.write(imu_frame)
                    tick += 1
                except Exception:
                    pass
                time.sleep(interval)
            self._running = False

        threading.Thread(target=_loop, daemon=True).start()

    def stop_loop(self):
        self._running = False
