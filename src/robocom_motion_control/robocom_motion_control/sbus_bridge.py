#!/usr/bin/env python3
"""
sbus_bridge.py — USB CDC SBUS 桥接

将上位机 MotionCmd 指令打包为 SBUS 帧（25 字节），通过 USB CDC 串口
发送给下位机 STM32，模拟遥控器信号。

SBUS 帧格式：
  [0]   = 0x0F  (起始字节)
  [1-22] = 16 个通道 × 11 位
  [23]  = flag
  [24]  = 0x00  (结束字节)

通道映射（与 STM32 遥控代码一致）：
  ch[0]  → 右摇杆水平 (roll)     → joystick[2]
  ch[1]  → 右摇杆垂直 (pitch)    → joystick[3]
  ch[2]  → 左摇杆垂直            → joystick[1] (差分处理→加速度)
  ch[3]  → 左摇杆水平 (yaw)      → joystick[0]
  ch[4]  → SB 三档开关            → state[1]
  ch[7]  → SD 三档开关            → state[3]
  ch[8]  → SD 两档开关            → state[2]
  ch[9]  → SA 两档开关            → state[0]

SBUS 值范围：352 ~ 1024 (中位) ~ 1695
"""

import struct
import time
import threading

import serial
import serial.tools.list_ports


SBUS_START_BYTE = 0x0F
SBUS_END_BYTE = 0x00
SBUS_FRAME_LEN = 25

# SBUS 通道值常量
SBUS_MIN = 352
SBUS_MID = 1024
SBUS_MAX = 1695

# 拨杆状态 → SBUS 值
SWITCH_LOW = 353    # 低位
SWITCH_MID = 1024   # 中位
SWITCH_HIGH = 1695  # 高位


def float_to_sbus(value: float) -> int:
    """将 -1.0~1.0 浮点数映射为 SBUS 通道值 352~1695"""
    value = max(-1.0, min(1.0, value))
    # -1 → 352, 0 → 1024, 1 → 1695
    return int(SBUS_MID + value * (SBUS_MAX - SBUS_MID))


def pack_sbus_frame(channels: list) -> bytes:
    """
    打包 16 个 11-bit 通道值 → 25 字节 SBUS 帧
    channels: 长度为 16 的 list，每个值范围 352~1695
    """
    if len(channels) != 16:
        channels = channels[:16] + [SBUS_MID] * (16 - len(channels))

    frame = bytearray(SBUS_FRAME_LEN)
    frame[0] = SBUS_START_BYTE

    # Bit packing: 16 channels × 11 bits = 176 bits = 22 bytes
    idx = 1
    bit_offset = 0
    for ch_val in channels:
        # 将 11-bit 值放入 buffer
        byte_idx = bit_offset // 8
        bit_remain = bit_offset % 8

        val_low = (ch_val << bit_remain) & 0xFF
        val_high = (ch_val >> (8 - bit_remain)) if bit_remain > 0 else 0

        if byte_idx < 22:
            frame[idx + byte_idx] |= val_low
        if byte_idx + 1 < 22 and bit_remain > 0:
            frame[idx + byte_idx + 1] = val_high

        bit_offset += 11

    # frame[23] = flag (ch17=ch15, ch18=ch16)
    frame[23] = 0x00
    # 如果需要设置 ch17/ch18:
    # if channel[16]: frame[23] |= 0x01
    # if channel[17]: frame[23] |= 0x02

    frame[24] = SBUS_END_BYTE
    return bytes(frame)


class SBUSBridge:
    """
    USB CDC → SBUS 桥接器
    通过串口发送 SBUS 帧给下位机 STM32
    """

    def __init__(self, port: str = '/dev/ttyACM0', baud: int = 100000,
                 timeout: float = 0.05):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial = None
        self._running = False
        self._lock = threading.Lock()

        # 默认通道值：全部中位，拨杆低位
        self._channels = [SBUS_MID] * 16
        self._channels[4] = SWITCH_LOW   # SB 低位
        self._channels[7] = SWITCH_LOW   # SD 三档低位
        self._channels[8] = SWITCH_LOW   # SD 两档低位
        self._channels[9] = SWITCH_LOW   # SA 低位

        # 使能状态
        self._enabled = True

    # ------------------------------------------------------------------
    def connect(self) -> bool:
        """自动连接 USB CDC 设备"""
        if self._serial and self._serial.is_open:
            return True

        # 自动搜索
        if self.port and self.port != 'auto':
            try:
                self._serial = serial.Serial(
                    self.port, self.baud, timeout=self.timeout,
                    bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE
                )
                return True
            except serial.SerialException as e:
                print(f'[SBUS] 无法打开端口 {self.port}: {e}')
                return False

        # 'auto' 模式：搜索 CDC 设备
        for port_info in serial.tools.list_ports.comports():
            if 'ACM' in port_info.device or 'USB' in port_info.device:
                try:
                    self._serial = serial.Serial(
                        port_info.device, self.baud, timeout=self.timeout
                    )
                    self.port = port_info.device
                    return True
                except serial.SerialException:
                    continue

        print('[SBUS] 未找到 USB CDC 设备')
        return False

    # ------------------------------------------------------------------
    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()

    # ------------------------------------------------------------------
    def set_channel(self, ch: int, value: int):
        """设置单个通道值 (352~1695)"""
        if 0 <= ch < 16:
            with self._lock:
                self._channels[ch] = max(SBUS_MIN, min(SBUS_MAX, value))

    # ------------------------------------------------------------------
    def set_joystick(self, ch: int, value: float):
        """
        设置摇杆通道 (-1.0 ~ 1.0 浮点数)
        ch: 0=左水平(yaw), 1=左垂直, 2=右水平, 3=右垂直
        """
        sbus_ch_map = {0: 3, 1: 2, 2: 0, 3: 1}
        if ch in sbus_ch_map:
            self.set_channel(sbus_ch_map[ch], float_to_sbus(value))

    # ------------------------------------------------------------------
    def set_switch(self, switch_id: int, position: int):
        """
        设置拨杆状态（模拟遥控器拨杆）
        switch_id: 0=SA(两档), 1=SB(三档), 2=SD1(两档), 3=SD2(三档)
        position: 0=低位, 1=中位(三档), 2=高位
        """
        sbus_ch_map = {0: 9, 1: 4, 2: 8, 3: 7}
        if switch_id not in sbus_ch_map:
            return

        if position == 0:
            val = SWITCH_LOW
        elif position == 1:
            val = SWITCH_MID
        else:
            val = SWITCH_HIGH

        self.set_channel(sbus_ch_map[switch_id], val)

    # ------------------------------------------------------------------
    def enable(self):
        """下位机使能"""
        self._enabled = True
        # SA 拨杆→高位 = 使能
        self.set_switch(0, 1)

    def disable(self):
        """下位机失能 / 急停"""
        self._enabled = False
        # SA 拨杆→低位 = 失能
        self.set_switch(0, 0)
        self.set_joystick(0, 0.0)
        self.set_joystick(1, 0.0)
        self.set_joystick(2, 0.0)
        self.set_joystick(3, 0.0)

    # ------------------------------------------------------------------
    def get_joystick_target(self, ch: int) -> int:
        """获取摇杆目标值（-100~100），与下位机 sbus_joystick.target 一致"""
        sbus_ch_map = {0: 3, 1: 2, 2: 0, 3: 1}
        if ch not in sbus_ch_map:
            return 0
        raw = self._channels[sbus_ch_map[ch]]
        return (raw - SBUS_MID) * 100 // 671

    # ------------------------------------------------------------------
    def send_frame(self):
        """发送当前 SBUS 帧"""
        if not self._serial or not self._serial.is_open:
            return False

        with self._lock:
            frame = pack_sbus_frame(self._channels)

        try:
            self._serial.write(frame)
            return True
        except serial.SerialException as e:
            print(f'[SBUS] 发送失败: {e}')
            return False

    # ------------------------------------------------------------------
    def start_loop(self, freq: float = 100.0):
        """在后台线程中持续发送 SBUS 帧"""
        if self._running:
            return

        self._running = True
        interval = 1.0 / freq

        def _loop():
            while self._running:
                if self._enabled:
                    self.send_frame()
                time.sleep(interval)

        t = threading.Thread(target=_loop, daemon=True)
        t.start()

    def stop_loop(self):
        self._running = False
