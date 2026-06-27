#!/usr/bin/env python3
"""sbus_bridge.py - USB CDC SBUS 桥接"""

import time
import threading
import serial
import serial.tools.list_ports


SBUS_START_BYTE = 0x0F
SBUS_END_BYTE = 0x00
SBUS_FRAME_LEN = 25

SBUS_MIN = 352
SBUS_MID = 1024
SBUS_MAX = 1695

SWITCH_LOW = 353
SWITCH_MID = 1024
SWITCH_HIGH = 1695


def float_to_sbus(value: float) -> int:
    """-1.0~1.0 -> 352~1695"""
    value = max(-1.0, min(1.0, value))
    return int(SBUS_MID + value * (SBUS_MAX - SBUS_MID))


def pack_sbus_frame(channels: list) -> bytes:
    """打包 16 路 11-bit SBUS 通道 -> 25 字节 (无bug版)"""
    frame = bytearray(25)
    frame[0] = SBUS_START_BYTE
    frame[24] = SBUS_END_BYTE

    data_buf = 0
    bits_in_buf = 0
    out_idx = 1  # frame[1..22] 数据区

    for ch_val in channels[:16]:
        ch_val = max(SBUS_MIN, min(SBUS_MAX, ch_val))
        data_buf |= (ch_val << bits_in_buf)
        bits_in_buf += 11
        while bits_in_buf >= 8:
            frame[out_idx] = data_buf & 0xFF
            data_buf >>= 8
            out_idx += 1
            bits_in_buf -= 8

    if bits_in_buf > 0:
        frame[out_idx] = data_buf & 0xFF

    return bytes(frame)


class SBUSBridge:
    """USB CDC -> SBUS 桥接器"""

    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 100000, timeout: float = 0.05):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial = None
        self._running = False
        self._lock = threading.Lock()
        self._channels = [SBUS_MID] * 16
        self._channels[4] = SWITCH_LOW
        self._channels[7] = SWITCH_LOW
        self._channels[8] = SWITCH_LOW
        self._channels[9] = SWITCH_LOW
        self._enabled = True

    def connect(self) -> bool:
        if self._serial and self._serial.is_open:
            return True
        if self.port and self.port != "auto":
            try:
                self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
                return True
            except serial.SerialException as e:
                print(f"[SBUS] 无法打开端口 {self.port}: {e}")
                return False
        for p in serial.tools.list_ports.comports():
            if "ACM" in p.device or "USB" in p.device:
                try:
                    self._serial = serial.Serial(p.device, self.baud, timeout=self.timeout)
                    self.port = p.device
                    return True
                except serial.SerialException:
                    continue
        print("[SBUS] 未找到 USB CDC 设备")
        return False

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()

    def set_channel(self, ch: int, value: int):
        if 0 <= ch < 16:
            with self._lock:
                self._channels[ch] = max(SBUS_MIN, min(SBUS_MAX, value))

    def set_joystick(self, ch: int, value: float):
        sbus_ch_map = {0: 3, 1: 2, 2: 0, 3: 1}
        if ch in sbus_ch_map:
            self.set_channel(sbus_ch_map[ch], float_to_sbus(value))

    def set_switch(self, switch_id: int, position: int):
        sbus_ch_map = {0: 9, 1: 4, 2: 8, 3: 7}
        if switch_id not in sbus_ch_map:
            return
        val = {0: SWITCH_LOW, 1: SWITCH_MID}.get(position, SWITCH_HIGH)
        self.set_channel(sbus_ch_map[switch_id], val)

    def enable(self):
        self._enabled = True
        self.set_switch(0, 1)

    def disable(self):
        self._enabled = False
        self.set_switch(0, 0)
        for ch in range(4):
            self.set_joystick(ch, 0.0)

    def send_frame(self) -> bool:
        if not self._serial or not self._serial.is_open:
            return False
        with self._lock:
            frame = pack_sbus_frame(self._channels)
        try:
            self._serial.write(frame)
            return True
        except serial.SerialException as e:
            print(f"[SBUS] 发送失败: {e}")
            return False

    def start_loop(self, freq: float = 100.0):
        if self._running:
            return
        self._running = True
        interval = 1.0 / freq

        def _loop():
            while self._running:
                if self._enabled:
                    self.send_frame()
                time.sleep(interval)
        threading.Thread(target=_loop, daemon=True).start()

    def stop_loop(self):
        self._running = False