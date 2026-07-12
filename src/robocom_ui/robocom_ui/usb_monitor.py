#!/usr/bin/env python3
"""usb_monitor.py — USB 通信帧实时监控

显示上位机向下位机发送的每一帧 USB 0x10/0x11 数据包，带解析摘要。
频率与原发送频率一致（控制帧 50Hz，IMU 帧 25Hz）。

运行: ros2 run robocom_ui usb_monitor
"""

import rclpy, struct
from rclpy.node import Node
from std_msgs.msg import String


def parse_frame(data: bytes):
    """解析 USB 帧并返回可读摘要"""
    if len(data) < 10:
        return "长度不足"
    magic = data[:2]
    if magic != b"\xA5\x5A":
        return f"Magic错误: {magic.hex()}"
    mtype = data[2:4]
    seq = struct.unpack("<H", data[6:8])[0]
    plen = struct.unpack("<H", data[8:10])[0]
    payload = data[10:10+plen]

    if mtype == b"\x01\x10":
        # CONTROL
        if len(payload) >= 28:
            sid, htime, ms, ss, flgs, yaw, fwd, spd, aj0, aj1, vmask, cnt = \
                struct.unpack("<IIBBHhhhhhHI", payload[:28])
            mode = ms * 3 + ss
            return (f"CTRL  seq={seq:4d}  mode={mode}({ms},{ss})  "
                    f"flgs=0x{flgs:04x}  fwd={fwd:5d}  yaw={yaw:5d}  "
                    f"spd={spd:5d}  cnt={cnt}")
        return f"CTRL  seq={seq}  payload={plen}B"
    elif mtype == b"\x01\x11":
        # IMU
        if len(payload) >= 28:
            rl, pt, yw, gx, gy, gz, ts = struct.unpack("<6fI", payload[:28])
            return (f"IMU   seq={seq:4d}  roll={rl:+7.2f}  pitch={pt:+7.2f}  "
                    f"yaw={yw:+7.2f}  gyro=({gx:+.2f},{gy:+.2f},{gz:+.2f})")
        return f"IMU   seq={seq}  payload={plen}B"
    else:
        return f"0x{mtype.hex()}  seq={seq}  payload={plen}B"


class UsbMonitor(Node):
    def __init__(self):
        super().__init__("usb_monitor")
        self.create_subscription(String, "/usb_tx_frame", self._cb, 10)
        self._count = 0
        self.get_logger().info("USB 帧监控已启动 (Ctrl+C 退出)")

    def _cb(self, msg: String):
        try:
            data = bytes.fromhex(msg.data)
            summary = parse_frame(data)
            print(f"[{self._count:6d}] {summary}")
            self._count += 1
        except Exception as e:
            print(f"[{self._count:6d}] 解析错误: {e}")
            self._count += 1


def main(args=None):
    rclpy.init(args=args)
    node = UsbMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
