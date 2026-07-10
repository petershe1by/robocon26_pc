#!/usr/bin/env python3
"""
color_mask_detector.py — 颜色掩码兑换区识别节点（任务 6、8）

功能：
  - 通过 USB 摄像头识别地面归位区颜色
  - 依次靠近 4 个识别区，判断是否为对应的兑换区
  - 如果是则到达对应坐标 → 放下物块
  - 如果不是则判断下一个兑换区

颜色映射：
  - 归位区 0 (food)     → green
  - 归位区 1 (tool)     → gray
  - 归位区 2 (instrument) → blue
  - 归位区 3 (medicine)   → red
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


# HSV 颜色范围（需根据实际场地光照标定）
COLOR_RANGES = {
    'green': ([30, 50, 50], [90, 255, 255]),
    'grey':  ([0, 0, 100], [180, 12, 150]),
    'blue':  ([90, 50, 50], [140, 255, 255]),
    'red':   ([0, 80, 80], [10, 255, 255]),
}
COLOR_RANGE2 = {
    'red': ([160, 80, 80], [180, 255, 255]),
}

EXCHANGE_ID_COLOR = {
    0: 'blue',
    1: 'green',
    2: 'grey',
    3: 'red',
}


class ColorMaskDetector(Node):
    """颜色掩码兑换区识别节点"""

    def __init__(self):
        super().__init__('color_mask_detector')

        self.declare_parameter('camera_id', 1)  # 第二路 USB 摄像头
        self._cam_id = self.get_parameter('camera_id').value

        # ---------- 发布/订阅 ----------
        self.pub_place = self.create_publisher(Bool, '/place_complete', 10)
        self.pub_exchange_id = self.create_publisher(String, '/detected_exchange_id', 10)
        self.pub_mismatch = self.create_publisher(String, '/exchange_mismatch', 10)

        self.create_subscription(String, '/color_vision_start', self._start_cb, 10)
        self.create_subscription(String, '/color_vision_stop', self._stop_cb, 10)

        # ---------- 状态 ----------
        self._running = False
        self._target_exchange_id = 0
        self._timer = None
        self._check_count = 0
        self._max_checks = 30  # 最多检查 30 帧 (~3 秒) 仍不匹配则宣布失败

        self.get_logger().info('ColorMaskDetector 已启动')

    def _stop_cb(self, msg: String):
        """外部停止颜色识别"""
        self._running = False
        if self._timer:
            self.destroy_timer(self._timer)
            self._timer = None
        self.get_logger().info('颜色识别已停止')

    # ------------------------------------------------------------------
    def _start_cb(self, msg: String):
        """启动颜色识别"""
        if self._running:
            return

        # msg.data 格式: "exchange_1"
        if msg.data.startswith('exchange_'):
            self._target_exchange_id = int(msg.data.split('_')[1])
            self.get_logger().info(
                f'启动颜色识别，目标兑换区 ID: {self._target_exchange_id} '
                f'({EXCHANGE_ID_COLOR.get(self._target_exchange_id, "?")})'
            )

        self._running = True
        self._check_count = 0
        if self._timer is None:
            self._timer = self.create_timer(0.1, self._detect_loop)

    # ------------------------------------------------------------------
    def _detect_loop(self):
        """检测循环：识别兑换区颜色"""
        if not self._running:
            return

        self._check_count += 1
        # 如果连续多帧都不匹配，发布 mismatch
        if self._check_count > self._max_checks:
            self.get_logger().warn(
                f'颜色 {EXCHANGE_ID_COLOR.get(self._target_exchange_id, "?")} '
                f'不匹配 ({self._check_count} 帧)'
            )
            self.pub_mismatch.publish(String(data=f'{self._target_exchange_id}'))
            self._running = False
            if self._timer:
                self.destroy_timer(self._timer)
                self._timer = None
            return

        if cv2 is None:
            # 模拟模式
            self.get_logger().info(f'[模拟] 检测到兑换区 {self._target_exchange_id} 颜色匹配')
            self._place()
            return

        # 1. 捕获图像
        cap = cv2.VideoCapture(self._cam_id)
        if not cap.isOpened():
            return
        ret, frame = cap.read()
        cap.release()
        if not ret or frame is None:
            return

        # 2. HSV 颜色检测
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        target_color = EXCHANGE_ID_COLOR.get(self._target_exchange_id, '')
        if not target_color:
            return

        # 创建掩码
        mask = None
        if target_color in COLOR_RANGES:
            lower, upper = COLOR_RANGES[target_color]
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

        # 对某些颜色（如红色）合并第二个区间
        if target_color in COLOR_RANGE2:
            lower2, upper2 = COLOR_RANGE2[target_color]
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            if mask is not None:
                mask = cv2.bitwise_or(mask, mask2)
            else:
                mask = mask2

        if mask is None:
            return

        # 3. 计算颜色占比
        color_ratio = cv2.countNonZero(mask) / (frame.shape[0] * frame.shape[1])

        self.get_logger().debug(
            f'颜色 {target_color} 占比: {color_ratio:.3f}'
        )

        # 4. 判断是否匹配（阈值需标定）
        if color_ratio > 0.15:
            self.get_logger().info(
                f'✓ 匹配到兑换区 {self._target_exchange_id} ({target_color})'
            )
            self._place()

    # ------------------------------------------------------------------
    def _place(self):
        """放置物块"""
        id_msg = String()
        id_msg.data = f'{self._target_exchange_id}'
        self.pub_exchange_id.publish(id_msg)

        self.pub_place.publish(Bool(data=True))
        self._running = False
        if self._timer:
            self.destroy_timer(self._timer)
            self._timer = None
        self.get_logger().info(f'物块已放置到兑换区 {self._target_exchange_id}')


def main(args=None):
    rclpy.init(args=args)
    node = ColorMaskDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
