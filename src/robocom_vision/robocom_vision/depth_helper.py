#!/usr/bin/env python3
"""
depth_helper.py — D435 深度信息辅助节点（任务 5）

功能：
  - 获取 D435 深度图像
  - 在吸取过程中监测深度值是否变化
  - 当吸取后深度信息不再变化 → 认为吸取成功
  - 发布 /grasp_verified 信号

需要: pip install pyrealsense2
"""

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


class DepthHelper(Node):
    """D435 深度辅助验证节点"""

    def __init__(self):
        super().__init__('depth_helper')

        self.declare_parameter('depth_stable_threshold', 0.005)   # 5mm 变化视为稳定
        self.declare_parameter('stable_frames', 10)               # 连续稳定帧数

        self._threshold = self.get_parameter('depth_stable_threshold').value
        self._stable_frames = self.get_parameter('stable_frames').value

        # ---------- 发布 ----------
        self.pub_verified = self.create_publisher(Bool, '/grasp_verified', 10)

        # ---------- 状态 ----------
        self._prev_depth = None
        self._stable_count = 0
        self._grasping = False

        # ---------- 订阅 ----------
        self.create_subscription(Bool, '/grasp_complete', self._grasp_start_cb, 10)

        # ---------- 定时器 ----------
        self.create_timer(0.1, self._depth_check)

    # ------------------------------------------------------------------
    def _grasp_start_cb(self, msg: Bool):
        """开始吸取 → 开始监测深度"""
        if msg.data:
            self._start_d435()
            self._grasping = True
            self._prev_depth = None
            self._stable_count = 0
            self.get_logger().info('开始深度监测（吸取验证）')

    def _start_d435(self):
        """需要时才启动 D435 深度流（不占用设备）"""
        if self._pipeline is not None:
            return
        if rs is None:
            return
        try:
            self._pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self._pipeline.start(config)
            self.get_logger().info('D435 深度流已启动')
        except Exception as e:
            self.get_logger().warn(f'D435 启动失败: {e}')

    def _stop_d435(self):
        """释放 D435 设备"""
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None
            self.get_logger().info('D435 深度流已释放')

    # ------------------------------------------------------------------
    def _depth_check(self):
        """检查深度是否稳定（吸取成功标志）"""
        if not self._grasping:
            return

        if self._pipeline is None:
            # 模拟模式
            self._simulate_depth_stable()
            return

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=200)
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                return

            # 取画面中心区域的深度平均值
            depth_image = np.asanyarray(depth_frame.get_data())
            h, w = depth_image.shape
            center_region = depth_image[h//2-20:h//2+20, w//2-20:w//2+20]
            current_depth = np.mean(center_region)

            self.get_logger().debug(f'中心深度: {current_depth:.3f} m')

            if self._prev_depth is not None:
                diff = abs(current_depth - self._prev_depth)
                if diff < self._threshold:
                    self._stable_count += 1
                else:
                    self._stable_count = 0

                if self._stable_count >= self._stable_frames:
                    self.get_logger().info('✓ 深度稳定，吸取成功验证')
                    self.pub_verified.publish(Bool(data=True))
                    self._stop_d435()
                    self._grasping = False
                    self._stable_count = 0

            self._prev_depth = current_depth

        except Exception as e:
            self.get_logger().warn(f'深度读取异常: {e}')

    # ------------------------------------------------------------------
    def _simulate_depth_stable(self):
        """模拟模式：延迟后直接确认"""
        if self._prev_depth is None:
            self._prev_depth = 0.5
            return

        self._stable_count += 1
        if self._stable_count >= 5:
            self.get_logger().info('[模拟] 深度稳定，吸取成功')
            self.pub_verified.publish(Bool(data=True))
            self._stop_d435()
            self._grasping = False
            self._stable_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = DepthHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_d435()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
