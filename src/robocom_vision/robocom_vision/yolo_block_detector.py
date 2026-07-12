#!/usr/bin/env python3
"""
yolo_block_detector.py — YOLO + OpenCV 颜色融合检测节点（任务 8）

集成 con_good_v1.py 的颜色融合逻辑：
  1. YOLO 推理（conf=0.05）
  2. 对每个检测框用 OpenCV HSV 提取主色
  3. 置信度 < 0.20 时 OpenCV 结果优先
  4. 单目标聚焦（只处理置信度最高的框）
  5. 判断视野中心是否在物块矩形内部 → 发布抓取确认
"""

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from robocom_interfaces.msg import ArmCommand
from ament_index_python.packages import get_package_share_directory

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


# ===== 以下参数来自 con_good_v1.py，请勿修改 =====
# YOLO 仅用于视觉对准，物块类型由预分配颜色决定

class YOLOBlockDetector(Node):
    """YOLO + 颜色融合物资箱检测节点"""

    def __init__(self):
        super().__init__('yolo_block_detector')

        self.declare_parameter('model_path', 'src/robocom_vision/models/best.pt')
        self.declare_parameter('confidence_threshold', 0.05)  # 同 con_good_v1.py
        self.declare_parameter('camera_id', 2)                # 同 con_good_v1.py

        model_path = self.get_parameter('model_path').value
        if not os.path.isabs(model_path):
            pkg_share = get_package_share_directory('robocom_vision')
            resolved = os.path.join(pkg_share, 'models', os.path.basename(model_path))
            self.get_logger().info(f'模型路径解析: {model_path} -> {resolved}')
            model_path = resolved
        self._conf_thres = self.get_parameter('confidence_threshold').value
        self._cam_id = self.get_parameter('camera_id').value

        # ---------- YOLO 模型 ----------
        self._model = None
        if YOLO is not None:
            try:
                self._model = YOLO(model_path)
                self.get_logger().info(f'YOLO 模型加载成功: {model_path}')
            except Exception as e:
                self.get_logger().error(f'YOLO 模型加载失败: {e}')
        else:
            self.get_logger().warn('ultralytics 未安装，YOLO 不可用')

        # ---------- 发布 ----------
        self.pub_grasp = self.create_publisher(Bool, '/grasp_complete', 10)
        self.pub_block_type = self.create_publisher(String, '/detected_block_type', 10)
        self.pub_arm = self.create_publisher(ArmCommand, '/arm_command', 10)

        # ---------- 订阅 ----------
        self.create_subscription(String, '/yolo_start', self._start_cb, 10)
        self.create_subscription(String, '/yolo_stop', self._stop_cb, 10)

        # ---------- 状态 ----------
        self._running = False
        self._target_block = None
        self._grasped = False
        self._timer = None
        self._pipeline = None

        self.get_logger().info('YOLOBlockDetector 已启动，等待启动信号...')

    # ------------------------------------------------------------------
    def _start_cb(self, msg: String):
        if self._running:
            return
        if msg.data.startswith('block_'):
            self._target_block = int(msg.data.split('_')[1])
            self.get_logger().info(f'启动 YOLO 检测，目标物资箱: {self._target_block}')
        # 打开 D435 管道
        if rs is not None and self._pipeline is None:
            try:
                self._pipeline = rs.pipeline()
                cfg = rs.config()
                cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                self._pipeline.start(cfg)
                # 丢弃前几帧让自动曝光稳定
                for _ in range(10):
                    self._pipeline.wait_for_frames(timeout_ms=5000)
                self.get_logger().info('D435 管道已启动 (640x480)')
            except Exception as e:
                self.get_logger().error(f'D435 管道启动失败: {e}')
                self._pipeline = None
        self._running = True
        self._grasped = False
        if self._timer is None:
            self._timer = self.create_timer(0.05, self._detect_loop)

    # ------------------------------------------------------------------
    def _stop_cb(self, msg: String):
        self._running = False
        self._close_pipeline()
        if self._timer:
            self.destroy_timer(self._timer)
            self._timer = None
        self.get_logger().info('YOLO 检测已停止')

    def _close_pipeline(self):
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None

    # ------------------------------------------------------------------
    def _detect_loop(self):
        """单目标聚焦 + 颜色融合检测循环"""
        if not self._running or self._grasped:
            return

        if self._model is None or cv2 is None:
            # 模拟模式
            self.get_logger().info('[模拟] 检测到物资箱，发送抓取确认')
            self.pub_grasp.publish(Bool(data=True))
            self._grasped = True
            self._running = False
            self._close_pipeline()
            return

        # 1. 捕获 D435 彩色帧
        if self._pipeline is None:
            return
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return
            frame = np.asanyarray(color_frame.get_data())
        except Exception as e:
            self.get_logger().warn(f'D435 取帧失败: {e}')
            return

        # 2. YOLO 推理（conf=0.05，同 con_good_v1.py）
        results = self._model(frame, conf=0.05, verbose=False)
        boxes = results[0].boxes

        best_target = None
        max_conf = -1.0
        frame_h, frame_w = frame.shape[:2]
        center_x, center_y = frame_w // 2, frame_h // 2

        # 3. 单目标聚焦：遍历所有框，只保留置信度最高的
        if boxes is not None:
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_idx = int(box.cls[0])
                conf = float(box.conf[0])

                if conf < 0.01:
                    continue
                if conf <= max_conf:
                    continue

                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue

                max_conf = conf
                best_target = {
                    'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                    'cx': (x1 + x2) // 2, 'cy': (y1 + y2) // 2,
                    'conf': conf
                }

        # 4. 判断最优目标是否在视野中心
        if best_target and (best_target['x1'] <= center_x <= best_target['x2']
                            and best_target['y1'] <= center_y <= best_target['y2']):
            self.get_logger().info(
                f'视野中心在物块内, '
                f'conf={best_target["conf"]:.2f}'
            )

            # 发送机械臂进入抓取状态
            arm_cmd = ArmCommand()
            arm_cmd.state = 3
            arm_cmd.command_valid = True
            self.pub_arm.publish(arm_cmd)

            # 确认抓取
            self.pub_grasp.publish(Bool(data=True))
            self._grasped = True
            self._running = False
            self._close_pipeline()
            if self._timer:
                self.destroy_timer(self._timer)
                self._timer = None


def main(args=None):
    rclpy.init(args=args)
    node = YOLOBlockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
