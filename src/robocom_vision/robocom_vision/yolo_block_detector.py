#!/usr/bin/env python3
"""
yolo_block_detector.py — YOLO 物资箱检测节点（任务 8）

通过 D435 相机识别物资箱类型：
  - food / tool / instrument / medicine
  - 判断视野中心是否在物块矩形内部 → 发布抓取确认信号
  - 判断对应归位区颜色
  - 机械臂 5 个状态机的视觉识别/辅助定位驱动

运行方式：
  - 被 navigation_node 通过 /yolo_start 启动
  - 通过 /yolo_stop 终止
"""

import math
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


# 物资箱 ID → 类型 / 归位区颜色映射
BLOCK_TYPE_MAP = {
    0: ('food', 'green'),
    1: ('tool', 'gray'),
    2: ('instrument', 'blue'),
    3: ('medicine', 'red'),
    4: ('food', 'green'),
    5: ('tool', 'gray'),
    6: ('instrument', 'blue'),
    7: ('medicine', 'red'),
}


class YOLOBlockDetector(Node):
    """YOLO 物资箱检测节点"""

    def __init__(self):
        super().__init__('yolo_block_detector')

        # ---------- 参数 ----------
        self.declare_parameter('model_path', 'src/robocom_vision/models/block_detector.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_id', 0)  # D435 color stream

        model_path = self.get_parameter('model_path').value
        if not os.path.isabs(model_path):
            pkg_share = get_package_share_directory('robocom_vision')
            resolved = os.path.join(pkg_share, 'models', os.path.basename(model_path))
            self.get_logger().info(f'模型路径解析: {model_path} -> {resolved}')
            model_path = resolved
        conf_thres = self.get_parameter('confidence_threshold').value
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

        self._conf_thres = conf_thres

        # ---------- 发布 ----------
        self.pub_grasp = self.create_publisher(Bool, '/grasp_complete', 10)
        self.pub_block_type = self.create_publisher(String, '/detected_block_type', 10)
        self.pub_arm = self.create_publisher(ArmCommand, '/arm_command', 10)

        # ---------- 订阅（启动/停止） ----------
        self.create_subscription(String, '/yolo_start', self._start_cb, 10)
        self.create_subscription(String, '/yolo_stop', self._stop_cb, 10)

        # ---------- 状态 ----------
        self._running = False
        self._target_block = None
        self._grasped = False
        self._timer = None

        self.get_logger().info('YOLOBlockDetector 已启动，等待启动信号...')

    # ------------------------------------------------------------------
    def _start_cb(self, msg: String):
        """启动 YOLO 检测"""
        if self._running:
            return

        # msg.data 格式: "block_3" 或 "block_7"
        if msg.data.startswith('block_'):
            self._target_block = int(msg.data.split('_')[1])
            self.get_logger().info(f'启动 YOLO 检测，目标物资箱: {self._target_block}')

        self._running = True
        self._grasped = False
        if self._timer is None:
            self._timer = self.create_timer(0.05, self._detect_loop)  # 20 Hz

    # ------------------------------------------------------------------
    def _stop_cb(self, msg: String):
        """停止 YOLO 检测"""
        self._running = False
        if self._timer:
            self.destroy_timer(self._timer)
            self._timer = None
        self.get_logger().info('YOLO 检测已停止')

    # ------------------------------------------------------------------
    def _detect_loop(self):
        """检测循环：识别物块并判断视野中心是否在矩形内"""
        if not self._running or self._grasped:
            return

        if self._model is None or cv2 is None:
            # 模拟模式
            self.get_logger().info('[模拟] 检测到物资箱，发送抓取确认')
            self.pub_grasp.publish(Bool(data=True))
            self._grasped = True
            self._running = False
            return

        # 1. 捕获 D435 彩色图像
        cap = cv2.VideoCapture(self._cam_id)
        if not cap.isOpened():
            return
        ret, frame = cap.read()
        cap.release()
        if not ret or frame is None:
            return

        # 2. YOLO 推理
        results = self._model(frame, conf=self._conf_thres, verbose=False)

        if not results or len(results) == 0:
            return

        # 3. 解析检测结果
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return

        frame_h, frame_w = frame.shape[:2]
        center_x, center_y = frame_w // 2, frame_h // 2

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = box.conf[0].item()
            cls_id = int(box.cls[0].item())

            if conf < self._conf_thres:
                continue

            # 判断视野中心是否在物块矩形内部
            if x1 <= center_x <= x2 and y1 <= center_y <= y2:
                self.get_logger().info(
                    f'视野中心在物块内 (cls={cls_id}, conf={conf:.2f})'
                )

                # 发布物块类型
                type_msg = String()
                type_msg.data = f'{cls_id}'
                self.pub_block_type.publish(type_msg)

                # 发送机械臂进入抓取状态
                arm_cmd = ArmCommand()
                arm_cmd.state = 3  # 抓取
                arm_cmd.command_valid = True
                self.pub_arm.publish(arm_cmd)

                # 确认抓取（深度信息验证由 depth_helper 节点完成）
                self.pub_grasp.publish(Bool(data=True))
                self._grasped = True
                self._running = False
                if self._timer:
                    self.destroy_timer(self._timer)
                    self._timer = None
                break

    # ------------------------------------------------------------------
    def get_target_info(self) -> tuple:
        """返回目标物资箱的 (类型, 归位区颜色)"""
        if self._target_block is not None and self._target_block in BLOCK_TYPE_MAP:
            return BLOCK_TYPE_MAP[self._target_block]
        return ('unknown', 'unknown')


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
