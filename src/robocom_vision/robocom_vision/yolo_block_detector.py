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


# ===== 以下参数来自 con_good_v1.py，请勿修改 =====
CLASS_NAMES = ['blue', 'green', 'grey', 'red']

COLOR_THRESHOLDS = {
    'red':    [([0, 80, 80], [10, 255, 255]), ([160, 80, 80], [180, 255, 255])],
    'green':  [([30, 50, 50], [90, 255, 255])],
    'blue':   [([90, 50, 50], [140, 255, 255])],
    'grey':   [([0, 0, 100], [180, 12, 150])]
}
# ==================================================


def get_dominant_color(roi_img):
    """返回 roi 区域的主色名称，无主色则返回 None"""
    hsv_roi = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    total_pixels = roi_img.shape[0] * roi_img.shape[1]
    color_pixel_counts = {}

    for color_name, ranges in COLOR_THRESHOLDS.items():
        mask = np.zeros(hsv_roi.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            lower_np = np.array(lower, dtype=np.uint8)
            upper_np = np.array(upper, dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv_roi, lower_np, upper_np))

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        color_ratio = cv2.countNonZero(mask) / total_pixels
        color_pixel_counts[color_name] = color_ratio

    if not color_pixel_counts:
        return None
    dominant = max(color_pixel_counts, key=color_pixel_counts.get)
    threshold = 0.50 if dominant == 'grey' else 0.15
    if color_pixel_counts[dominant] < threshold:
        return None
    return dominant


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

        self.get_logger().info('YOLOBlockDetector 已启动，等待启动信号...')

    # ------------------------------------------------------------------
    def _start_cb(self, msg: String):
        if self._running:
            return
        if msg.data.startswith('block_'):
            self._target_block = int(msg.data.split('_')[1])
            self.get_logger().info(f'启动 YOLO 检测，目标物资箱: {self._target_block}')
        self._running = True
        self._grasped = False
        if self._timer is None:
            self._timer = self.create_timer(0.05, self._detect_loop)

    # ------------------------------------------------------------------
    def _stop_cb(self, msg: String):
        self._running = False
        if self._timer:
            self.destroy_timer(self._timer)
            self._timer = None
        self.get_logger().info('YOLO 检测已停止')

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
            return

        # 1. 捕获图像
        cap = cv2.VideoCapture(self._cam_id)
        if not cap.isOpened():
            return
        ret, frame = cap.read()
        cap.release()
        if not ret or frame is None:
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

                # 颜色融合（同 con_good_v1.py）
                opencv_color = get_dominant_color(roi)
                yolo_class = CLASS_NAMES[cls_idx]
                final_class = yolo_class

                if opencv_color is not None:
                    if yolo_class != opencv_color:
                        final_class = opencv_color if conf < 0.20 else yolo_class

                max_conf = conf
                best_target = {
                    'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                    'cx': (x1 + x2) // 2, 'cy': (y1 + y2) // 2,
                    'cls_idx': cls_idx,
                    'final_class': final_class,
                    'conf': conf
                }

        # 4. 判断最优目标是否在视野中心
        if best_target and (best_target['x1'] <= center_x <= best_target['x2']
                            and best_target['y1'] <= center_y <= best_target['y2']):
            t = best_target
            # 将 final_class 转成对应的类索引
            try:
                final_idx = CLASS_NAMES.index(t['final_class'])
            except ValueError:
                final_idx = t['cls_idx']

            self.get_logger().info(
                f'视野中心在物块内 → class={t["final_class"]}(idx={final_idx}), '
                f'conf={t["conf"]:.2f}'
            )

            # 发布物块类型（导航用此决定兑换站）
            type_msg = String()
            type_msg.data = f'{final_idx}'
            self.pub_block_type.publish(type_msg)

            # 发送机械臂进入抓取状态
            arm_cmd = ArmCommand()
            arm_cmd.state = 3
            arm_cmd.command_valid = True
            self.pub_arm.publish(arm_cmd)

            # 确认抓取
            self.pub_grasp.publish(Bool(data=True))
            self._grasped = True
            self._running = False
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
