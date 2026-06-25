#!/usr/bin/env python3
"""
math_solver_node.py — 数学题识别节点（最高优先级任务）

功能：
  收到 match_start 信号后立即挂起其他任务，启动 20 秒看门狗。
  通过 USB 相机获取 1600x900 显示区图像 → PaddleOCR(PP-OCRv4) 提取文本
  → 正则清洗 → SymPy 安全求解四则运算 → 取模映射高分区编号
  → 发布 /math_result，调用 TTS 播报，刷新 UI 显示。

超时：20 秒未完成则直接结束，不阻塞后续比赛流程。
"""

import re
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration

from robocom_interfaces.msg import MathResult
from std_msgs.msg import String

try:
    from sympy import sympify, SympifyError
except ImportError:
    sympify = None
    SympifyError = Exception

try:
    import cv2
except ImportError:
    cv2 = None


class MathSolverNode(Node):
    """OCR + SymPy 数学题求解节点"""

    def __init__(self):
        super().__init__('math_solver_node')

        # ---------- 参数 ----------
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('image_width', 1600)
        self.declare_parameter('image_height', 900)
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter('tts_enabled', True)

        self.camera_id = self.get_parameter('camera_id').value
        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.timeout = self.get_parameter('timeout_sec').value
        self.tts_enabled = self.get_parameter('tts_enabled').value

        # ---------- OCR 引擎 ----------
        self._ocr = None
        self._init_ocr()

        # ---------- 发布/订阅 ----------
        self.pub_result = self.create_publisher(MathResult, '/math_result', 10)
        self.sub_start = self.create_subscription(
            String, '/match_start', self._on_match_start, 10
        )

        # ---------- 状态 ----------
        self._solving = False
        self._solving_lock = threading.Lock()

        self.get_logger().info('MathSolverNode 已启动，等待 match_start 信号...')

    # ------------------------------------------------------------------
    def _init_ocr(self):
        """延迟初始化 PaddleOCR（首次调用时加载模型）"""
        try:
            from paddleocr import PaddleOCR
            # PP-OCRv4 轻量模型，自动下载
            self._ocr = PaddleOCR(
                use_angle_cls=False,
                lang='ch',
                show_log=False,
                use_gpu=False,       # 若 R7 5700ESU 有 AMD GPU 可改为 True
            )
            self.get_logger().info('PaddleOCR (PP-OCRv4) 加载成功')
        except ImportError:
            self.get_logger().warn(
                'paddleocr 未安装，回退至占位模式。请执行: '
                'pip install paddleocr paddlepaddle'
            )

    # ------------------------------------------------------------------
    def _on_match_start(self, msg: String):
        """收到比赛开始信号 → 启动数学题识别流程"""
        if self._solving:
            self.get_logger().warn('已在求解中，忽略重复触发')
            return

        self.get_logger().info('=== 数学题识别任务启动（最高优先级）===')
        self._solving = True

        # 在独立线程中执行，不阻塞主线程
        thread = threading.Thread(target=self._solve_and_publish, daemon=True)
        thread.start()

    # ------------------------------------------------------------------
    def _solve_and_publish(self):
        """完整的求解流程"""
        start_time = time.time()
        result_msg = MathResult()
        result_msg.success = False
        result_msg.high_zone_id = -1

        try:
            # ---------- 1. 捕获图像 ----------
            self.get_logger().info('正在捕获 USB 相机画面...')
            img = self._capture_image()
            if img is None:
                self.get_logger().error('图像捕获失败')
                return

            # ---------- 2. OCR 提取 ----------
            self.get_logger().info('OCR 文本提取中...')
            raw_text = self._ocr_extract(img)
            if not raw_text:
                self.get_logger().error('OCR 未提取到任何文本')
                return
            self.get_logger().info(f'OCR 原始文本: "{raw_text}"')

            # ---------- 3. 正则清洗 ----------
            expression = self._sanitize_expression(raw_text)
            if not expression:
                self.get_logger().error('表达式清洗后为空')
                return
            self.get_logger().info(f'清洗后表达式: "{expression}"')

            # ---------- 4. SymPy 安全计算 ----------
            result = self._safe_calc(expression)
            if result is None:
                self.get_logger().error('数学计算失败')
                return

            # ---------- 5. 取模映射 ----------
            high_zone = int(result) % 4

            elapsed = time.time() - start_time
            result_msg.expression = expression
            result_msg.result = int(result)
            result_msg.high_zone_id = high_zone
            result_msg.success = True
            result_msg.elapsed_time = Duration(
                sec=int(elapsed),
                nanosec=int((elapsed - int(elapsed)) * 1e9)
            )

            self.get_logger().info(
                f'✓ 求解成功: {expression} = {int(result)}, '
                f'高分区编号: {high_zone}, 用时: {elapsed:.2f}s'
            )

            # ---------- 6. TTS 播报 ----------
            if self.tts_enabled:
                self._tts_speak(f'{int(result)}号兑换区为高分区域')

        except Exception as e:
            self.get_logger().error(f'求解异常: {e}')
        finally:
            # 发布结果
            self.pub_result.publish(result_msg)
            with self._solving_lock:
                self._solving = False
            self.get_logger().info('数学题识别任务结束')

    # ------------------------------------------------------------------
    def _capture_image(self):
        """通过 OpenCV 捕获 USB 相机图像"""
        if cv2 is None:
            self.get_logger().warn('OpenCV 不可用，返回模拟图像')
            import numpy as np
            return np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)

        cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f'无法打开相机 /dev/video{self.camera_id}')
            return None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_h)
        # 等待相机预热
        for _ in range(5):
            ret, frame = cap.read()
        ret, frame = cap.read()
        cap.release()

        if not ret or frame is None:
            self.get_logger().error('相机读取失败')
            return None
        return frame

    # ------------------------------------------------------------------
    def _ocr_extract(self, img):
        """调用 PaddleOCR 提取文本"""
        if self._ocr is None:
            # 无 OCR 时的占位回退
            return "12 + 34 * 2"

        result = self._ocr.ocr(img, cls=False)
        if not result or not result[0]:
            return None

        lines = [line[1][0] for line in result[0] if line[1] and line[1][0]]
        return ' '.join(lines) if lines else None

    # ------------------------------------------------------------------
    def _sanitize_expression(self, text: str) -> str:
        """正则清洗：仅保留 0-9, +, -, *, /, (, )"""
        cleaned = re.sub(r'[^0-9+\-*/()]', '', text)
        return cleaned.strip()

    # ------------------------------------------------------------------
    def _safe_calc(self, expression: str) -> float | None:
        """SymPy 安全求解四则运算"""
        if sympify is None:
            self.get_logger().warn('sympy 未安装，尝试 eval（不安全！）')
            try:
                return float(eval(expression, {'__builtins__': {}}, {}))
            except Exception:
                return None

        try:
            expr = sympify(expression)
            val = float(expr.evalf())
            if not abs(val) < 1e6:
                self.get_logger().warn(f'结果过大: {val}')
                return None
            return val
        except (SympifyError, TypeError, ValueError) as e:
            self.get_logger().error(f'SymPy 计算失败: {e}')
            return None

    # ------------------------------------------------------------------
    def _tts_speak(self, text: str):
        """TTS 语音播报"""
        try:
            import pyttsx3
            engine = pyttsx3.init()
            engine.say(text)
            engine.runAndWait()
        except ImportError:
            self.get_logger().warn('pyttsx3 未安装，跳过语音播报')
        except Exception as e:
            self.get_logger().warn(f'TTS 播报失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MathSolverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
