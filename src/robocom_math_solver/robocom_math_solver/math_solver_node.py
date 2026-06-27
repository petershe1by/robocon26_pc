#!/usr/bin/env python3
"""math_solver_node.py - 数学题识别节点（最高优先级任务）"""

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
    def __init__(self):
        super().__init__("math_solver_node")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("image_width", 1600)
        self.declare_parameter("image_height", 900)
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("tts_enabled", True)

        self.camera_id = self.get_parameter("camera_id").value
        self.img_w = self.get_parameter("image_width").value
        self.img_h = self.get_parameter("image_height").value
        self.tts_enabled = self.get_parameter("tts_enabled").value

        self._ocr = None
        self._init_ocr()

        self.pub_result = self.create_publisher(MathResult, "/math_result", 10)
        self.create_subscription(String, "/match_start", self._on_match_start, 10)

        self._solving = False
        self._solving_lock = threading.Lock()
        self.get_logger().info("MathSolverNode 已启动")

    def _init_ocr(self):
        try:
            from paddleocr import PaddleOCR
            self._ocr = PaddleOCR(use_angle_cls=False, lang="ch", show_log=False, use_gpu=False)
            self.get_logger().info("PaddleOCR (PP-OCRv4) 加载成功")
        except ImportError:
            self.get_logger().warn("paddleocr 未安装，使用模拟模式")

    def _on_match_start(self, msg: String):
        if self._solving:
            return
        self.get_logger().info("=== 数学题识别启动 ===")
        self._solving = True
        threading.Thread(target=self._solve_and_publish, daemon=True).start()

    def _solve_and_publish(self):
        start_time = time.time()
        result_msg = MathResult()
        result_msg.success = False
        result_msg.high_zone_id = -1
        try:
            img = self._capture_image()
            if img is None:
                return
            raw_text = self._ocr_extract(img)
            if not raw_text:
                return
            expression = self._sanitize_expression(raw_text)
            if not expression:
                return
            result = self._safe_calc(expression)
            if result is None:
                return

            high_zone = int(result) % 4
            elapsed = time.time() - start_time
            result_msg.expression = expression
            result_msg.result = int(result)
            result_msg.high_zone_id = high_zone
            result_msg.success = True
            result_msg.elapsed_time = Duration(sec=int(elapsed), nanosec=int((elapsed % 1) * 1e9))

            self.get_logger().info(f"求解成功: {expression} = {int(result)}, 高分区: {high_zone}, 用时: {elapsed:.1f}s")
            if self.tts_enabled:
                self._tts_speak(f"第{high_zone}号兑换区为高分区域")
        except Exception as e:
            self.get_logger().error(f"求解异常: {e}")
        finally:
            self.pub_result.publish(result_msg)
            with self._solving_lock:
                self._solving = False

    def _capture_image(self):
        if cv2 is None:
            import numpy as np
            return np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_h)
        for _ in range(5):
            cap.read()
        ret, frame = cap.read()
        cap.release()
        return frame if ret else None

    def _ocr_extract(self, img):
        if self._ocr is None:
            return "12 + 34 * 2"
        result = self._ocr.ocr(img, cls=False)
        if not result or not result[0]:
            return None
        lines = [line[1][0] for line in result[0] if line[1] and line[1][0]]
        return " ".join(lines) if lines else None

    def _sanitize_expression(self, text: str) -> str:
        cleaned = re.sub(r"[^0-9+\-*/()]", "", text)
        return cleaned.strip()

    def _safe_calc(self, expression: str) -> float | None:
        if sympify is None:
            try:
                return float(eval(expression, {"__builtins__": {}}, {}))
            except Exception:
                return None
        try:
            expr = sympify(expression)
            val = float(expr.evalf())
            return val if abs(val) < 1e6 else None
        except (SympifyError, TypeError, ValueError):
            return None

    def _tts_speak(self, text: str):
        try:
            import pyttsx3
            engine = pyttsx3.init()
            engine.say(text)
            engine.runAndWait()
        except ImportError:
            pass


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

if __name__ == "__main__":
    main()