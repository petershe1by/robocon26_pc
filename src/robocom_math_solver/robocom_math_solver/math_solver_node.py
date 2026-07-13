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
import numpy as np

try:
    from sympy import sympify, SympifyError
except ImportError:
    sympify = None
    SympifyError = Exception

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
        self._capture_retries = 3
        self.tts_enabled = self.get_parameter("tts_enabled").value

        self._pre_solved_result = None
        self._pre_solve_status = self.create_publisher(String, '/pre_solve_status', 10)
        self.create_subscription(String, '/pre_solve_math', self._on_pre_solve, 10)

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

    def _on_pre_solve(self, msg: String):
        """预解数学题（点击“拍照计算”触发，结果暂不发布）"""
        if self._solving:
            self._pre_solve_status.publish(String(data='busy'))
            return
        self._pre_solve_status.publish(String(data='solving'))
        self._solving = True
        result = self._solve()
        with self._solving_lock:
            self._solving = False
        if result and result.success:
            self._pre_solved_result = result
            self._pre_solve_status.publish(String(data='done'))
            self.get_logger().info(f"预解成功: {result.expression} = {result.result}, 等待一键启动...")
        else:
            self._pre_solved_result = None
            self._pre_solve_status.publish(String(data='failed'))
            self.get_logger().error("预解失败")

    def _on_match_start(self, msg: String):
        if self._solving:
            return
        self.get_logger().info("=== 数学题识别启动 ===")
        # 如果已经预解，直接发布结果
        if self._pre_solved_result is not None:
            self.pub_result.publish(self._pre_solved_result)
            self._pre_solved_result = None
            self._pre_solve_status.publish(String(data='published'))
            self.get_logger().info("已发布预解结果")
            return
        # 没有预解 → 默认返回高分区 3
        self.get_logger().warn("未预解数学题，默认返回高分区 3")
        msg = MathResult()
        msg.success = True
        msg.high_zone_id = 3
        msg.expression = ''
        msg.result = 0
        msg.elapsed_time = Duration(sec=0, nanosec=0)
        self.pub_result.publish(msg)
        self._pre_solve_status.publish(String(data='default_3'))

    def _solve(self):
        """内核求解逻辑：拍照→OCR→SymPy。返回 MathResult 或 None。"""
        start = time.time()
        def _fail():
            msg = MathResult()
            msg.success = False
            msg.high_zone_id = -1
            msg.expression = ''
            msg.result = 0
            msg.elapsed_time = Duration(sec=int(time.time()-start), nanosec=0)
            return msg
        try:
            img = self._capture_image()
            if img is None: return _fail()
            raw = self._ocr_extract(img)
            if not raw: return _fail()
            expr = self._sanitize_expression(raw)
            if not expr: return _fail()
            val = self._safe_calc(expr)
            if val is None: return _fail()
            hi = int(val) % 4
            el = time.time() - start
            msg = MathResult()
            msg.expression = expr; msg.result = int(val)
            msg.high_zone_id = hi; msg.success = True
            msg.elapsed_time = Duration(sec=int(el), nanosec=int((el%1)*1e9))
            self.get_logger().info(f"求解成功: {expr} = {int(val)}, 高分区: {hi}, 用时: {el:.1f}s")
            if self.tts_enabled:
                self._tts_speak(f"第{hi}号兑换区为高分区域")
            return msg
        except Exception as e:
            self.get_logger().error(f"求解异常: {e}")
            return _fail()

    def _solve_and_publish(self):
        result = self._solve()
        with self._solving_lock:
            self._solving = False
        self.pub_result.publish(result)

    def _capture_image(self):
        if cv2 is None:
            return np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        if rs is None:
            self.get_logger().error("pyrealsense2 未安装")
            return None
        try:
            pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, self.img_w, self.img_h, rs.format.bgr8, 30)
            pipeline.start(cfg)
            # 等待帧稳定
            for _ in range(10):
                pipeline.wait_for_frames(timeout_ms=5000)
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                pipeline.stop()
                return None
            frame = np.asanyarray(color_frame.get_data())
            pipeline.stop()
            # 图像预处理：CLAHE + 自适应二值化
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            binary = cv2.adaptiveThreshold(enhanced, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 6)
            return cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            self.get_logger().error(f"拍照失败: {e}")
            return None

    def _ocr_extract(self, img):
        if self._ocr is None:
            return "12 + 34 * 2"
        result = self._ocr.ocr(img, cls=False)
        if not result or not result[0]:
            return None
        lines = [line[1][0] for line in result[0] if line[1] and line[1][0]]
        return " ".join(lines) if lines else None

    def _sanitize_expression(self, raw: str) -> str:
        # 统一数学符号：×→*, ÷→/, −(U+2212)→-, 全角数字→半角
        text = raw.replace('×', '*').replace('÷', '/')
        text = text.replace('−', '-').replace('－', '-')
        text = text.replace('＋', '+').replace('＝', '=')
        # 全角数字/字母转半角
        full_to_half = str.maketrans(
            '０１２３４５６７８９．',
            '0123456789.'
        )
        text = text.translate(full_to_half)
        # 只保留数字、基本运算符和括号
        cleaned = re.sub(r"[^0-9+\-*/()]", "", text)
        # 去掉末尾的等号或多余字符
        cleaned = cleaned.strip('*=')
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
