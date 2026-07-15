#!/usr/bin/env python3
"""math_solver_node.py — 数学题识别节点（集成用户 OCR + 防抖 + Edge TTS）"""

import re, time, threading, os, asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration
from robocom_interfaces.msg import MathResult
from std_msgs.msg import String

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

try:
    from paddleocr import PaddleOCR
except ImportError:
    PaddleOCR = None

# ===== 以下参数来自 sizeyunsuan_yuyinbobao.py，请勿修改 =====
SCORE_THRESHOLD = 0.6
OCR_SKIP_FRAMES = 5
REQUIRED_STABLE_COUNT = 5
MATH_PATTERN = re.compile(r"[0-9\+\-\*/\(\)]+")
VOICE_NAME = "zh-CN-XiaoxiaoNeural"
VOICE_RATE = "+10%"
VOICE_VOLUME = "+0%"
TEMP_AUDIO_FILE = "/tmp/robocom_math_voice.mp3"
RESULT_FILE = os.path.expanduser("~/.robocom_math_result.txt")
MOD_FILE = os.path.expanduser("~/.robocom_math_mod.txt")
# ==========================================================


async def _edge_tts_async(text):
    try:
        import edge_tts
        comm = edge_tts.Communicate(text, voice=VOICE_NAME, rate=VOICE_RATE, volume=VOICE_VOLUME)
        await comm.save(TEMP_AUDIO_FILE)
        from playsound import playsound
        playsound(TEMP_AUDIO_FILE)
        if os.path.exists(TEMP_AUDIO_FILE):
            os.remove(TEMP_AUDIO_FILE)
    except Exception:
        pass


def _voice_broadcast(text):
    asyncio.run(_edge_tts_async(text))


def _safe_eval(expr):
    try:
        return eval(expr)
    except Exception:
        return None


class MathSolverNode(Node):
    def __init__(self):
        super().__init__("math_solver_node")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("tts_enabled", True)

        self._cam_id = self.get_parameter("camera_id").value
        self._tts_enabled = self.get_parameter("tts_enabled").value

        # OCR
        self._ocr = None
        self._init_ocr()

        # Pre-solve
        self._pre_solved_result = None
        self._pre_solve_status = self.create_publisher(String, "/pre_solve_status", 10)
        self.create_subscription(String, "/pre_solve_math", self._on_pre_solve, 10)

        self.pub_result = self.create_publisher(MathResult, "/math_result", 10)
        self.create_subscription(String, "/match_start", self._on_match_start, 10)

        self._solving = False
        self._solving_lock = threading.Lock()
        self.get_logger().info("MathSolverNode 已启动")

    def _init_ocr(self):
        if PaddleOCR is None:
            self.get_logger().warn("paddleocr 未安装，使用模拟模式")
            return
        try:
            self._ocr = PaddleOCR(use_angle_cls=False, lang="ch", show_log=False, use_gpu=False)
            self.get_logger().info("PaddleOCR (PP-OCRv4) 加载成功")
        except Exception as e:
            self.get_logger().error(f"PaddleOCR 加载失败: {e}")

    # ---------- 预解 / 一键启动 ----------
    def _on_pre_solve(self, msg):
        if self._solving:
            self._pre_solve_status.publish(String(data="busy"))
            return
        self._pre_solve_status.publish(String(data="solving"))
        self._solving = True
        result = self._solve()
        with self._solving_lock:
            self._solving = False
        if result and result.success:
            self._pre_solved_result = result
            self._pre_solve_status.publish(String(data="done"))
            self.get_logger().info(f"预解成功: {result.expression} = {result.result}, 等待一键启动...")
        else:
            self._pre_solved_result = None
            self._pre_solve_status.publish(String(data="failed"))
            self.get_logger().error("预解失败")

    def _on_match_start(self, msg):
        if self._solving:
            return
        self.get_logger().info("=== 数学题 ===")
        if self._pre_solved_result is not None:
            self.pub_result.publish(self._pre_solved_result)
            r = self._pre_solved_result
            self._pre_solved_result = None
            self._pre_solve_status.publish(String(data="published"))
            self.get_logger().info(f"已发布预解: {r.expression} = {r.result}, 高分区 {r.high_zone_id}")
            return
        self.get_logger().warn("未预解 → 默认高分区 3")
        m = MathResult()
        m.success = True; m.high_zone_id = 3
        m.expression = ""; m.result = 0
        m.elapsed_time = Duration(sec=0, nanosec=0)
        self.pub_result.publish(m)

    # ---------- 核心求解（防抖） ----------
    def _solve(self):
        """连续多帧 OCR 防抖后返回 MathResult"""
        start = time.time()
        stable_key = None
        stable_count = 0
        final_expr = None
        final_val = None
        frame_idx = 0
        max_time = 60.0

        cap = None
        pipeline = None

        try:
            # 开相机
            if rs is not None:
                pipeline = rs.pipeline()
                cfg = rs.config()
                cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                pipeline.start(cfg)
                for _ in range(10):
                    pipeline.wait_for_frames(timeout_ms=1000)
            elif cv2 is not None:
                cap = cv2.VideoCapture(self._cam_id)
        except Exception as e:
            self.get_logger().error(f"相机打开失败: {e}")
            return None

        try:
            while time.time() - start < max_time:
                frame = None
                if pipeline:
                    try:
                        frames = pipeline.wait_for_frames(timeout_ms=3000)
                        cf = frames.get_color_frame()
                        if cf:
                            frame = np.asanyarray(cf.get_data())
                    except Exception:
                        continue
                elif cap and cap.isOpened():
                    ret, f = cap.read()
                    if ret:
                        frame = f
                if frame is None:
                    continue

                frame_idx += 1
                if frame_idx % OCR_SKIP_FRAMES != 0:
                    continue

                # OCR
                raw_text = self._ocr_extract(frame)
                if not raw_text:
                    continue

                # 提取算式
                matches = MATH_PATTERN.findall(raw_text)
                if not matches:
                    continue
                expr = matches[0]

                # 计算
                val = _safe_eval(expr)
                if val is None:
                    continue
                if not (isinstance(val, int) or (isinstance(val, float) and val.is_integer())):
                    continue

                val = int(val)
                key = f"{expr}={val}"

                # 防抖
                if key == stable_key:
                    stable_count += 1
                    if stable_count >= REQUIRED_STABLE_COUNT:
                        final_expr = expr
                        final_val = val
                        break
                else:
                    stable_key = key
                    stable_count = 1

            if final_expr is not None and final_val is not None:
                hi = final_val % 4
                elapsed = time.time() - start
                msg = MathResult()
                msg.expression = final_expr
                msg.result = final_val
                msg.high_zone_id = hi
                msg.success = True
                msg.elapsed_time = Duration(sec=int(elapsed), nanosec=int((elapsed % 1) * 1e9))

                self.get_logger().info(f"求解成功: {final_expr} = {final_val}, 高分区: {hi}, 用时: {elapsed:.1f}s")

                # 写文件
                try:
                    with open(RESULT_FILE, "w") as f:
                        f.write(f"{final_expr}={final_val}")
                    with open(MOD_FILE, "w") as f:
                        f.write(str(hi))
                except Exception:
                    pass

                # TTS
                if self._tts_enabled:
                    threading.Thread(
                        target=_voice_broadcast,
                        args=(f"计算结果为 {final_val}",),
                        daemon=True
                    ).start()

                return msg

            self.get_logger().error(f"求解超时/失败 ({time.time()-start:.0f}s)")
            return None

        finally:
            if pipeline:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            if cap:
                cap.release()

    # ---------- OCR ----------
    def _ocr_extract(self, frame):
        if self._ocr is None:
            return "12+34*2"
        small = cv2.resize(frame, None, fx=0.8, fy=0.8)
        try:
            result = self._ocr.ocr(small, cls=False)
            if not result or not result[0]:
                return None
            texts = []
            for line in result[0]:
                try:
                    txt = line[1][0]
                    score = line[1][1]
                    if score < SCORE_THRESHOLD:
                        continue
                    txt = txt.replace("÷", "/").replace(":", "/").replace("：", "/")
                    txt = txt.replace("x", "*").replace("X", "*").replace("×", "*")
                    texts.append(txt)
                except Exception:
                    continue
            return " ".join(texts) if texts else None
        except Exception:
            return None

    # ---------- 兼容旧接口 ----------
    def _solve_and_publish(self):
        result = self._solve()
        with self._solving_lock:
            self._solving = False
        if result:
            self.pub_result.publish(result)


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
