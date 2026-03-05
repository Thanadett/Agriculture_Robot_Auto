#!/usr/bin/env python3
"""
Laptop AprilTag Visual Servo — ROS2  (with YOLO guard + Tag ID Lock)
============================================================
รับภาพจาก Pi ผ่าน /camera/image_raw/compressed
ประมวลผล AprilTag + YOLO guard บน Laptop แล้วส่ง cmd_vel กลับ Pi

YOLO Guard Logic (ใช้เฉพาะตอนหา tag ครั้งแรก):
  • รัน YOLO เฉพาะตอนที่ยังไม่มี locked_tag_id
  • ตรวจสอบว่า tag corners อยู่ภายใน YOLO bbox ± YOLO_MARGIN pixels
  • เมื่อ lock แล้ว → ไม่ผ่าน YOLO อีก (ลด noise จาก YOLO bbox)

Tag ID Lock Logic:
  • เมื่อเลือก tag แรกได้ → lock tag_id นั้นไว้
  • เฟรมถัดไปใช้เฉพาะ tag ที่มี id ตรงกัน
  • ถ้าหาย LOCK_LOST_MAX เฟรมติดต่อกัน → ปลด lock แล้วหาใหม่

Topics subscribed:
  /camera/image_raw/compressed  (sensor_msgs/CompressedImage)

Topics published:
  /cmd_vel_pid                  (geometry_msgs/Twist)
  /vision_debug                 (std_msgs/Float32MultiArray)
  /apriltag/planting_distance   (std_msgs/Int32)
  /apriltag/gap_type            (std_msgs/Int32)
  /apriltag/cabbage_interval    (std_msgs/Int32)
  /apriltag/pose                (std_msgs/Float32MultiArray)

Control law (symmetric left/right):
  bearing = atan2(x, z)          # positive = tag is RIGHT
  angular.z = -Kp * bearing      # negative ω turns robot RIGHT  ✓
  angular.z = -Kp * yaw          # same sign convention for yaw align
"""

import math
import collections

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Int32
from cv_bridge import CvBridge

import cv2
import numpy as np
from pupil_apriltags import Detector

# YOLO (ultralytics)
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("[WARN] ultralytics not installed — YOLO guard disabled")


# ════════════════════════════════════════════════════════════════
# States
# ════════════════════════════════════════════════════════════════
STATE_SEARCH    = 0
STATE_APPROACH  = 1
STATE_ALIGN     = 2
STATE_FORWARD   = 3
STATE_DONE      = 4
STATE_SCAN_BACK = 5
STATE_REVERSE   = 6

STATE_NAME = {
    STATE_SEARCH:    "SEARCH",
    STATE_APPROACH:  "APPROACH",
    STATE_ALIGN:     "ALIGN",
    STATE_FORWARD:   "FORWARD",
    STATE_DONE:      "DONE",
    STATE_SCAN_BACK: "SCAN_BACK",
    STATE_REVERSE:   "REVERSE",
}

STATE_COLOR = {
    STATE_SEARCH:    (150, 150, 150),
    STATE_APPROACH:  (0,   210,   0),
    STATE_ALIGN:     (255, 200,  30),
    STATE_FORWARD:   (0,   255, 120),
    STATE_DONE:      (0,   255, 200),
    STATE_SCAN_BACK: (60,  160, 255),
    STATE_REVERSE:   (30,   30, 255),
}

# ════════════════════════════════════════════════════════════════
# Tuning constants
# ════════════════════════════════════════════════════════════════

# Distance zones (metres)
Z_ALIGN   = 0.58
Z_FORWARD = 0.40
Z_STOP    = 0.25

# ── APPROACH: bearing PID ────────────────────────────────────────
APPROACH_KP     = 1.00
APPROACH_KI     = 0.00
APPROACH_KD     = 0.20
APPROACH_W_MAX  = 0.30
APPROACH_V_BASE = 0.18
APPROACH_V_MIN  = 0.04

# ── ALIGN: yaw PID ───────────────────────────────────────────────
ALIGN_KP       = 0.80
ALIGN_KI       = 0.00
ALIGN_KD       = 0.25
ALIGN_W_MAX    = 0.30
ALIGN_V        = 0.08
ALIGN_YAW_DEAD = math.radians(2.5)

DOCK_X       = 0.046
DOCK_BEARING = math.radians(7.2)
DOCK_YAW_TOL = math.radians(2.5)

# ── FORWARD ───────────────────────────────────────────────────────
FORWARD_V = 0.10

# ── SEARCH ───────────────────────────────────────────────────────
SEARCH_W = 0.20

# ── SCAN_BACK ────────────────────────────────────────────────────
SCAN_BACK_W       = 0.18
SCAN_BACK_MAX_DEG = 90.0
SCAN_BACK_TIMEOUT = 10.0

# ── REVERSE ──────────────────────────────────────────────────────
REVERSE_V = -0.15
REVERSE_T = 2.0

# ── Stuck detection ───────────────────────────────────────────────
STUCK_SEC   = 3.0
STUCK_DELTA = 0.03
STUCK_ZONE  = 1.50

# ── Detection ────────────────────────────────────────────────────
DETECT_EVERY    = 1
STABLE_REQUIRED = 4
LOST_TIMEOUT    = 4.0
MISS_GRACE      = 5

# ── Tag ID Lock ───────────────────────────────────────────────────
LOCK_LOST_MAX    = 15   # เฟรมที่ยอมให้หาย tag ที่ lock ไว้ก่อนจะปลด lock
PRELOCK_REQUIRED = 3    # ต้องเห็น tag_id เดิมติดต่อกัน N เฟรม ก่อนจะ lock

# ── YOLO Guard (ใช้เฉพาะตอนหา tag ครั้งแรก) ──────────────────────
YOLO_EVERY          = 10     # รัน YOLO ทุก N เฟรม (ใช้เฉพาะก่อน lock)
YOLO_CONF           = 0.40
YOLO_MARGIN         = 30
YOLO_STALE_MAX      = 30
YOLO_CLASS_NAME     = None

# ════════════════════════════════════════════════════════════════
# X11 window layout
# ════════════════════════════════════════════════════════════════
WIN_W, WIN_H = 1200, 660
HIST_N       = 300

CAM_W, CAM_H = 480, 360
MAP_W, MAP_H = 280, 360
MAP_SCALE    = 100
MAP_CX       = MAP_W // 2
MAP_CY       = MAP_H - 30

CHART_X = CAM_W + MAP_W
CHART_W = WIN_W - CHART_X

INFO_Y = CAM_H
INFO_H = WIN_H - CAM_H

X11_SKIP = 2

_FONT = cv2.FONT_HERSHEY_SIMPLEX


# ════════════════════════════════════════════════════════════════
# PID controller
# ════════════════════════════════════════════════════════════════
class PID:
    def __init__(self, kp, ki, kd, lo=-1.0, hi=1.0):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.lo = lo; self.hi = hi
        self._int = 0.0; self._prev = 0.0
        self.err = self.p = self.i = self.d = self.out = 0.0

    def reset(self):
        self._int = 0.0; self._prev = 0.0
        self.err = self.p = self.i = self.d = self.out = 0.0

    def update(self, error, dt=0.05):
        self._int += error * dt
        deriv      = (error - self._prev) / max(dt, 1e-6)
        self._prev = error
        p   = self.kp * error
        i   = self.ki * self._int
        d   = self.kd * deriv
        out = max(self.lo, min(self.hi, p + i + d))
        self.err = error; self.p = p; self.i = i; self.d = d; self.out = out
        return out


# ════════════════════════════════════════════════════════════════
# Tag smoother
# ════════════════════════════════════════════════════════════════
class TagSmoother:
    def __init__(self, alpha=0.55, window=3):
        self.alpha = alpha; self.window = window
        self._xb = collections.deque(maxlen=window)
        self._zb = collections.deque(maxlen=window)
        self._yb = collections.deque(maxlen=window)
        self.x = self.z = self.yaw = None

    def push(self, x, z, yaw):
        self._xb.append(x); self._zb.append(z); self._yb.append(yaw)
        mx = float(np.median(self._xb))
        mz = float(np.median(self._zb))
        my = float(np.median(self._yb))
        if self.x is None:
            self.x, self.z, self.yaw = mx, mz, my
        else:
            a = self.alpha
            self.x   = a * mx + (1 - a) * self.x
            self.z   = a * mz + (1 - a) * self.z
            self.yaw = a * my + (1 - a) * self.yaw

    def reset(self):
        self._xb.clear(); self._zb.clear(); self._yb.clear()
        self.x = self.z = self.yaw = None

    @property
    def bearing(self):
        if self.x is None:
            return 0.0
        return math.atan2(self.x, max(self.z, 0.05))

    @property
    def valid(self):
        return len(self._zb) >= self.window and self.x is not None


# ════════════════════════════════════════════════════════════════
# Ring buffer
# ════════════════════════════════════════════════════════════════
class Ring:
    def __init__(self, n, v=0.0):
        self._d = collections.deque([float(v)] * n, maxlen=n)

    def push(self, v): self._d.append(float(v))
    def last(self):    return self._d[-1]
    def arr(self):     return np.array(self._d, dtype=np.float32)


# ════════════════════════════════════════════════════════════════
# YOLO Guard  (ใช้เฉพาะตอนยังไม่มี locked_tag_id)
# ════════════════════════════════════════════════════════════════
class YOLOGuard:
    """
    ใช้ YOLO detect bounding box ของ object ที่ AprilTag ติดอยู่
    AprilTag จะ trusted เฉพาะเมื่อ tag อยู่บริเวณ "ขอบบน" ของ YOLO bbox

    หลังจาก Tag ID Lock → YOLOGuard จะถูก bypass โดยสิ้นเชิง
    """

    def __init__(self, model_path: str, conf: float = YOLO_CONF,
                 margin: int = YOLO_MARGIN,
                 stale_max: int = YOLO_STALE_MAX,
                 class_name: str | None = YOLO_CLASS_NAME):
        self.conf      = conf
        self.margin    = margin
        self.stale_max = stale_max
        self.class_name = class_name

        self.bbox      = None
        self.bbox_age  = 0
        self.yolo_ok   = False
        self.disabled  = False

        if not YOLO_AVAILABLE:
            print("[YOLOGuard] ultralytics unavailable — guard disabled")
            self.disabled = True
            return

        try:
            self.model = YOLO(model_path)
            print(f"[YOLOGuard] loaded model: {model_path}")
        except Exception as e:
            print(f"[YOLOGuard] model load failed: {e} — guard disabled")
            self.disabled = True

    def update(self, frame: np.ndarray) -> tuple | None:
        if self.disabled:
            return None

        results = self.model(frame, conf=self.conf, verbose=False)
        best_box  = None
        best_conf = 0.0

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                if self.class_name is not None:
                    cls_name = self.model.names.get(cls_id, '')
                    if cls_name != self.class_name:
                        continue
                if conf > best_conf:
                    best_conf = conf
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    best_box = (int(x1), int(y1), int(x2), int(y2))

        if best_box is not None:
            self.bbox     = best_box
            self.bbox_age = 0
            self.yolo_ok  = True
        else:
            self.bbox_age += 1
            if self.bbox_age > self.stale_max:
                self.bbox    = None
                self.yolo_ok = False

        return self.bbox

    def tick(self):
        if self.disabled:
            return
        self.bbox_age += 1
        if self.bbox_age > self.stale_max:
            self.bbox    = None
            self.yolo_ok = False

    def score_tag_proximity(self, corners: np.ndarray) -> tuple[float, dict]:
        """
        คำนวณ proximity score ของ AprilTag กับ YOLO bbox top-edge

        แทนที่จะถามว่า "tag อยู่ใน bbox ไหม" → ถามว่า "tag อยู่ใกล้ขอบบนแค่ไหน"

        score = abs(centroid_y - y1)  pixels  (น้อย = ใกล้ top-edge = ดี)

        x-range filter (ป้องกัน tag คนละด้านซ้าย/ขวา):
          corners ทุกจุดต้องอยู่ใน [x1 - margin, x2 + margin]
          ถ้าไม่ผ่าน → return inf

        return: (score, info_dict)
        """
        info = {'centroid_y': 0.0, 'top_y': 0.0, 'score': float('inf'), 'reason': ''}

        if self.disabled:
            info['reason'] = 'guard_disabled'
            return 0.0, info          # disabled → score 0 = ผ่านทุก tag, caller เลือกเอง

        if self.bbox is None:
            info['reason'] = 'no_bbox'
            return float('inf'), info

        x1, y1, x2, y2 = self.bbox
        m = self.margin

        # ── x-range filter ──────────────────────────────────────
        for cx, cy in corners:
            if not (x1 - m <= cx <= x2 + m):
                info['reason'] = f'x_out cx={cx:.0f} range=[{x1-m},{x2+m}]'
                return float('inf'), info

        # ── proximity score ──────────────────────────────────────
        centroid_y = float(np.mean(corners[:, 1]))
        score      = abs(centroid_y - y1)

        info['centroid_y'] = centroid_y
        info['top_y']      = float(y1)
        info['score']      = score
        info['reason']     = 'ok'
        return score, info

    def draw(self, frame: np.ndarray, color=(0, 180, 255)):
        if self.bbox is None:
            return
        x1, y1, x2, y2 = self.bbox
        col = color if self.yolo_ok else (80, 80, 80)
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)
        cv2.putText(frame, f"YOLO age={self.bbox_age}",
                    (x1, y1 - 6), _FONT, 0.35, col, 1)
        # top-edge line = target zone (tag ที่ดีควรอยู่ใกล้เส้นนี้)
        cv2.line(frame, (x1 - 6, y1), (x2 + 6, y1), (0, 255, 120), 2)
        cv2.putText(frame, "TOP", (x2 + 8, y1 + 4), _FONT, 0.30, (0, 255, 120), 1)


# ════════════════════════════════════════════════════════════════
# Drawing helpers
# ════════════════════════════════════════════════════════════════
def _txt(canvas, text, pos, scale, color, thick=1):
    cv2.putText(canvas, text, pos, _FONT, scale, (0, 0, 0), thick + 3)
    cv2.putText(canvas, text, pos, _FONT, scale, color,     thick)


def draw_strip(canvas, x, y, w, h, ring, label, lo, hi, color):
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (20, 20, 20), -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (50, 50, 50),  1)
    span = float(hi - lo) or 1.0
    if lo < 0 < hi:
        zy = y + h - int(-lo / span * h)
        cv2.line(canvas, (x, zy), (x + w, zy), (55, 55, 55), 1)
    data = ring.arr(); n = len(data)
    pts = [(x + int(i / max(n - 1, 1) * (w - 1)),
            y + h - int(np.clip((v - lo) / span, 0, 1) * (h - 2)))
           for i, v in enumerate(data)]
    for i in range(1, len(pts)):
        cv2.line(canvas, pts[i - 1], pts[i], color, 1)
    val = ring.last()
    _txt(canvas, f"{label}:{val:+.3f}", (x + 4, y + 13), 0.33, color)
    cv2.putText(canvas, f"[{lo:.2f},{hi:.2f}]",
        (x + 4, y + h - 4), _FONT, 0.27, (70, 70, 70), 1)


def draw_pid_panel(canvas, x, y, w, h, pid, title):
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (15, 15, 15), -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (55, 55, 55),  1)
    _txt(canvas, title, (x + 4, y + 14), 0.36, (200, 200, 200))
    components = [
        ("P", pid.p,   (0, 210, 255)),
        ("I", pid.i,   (80, 255, 80)),
        ("D", pid.d,   (255, 160, 40)),
        ("→", pid.out, (220, 220, 220)),
    ]
    bar_max  = max(abs(v) for _, v, _ in components) or 0.01
    bar_area = w - 70
    row_h    = max((h - 22) // 4, 8)
    for i, (lbl, val, col) in enumerate(components):
        ry  = y + 20 + i * row_h
        bw  = int(abs(val) / bar_max * bar_area)
        bx0 = x + 68
        if val >= 0:
            cv2.rectangle(canvas, (bx0, ry+2), (bx0+bw, ry+row_h-2), col, -1)
        else:
            cv2.rectangle(canvas, (bx0-bw, ry+2), (bx0, ry+row_h-2), col, -1)
        cv2.line(canvas, (bx0, ry+2), (bx0, ry+row_h-2), (80,80,80), 1)
        cv2.putText(canvas, f"{lbl}:{val:+.4f}",
            (x+4, ry+row_h-3), _FONT, 0.29, col, 1)


def draw_topview(canvas, ox, oy, tag_x, tag_z, tag_yaw, state, trail):
    cv2.rectangle(canvas, (ox, oy), (ox+MAP_W, oy+MAP_H), (14,14,14), -1)
    cv2.rectangle(canvas, (ox, oy), (ox+MAP_W, oy+MAP_H), (48,48,48),  1)
    rx = ox + MAP_CX; ry = oy + MAP_CY
    for zm in np.arange(0.2, 2.5, 0.2):
        gy  = ry - int(zm * MAP_SCALE)
        col = (45,45,45) if round(zm*5)%5 else (65,65,65)
        if oy < gy < oy+MAP_H:
            cv2.line(canvas, (ox, gy), (ox+MAP_W, gy), col, 1)
    for zm, col in [(Z_ALIGN,  (60,140,255)),
                    (Z_FORWARD,(60,200, 80)),
                    (Z_STOP,   (30, 30,220))]:
        gy = ry - int(zm*MAP_SCALE)
        if oy < gy < oy+MAP_H:
            cv2.line(canvas, (ox, gy), (ox+MAP_W, gy), col, 1)
            cv2.putText(canvas, f"{zm:.2f}m", (ox+MAP_W-40, gy-2), _FONT, 0.26, col, 1)
    for i in range(1, len(trail)):
        cv2.line(canvas, trail[i-1], trail[i], (35,70,35), 1)
    scol = STATE_COLOR.get(state, (180,180,180))
    if tag_z is not None:
        tx = rx + int(tag_x*MAP_SCALE)
        ty = ry - int(tag_z*MAP_SCALE)
        cv2.circle(canvas, (tx, ty), 9, scol, -1)
        ax = tx + int(math.sin(tag_yaw)*26)
        ay = ty - int(math.cos(tag_yaw)*26)
        cv2.arrowedLine(canvas, (tx,ty), (ax,ay), (255,200,0), 2, tipLength=0.4)
        cv2.line(canvas, (rx,ry), (tx,ty), (50,50,50), 1)
        cv2.putText(canvas, "TAG", (tx+10, ty-4), _FONT, 0.30, scol, 1)
    cv2.circle(canvas, (rx,ry), 8, (0,170,255), -1)
    cv2.arrowedLine(canvas, (rx,ry), (rx, ry-24), (0,220,255), 2, tipLength=0.40)
    cv2.putText(canvas, "MAP", (ox+4, oy+13), _FONT, 0.33, (90,90,90), 1)


# ════════════════════════════════════════════════════════════════
# ROS2 Node
# ════════════════════════════════════════════════════════════════
class AprilTagServo(Node):

    def __init__(self):
        super().__init__('apriltag_servo')
        self._declare_params()
        self._load_params()
        self._build_camera_model()
        self._build_pids()
        self._build_yolo()
        self._build_ros()
        self._init_state()
        if self.use_x11:
            self._x11_init()

        self.get_logger().info(
            f"AprilTag Servo (Laptop) | {self.W}×{self.H}"
            f" | YOLO guard={'ON' if not self.yolo.disabled else 'OFF'} (pre-lock only)"
            f" | Tag ID Lock: LOCK_LOST_MAX={LOCK_LOST_MAX}"
            f" | target_tag_id={'AUTO (YOLO)' if self.target_tag_id < 0 else str(self.target_tag_id)}"
            f" | cmd={self._cmd_topic}")

    # ── Parameters ──────────────────────────────────────────────
    def _declare_params(self):
        d = self.declare_parameter
        d('image_width',   640)
        d('image_height',  480)
        d('use_x11_debug', True)
        d('invert_x',      False)
        d('invert_yaw',    False)
        d('tag_size',      0.042)
        d('max_linear',    0.40)
        d('max_angular',   0.60)
        d('cmd_topic',     '/cmd_vel_pid')
        d('yolo_model',    'best.pt')
        d('target_tag_id', -1)   # -1 = ใช้ YOLO เลือกอัตโนมัติ, >=0 = ระบุ id ตรงๆ
        d('fx', 651.50491737); d('fy', 650.39077601)
        d('cx', 320.62707882); d('cy', 236.91812436)
        d('dist_k1',  0.21581633); d('dist_k2', -1.09508649)
        d('dist_p1', -0.00213472); d('dist_p2',  0.00169510)
        d('dist_k3',  1.64003200)

    def _load_params(self):
        g = self.get_parameter
        self.W           = g('image_width').value
        self.H           = g('image_height').value
        self.use_x11     = g('use_x11_debug').value
        self.invert_x    = g('invert_x').value
        self.invert_yaw  = g('invert_yaw').value
        self.tag_size    = g('tag_size').value
        self.max_linear  = g('max_linear').value
        self.max_angular = g('max_angular').value
        self._cmd_topic   = g('cmd_topic').value
        self._yolo_model  = g('yolo_model').value
        self.target_tag_id = int(g('target_tag_id').value)  # -1 = auto
        self.fx  = g('fx').value;  self.fy = g('fy').value
        self.cx0 = g('cx').value;  self.cy0 = g('cy').value

    def _build_camera_model(self):
        g = self.get_parameter
        self.K = np.array([[self.fx, 0, self.cx0],
                           [0, self.fy, self.cy0],
                           [0, 0, 1]], dtype=np.float64)
        self.D = np.array([[g('dist_k1').value, g('dist_k2').value,
                            g('dist_p1').value, g('dist_p2').value,
                            g('dist_k3').value]], dtype=np.float64)
        self.K_new, _ = cv2.getOptimalNewCameraMatrix(
            self.K, self.D, (self.W, self.H), 1, (self.W, self.H))
        self.detector = Detector(
            families="tagStandard52h13",
            nthreads=2, quad_decimate=1.5, refine_edges=1)

    def _build_pids(self):
        self.pid_b = PID(APPROACH_KP, APPROACH_KI, APPROACH_KD,
                         lo=-APPROACH_W_MAX, hi=APPROACH_W_MAX)
        self.pid_y = PID(ALIGN_KP, ALIGN_KI, ALIGN_KD,
                         lo=-ALIGN_W_MAX, hi=ALIGN_W_MAX)

    def _build_yolo(self):
        self.yolo = YOLOGuard(
            model_path=self._yolo_model,
            conf=YOLO_CONF,
            margin=YOLO_MARGIN,
            stale_max=YOLO_STALE_MAX,
            class_name=YOLO_CLASS_NAME,
        )

    def _build_ros(self):
        self.sub_img = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self._image_callback,
            10)
        self.cmd_pub      = self.create_publisher(Twist,            self._cmd_topic,               1)
        self.dbg_pub      = self.create_publisher(Float32MultiArray, '/vision_debug',               5)
        self.plant_pub    = self.create_publisher(Int32,             '/apriltag/planting_distance',  5)
        self.gap_pub      = self.create_publisher(Int32,             '/apriltag/gap_type',           5)
        self.interval_pub = self.create_publisher(Int32,             '/apriltag/cabbage_interval',   5)
        self.pose_pub     = self.create_publisher(Float32MultiArray, '/apriltag/pose',               1)
        self.bridge = CvBridge()

    def _init_state(self):
        self.state      = STATE_SEARCH
        self.frame_cnt  = 0
        self.tag        = TagSmoother(alpha=0.55, window=3)
        self.n_stable   = 0
        self.n_miss     = 0
        self.last_t     = None
        self.published  = False

        # ── เก็บข้อมูล tag ล่าสุด สำหรับ publish ตอน STATE_DONE ──
        self._last_tag_info = None  # dict: {ab, c, de, x, z, bearing}

        self._sframe    = 0
        self._last_wdir = 1.0

        self._sb_t0     = None
        self._sb_last_t = None
        self._sb_turned = 0.0
        self._sb_dir    = 1.0

        self._rv_t0     = None

        self._stuck_z   = None
        self._stuck_t   = None
        self._stuck_n   = 0

        # YOLO guard frame counter (ใช้เฉพาะก่อน lock)
        self._yolo_frame = 0

        # ── Tag ID Lock ──────────────────────────────────────────
        self.locked_tag_id    = None   # tag_id ที่ lock ไว้  (None = ยังไม่ lock)
        self.lock_lost_frames = 0      # นับเฟรมที่หา locked tag ไม่เจอ

        # ── Pre-lock confidence ───────────────────────────────────
        self.prelock_id    = None  # tag_id ที่กำลัง accumulate
        self.prelock_count = 0     # เห็นติดต่อกันกี่เฟรมแล้ว

    # ── X11 init ────────────────────────────────────────────────
    def _x11_init(self):
        n = HIST_N
        self.rb_bear = Ring(n)
        self.rb_yaw  = Ring(n)
        self.rb_w    = Ring(n)
        self.rb_v    = Ring(n)
        self.rb_z    = Ring(n, 2.0)
        self.rb_x    = Ring(n)
        self.rb_bP   = Ring(n)
        self.rb_bD   = Ring(n)
        self.rb_yP   = Ring(n)
        self.rb_yD   = Ring(n)
        self._trail: list[tuple[int, int]] = []
        cv2.namedWindow("AprilTag Servo", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("AprilTag Servo", WIN_W, WIN_H)
        self.get_logger().info("X11 window open  [Q/Esc=quit  C=clear trail]")

    def _x11_push(self, cmd, x, z, yaw, bearing):
        self.rb_w.push(cmd.angular.z)
        self.rb_v.push(cmd.linear.x)
        self.rb_x.push(x       if x       is not None else 0.0)
        self.rb_z.push(z       if z       is not None else 2.0)
        self.rb_bear.push(bearing if bearing is not None else 0.0)
        self.rb_yaw.push(yaw   if yaw     is not None else 0.0)
        self.rb_bP.push(self.pid_b.p); self.rb_bD.push(self.pid_b.d)
        self.rb_yP.push(self.pid_y.p); self.rb_yD.push(self.pid_y.d)
        if x is not None and z is not None:
            px = CAM_W + MAP_CX + int(x * MAP_SCALE)
            py = MAP_CY - int(z * MAP_SCALE)
            if not self._trail or self._trail[-1] != (px, py):
                self._trail.append((px, py))
                if len(self._trail) > HIST_N:
                    self._trail.pop(0)

    def _x11_draw(self, frame, cmd, x, z, yaw, bearing):
        canvas = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
        scol   = STATE_COLOR.get(self.state, (180, 180, 180))

        cam = cv2.resize(frame, (CAM_W, CAM_H))
        cv2.rectangle(cam, (0, 0), (CAM_W, 26), (0, 0, 0), -1)
        cv2.putText(cam, STATE_NAME.get(self.state, "?"),
            (6, 19), _FONT, 0.60, scol, 2)

        # Lock status overlay
        if self.locked_tag_id is not None:
            lock_txt = f"LOCK: tag {self.locked_tag_id}  lost={self.lock_lost_frames}"
            lock_col = (0, 255, 100)
        else:
            yolo_ok  = self.yolo.yolo_ok
            lock_txt = f"SEARCHING  YOLO:{'OK' if yolo_ok else 'STALE'} age={self.yolo.bbox_age}"
            lock_col = (0, 180, 255) if yolo_ok else (80, 80, 200)
        cv2.putText(cam, lock_txt, (6, CAM_H - 8), _FONT, 0.35, lock_col, 1)

        canvas[0:CAM_H, 0:CAM_W] = cam
        draw_topview(canvas, CAM_W, 0,
            x   if x   is not None else 0.0,
            z   if z   is not None else None,
            yaw if yaw is not None else 0.0,
            self.state, self._trail)

        ib = INFO_Y
        cv2.rectangle(canvas, (0, ib), (CAM_W + MAP_W, WIN_H), (12,12,12), -1)
        cv2.rectangle(canvas, (8, ib+4), (170, ib+34), scol, -1)
        cv2.putText(canvas, STATE_NAME.get(self.state, "?"),
            (12, ib+24), _FONT, 0.46, (0,0,0), 2)

        nan = float('nan')
        xv = x       if x       is not None else nan
        zv = z       if z       is not None else nan
        yv = yaw     if yaw     is not None else nan
        bv = bearing if bearing is not None else nan
        fmtdeg = lambda v: f"{math.degrees(v):+.1f}°" if not math.isnan(v) else "---"

        fields = [
            ("x",      f"{xv:+.3f}m",         (  0,210,210)),
            ("z",      f"{zv:.3f}m",           ( 80,255, 80)),
            ("bearing",fmtdeg(bv),             (  0,190,255)),
            ("yaw",    fmtdeg(yv),             (255,195, 30)),
            ("v",      f"{cmd.linear.x:+.3f}", (200,200,200)),
            ("ω",      f"{cmd.angular.z:+.3f}",(  0,155,255)),
            ("stable", str(self.n_stable),     (140,140,140)),
            ("stuck#", str(self._stuck_n),     (255, 70, 70)),
        ]
        for i, (lbl, val, fc) in enumerate(fields):
            gx = 178 + (i % 4) * 72
            gy = ib + 18 + (i // 4) * 28
            cv2.putText(canvas, lbl, (gx, gy),     _FONT, 0.28, (90,90,90), 1)
            cv2.putText(canvas, val, (gx, gy+14),  _FONT, 0.37, fc,         1)

        gain_y = ib + INFO_H - 14
        cv2.putText(canvas,
            f"BEARING PID  Kp={APPROACH_KP}  Ki={APPROACH_KI}  Kd={APPROACH_KD}",
            (8, gain_y-16), _FONT, 0.30, (80,80,80), 1)
        cv2.putText(canvas,
            f"YAW PID      Kp={ALIGN_KP}  Ki={ALIGN_KI}  Kd={ALIGN_KD}",
            (8, gain_y),    _FONT, 0.30, (80,80,80), 1)

        rx = CHART_X + 2; rw = CHART_W - 4
        sh = (WIN_H - 4) // 7
        strips = [
            (self.rb_z,    "z (m)",         0.0,  2.0, (  0,220,210)),
            (self.rb_x,    "x (m)",        -0.8,  0.8, (255,180,  0)),
            (self.rb_bear, "bearing (rad)",-1.5,  1.5, (  0,190,255)),
            (self.rb_yaw,  "yaw (rad)",    -1.5,  1.5, (255,195, 30)),
            (self.rb_w,    "ω (rad/s)",    -0.8,  0.8, (  0,155,255)),
            (self.rb_v,    "v (m/s)",      -0.05, 0.35,(  0,210,  0)),
        ]
        for i, (ring, lbl, lo, hi, col) in enumerate(strips):
            draw_strip(canvas, rx, i*sh, rw, sh-2, ring, lbl, lo, hi, col)

        pd_y = 6*sh; pd_h = WIN_H - pd_y - 2; pd_hw = (rw-2)//2
        draw_pid_panel(canvas, rx,         pd_y, pd_hw, pd_h,
                       self.pid_b, "BEARING PID (approach)")
        draw_pid_panel(canvas, rx+pd_hw+2, pd_y, pd_hw, pd_h,
                       self.pid_y, "YAW PID (align)")

        cv2.imshow("AprilTag Servo", canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            raise KeyboardInterrupt
        if key == ord('c'):
            self._trail.clear()

    # ── Image callback ──────────────────────────────────────────
    def _image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception as e:
            self.get_logger().warn(f"decode error: {e}", throttle_duration_sec=1.0)
            return
        self._process_frame(frame)

    # ── AprilTag detection with Tag ID Lock + YOLO guard ────────
    def _detect(self, frame):
        """
        Pipeline:
          1. Undistort + detect AprilTag ทุกเฟรม
          2. ถ้ามี locked_tag_id → ใช้เฉพาะ tag id นั้น (ข้าม YOLO)
          3. ถ้ายังไม่มี lock → ใช้ YOLO guard กรอง → lock id ที่เลือก
          4. เลือก tag ที่ใกล้สุด (pose_t[2] = z ต่ำสุด)
        """
        undist = cv2.undistort(frame, self.K, self.D, None, self.K_new)
        gray   = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)

        # ── AprilTag detection (ทุกเฟรม) ────────────────────────
        try:
            dets = self.detector.detect(
                gray, estimate_tag_pose=True,
                camera_params=(self.fx, self.fy, self.cx0, self.cy0),
                tag_size=self.tag_size)
        except Exception as e:
            self.get_logger().warn(f"detect: {e}")
            return None

        if not dets:
            # ── ไม่เจอ tag เลย ──────────────────────────────────
            if self.locked_tag_id is not None:
                self.lock_lost_frames += 1
                if self.lock_lost_frames > LOCK_LOST_MAX:
                    self.get_logger().warn(
                        f"[LOCK] tag {self.locked_tag_id} lost "
                        f"{self.lock_lost_frames} frames → unlock")
                    self.locked_tag_id    = None
                    self.lock_lost_frames = 0
                    self.prelock_id       = None   # reset pre-lock
                    self.prelock_count    = 0
                    # 🔴 force YOLO detect ทันทีตอน unlock
                    self._yolo_frame = 0
                    self.yolo.update(frame)
            return None

        # ════════════════════════════════════════════════════════
        # BRANCH A: มี locked_tag_id → bypass YOLO
        # ════════════════════════════════════════════════════════
        if self.locked_tag_id is not None:
            locked_dets = [d for d in dets if d.tag_id == self.locked_tag_id]

            if locked_dets:
                # เจอ tag ที่ lock ไว้ → reset lost counter
                self.lock_lost_frames = 0
                # เลือกอันใกล้สุด (z ต่ำสุด)
                best = min(locked_dets, key=lambda d: float(d.pose_t[2]))
            else:
                # ไม่เจอ locked tag ในเฟรมนี้
                self.lock_lost_frames += 1
                if self.lock_lost_frames > LOCK_LOST_MAX:
                    self.get_logger().warn(
                        f"[LOCK] tag {self.locked_tag_id} lost "
                        f"{self.lock_lost_frames} frames → unlock")
                    self.locked_tag_id    = None
                    self.lock_lost_frames = 0
                    self.prelock_id       = None   # reset pre-lock
                    self.prelock_count    = 0
                    # 🔴 force YOLO detect ทันทีตอน unlock
                    self._yolo_frame = 0
                    self.yolo.update(frame)

                    # วาด tag อื่นที่เห็นอยู่ (สีเหลือง = ยังไม่ trust)
                    for det in dets:
                        c = det.corners.astype(int)
                        for i in range(4):
                            cv2.line(frame, tuple(c[i]), tuple(c[(i+1)%4]),
                                     (0, 200, 200), 1)
                        cv2.putText(frame, f"WAIT id={det.tag_id}",
                                    (c[0][0], c[0][1] - 4),
                                    _FONT, 0.30, (0, 200, 200), 1)
                return None

        # ════════════════════════════════════════════════════════
        # BRANCH B: ยังไม่มี lock → เลือก tag ตาม mode
        # ════════════════════════════════════════════════════════
        else:
            # ── MODE 1: target_tag_id ระบุไว้ → ข้าม YOLO ─────────
            if self.target_tag_id >= 0:
                target_dets = [d for d in dets if d.tag_id == self.target_tag_id]
                if not target_dets:
                    # วาด tag อื่นที่เห็น (สีเทา = ไม่ใช่ target)
                    for det in dets:
                        c = det.corners.astype(int)
                        for i in range(4):
                            cv2.line(frame, tuple(c[i]), tuple(c[(i+1)%4]),
                                     (60, 60, 60), 1)
                        cv2.putText(frame, f"id={det.tag_id} (not target)",
                                    (c[0][0], c[0][1] - 4), _FONT, 0.28, (60, 60, 60), 1)
                    cv2.putText(frame, f"WAITING target id={self.target_tag_id}",
                                (6, self.H - 30), _FONT, 0.40, (0, 140, 255), 1)
                    return None
                # เลือกอันใกล้สุดในบรรดา target dets
                best = min(target_dets, key=lambda d: float(d.pose_t[2]))

            # ── MODE 2: AUTO → ใช้ YOLO guard กรอง ────────────────
            else:
                # อัพเดท YOLO ทุก YOLO_EVERY เฟรม
                self._yolo_frame += 1
                if self._yolo_frame % YOLO_EVERY == 0:
                    self.yolo.update(frame)
                    self.get_logger().info(
                        f"[YOLO] update: bbox={self.yolo.bbox} ok={self.yolo.yolo_ok}",
                        throttle_duration_sec=0.5)
                else:
                    self.yolo.tick()

                # วาด YOLO bbox (debug — เฉพาะตอน pre-lock)
                self.yolo.draw(frame)

                # คำนวณ score ทุก tag → เลือกอันที่ใกล้ top-edge มากสุด
                scored = []
                for det in dets:
                    corners = det.corners.astype(int)
                    score, info = self.yolo.score_tag_proximity(corners)
                    # วาด score บน frame เพื่อ debug
                    cx0, cy0 = int(corners[0][0]), int(corners[0][1])
                    if score == float('inf'):
                        # ไม่ผ่าน x-filter
                        for i in range(4):
                            cv2.line(frame, tuple(corners[i]),
                                     tuple(corners[(i+1)%4]), (0, 0, 180), 1)
                        cv2.putText(frame, f"x_out {info.get('reason','')}",
                                    (cx0, cy0 - 4), _FONT, 0.28, (0, 60, 220), 1)
                    else:
                        scored.append((score, det))
                        for i in range(4):
                            cv2.line(frame, tuple(corners[i]),
                                     tuple(corners[(i+1)%4]), (0, 180, 80), 1)
                        cv2.putText(frame, f"score={score:.0f}px",
                                    (cx0, cy0 - 4), _FONT, 0.28, (0, 220, 100), 1)

                if not scored:
                    cv2.putText(frame, "NO VALID TAG (x-filter)",
                                (6, self.H - 30), _FONT, 0.40, (0, 60, 220), 1)
                    return None

                # tag ที่ score ต่ำสุด = centroid ใกล้ top-edge มากสุด
                scored.sort(key=lambda t: t[0])
                best = scored[0][1]
                self.get_logger().info(
                    f"[YOLO proximity] best id={best.tag_id}"
                    f" score={scored[0][0]:.1f}px"
                    f" (of {len(scored)} candidates)",
                    throttle_duration_sec=0.3)

            # ── Prelock: ต้องเห็น tag id เดิม PRELOCK_REQUIRED เฟรมติดต่อกัน ──
            if self.prelock_id == best.tag_id:
                self.prelock_count += 1
            else:
                # tag id เปลี่ยน → reset นับใหม่
                if self.prelock_id is not None:
                    self.get_logger().info(
                        f"[PRELOCK] id changed {self.prelock_id}→{best.tag_id}"
                        f"  reset count")
                self.prelock_id    = best.tag_id
                self.prelock_count = 1

            self.get_logger().info(
                f"[PRELOCK] id={self.prelock_id} count={self.prelock_count}"
                f"/{PRELOCK_REQUIRED}",
                throttle_duration_sec=0.2)

            if self.prelock_count < PRELOCK_REQUIRED:
                # ยังไม่ถึง threshold → วาด tag (สีส้ม = กำลัง accumulate) แต่ยังไม่ lock
                corners_pre = best.corners.astype(int)
                for i in range(4):
                    cv2.line(frame, tuple(corners_pre[i]),
                             tuple(corners_pre[(i+1)%4]), (0, 140, 255), 2)
                cv2.putText(frame,
                    f"PRELOCK {self.prelock_count}/{PRELOCK_REQUIRED} id={self.prelock_id}",
                    (corners_pre[0][0], corners_pre[0][1] - 4),
                    _FONT, 0.32, (0, 140, 255), 1)
                return None

            # ── ถึง threshold → lock จริง ────────────────────────
            self.locked_tag_id    = best.tag_id
            self.lock_lost_frames = 0
            self.prelock_id       = None   # ล้างหลัง lock สำเร็จ
            self.prelock_count    = 0
            self.get_logger().info(
                f"[LOCK] ✓ locked tag_id={self.locked_tag_id}"
                f"  z={float(best.pose_t[2]):.3f}m"
                f"  (confirmed {PRELOCK_REQUIRED} frames)")

        # ════════════════════════════════════════════════════════
        # ได้ best detection → คำนวณ pose
        # ════════════════════════════════════════════════════════
        tx, _, tz = best.pose_t.flatten()
        x_m = float(tx) * (-1.0 if self.invert_x else 1.0)
        z_m = float(tz)

        R   = best.pose_R
        nx  = float(R[0, 2])
        nz  = float(R[2, 2])
        yaw = math.atan2(nx, nz) * (-1.0 if self.invert_yaw else 1.0)

        tag_id  = best.tag_id
        corners = best.corners.astype(int)

        # วาด tag ที่ accepted (สีเขียวอมม่วง + แสดง LOCK)
        lock_color = (0, 255, 120)
        for i in range(4):
            cv2.line(frame, tuple(corners[i]),
                     tuple(corners[(i+1)%4]), lock_color, 2)
        bearing = math.atan2(x_m, max(z_m, 0.05))
        cv2.putText(frame,
            f"LOCK:{tag_id} x={x_m:+.3f}m z={z_m:.3f}m"
            f" bear={math.degrees(bearing):+.1f}° yaw={math.degrees(yaw):+.1f}°",
            (6, self.H - 14), _FONT, 0.35, lock_color, 1)

        # ── บันทึก tag info ไว้ publish ตอน STATE_DONE ────────────
        bearing = math.atan2(x_m, max(z_m, 0.05))
        self._last_tag_info = {
            'ab':      tag_id // 1000,
            'c':       (tag_id // 100) % 10,
            'de':      tag_id % 100,
            'x':       x_m,
            'z':       z_m,
            'bearing': bearing,
        }

        self.last_t = self.get_clock().now()
        return x_m, z_m, yaw

    # ── State transition ─────────────────────────────────────────
    def _go(self, new_state, force=False):
        if new_state == self.state:
            self._sframe += 1
            return
        if not force:
            stable_states = (STATE_SEARCH, STATE_DONE, STATE_SCAN_BACK, STATE_REVERSE)
            if self.state not in stable_states and self._sframe < 3:
                return
        self.get_logger().info(
            f"[STATE] {STATE_NAME[self.state]} → {STATE_NAME[new_state]}"
            f"  (held {self._sframe} fr{'  FORCE' if force else ''})")
        self.state   = new_state
        self._sframe = 0
        self._stuck_z = None; self._stuck_t = None
        if new_state in (STATE_APPROACH,):
            self.pid_b.reset()
        if new_state in (STATE_ALIGN,):
            self.pid_y.reset()
        if new_state == STATE_SEARCH:
            self.published        = False
            # ── reset lock เมื่อกลับมา SEARCH ──────────────────
            self.locked_tag_id    = None
            self.lock_lost_frames = 0
            self.get_logger().info("[LOCK] reset (back to SEARCH)")
        if new_state == STATE_DONE and not self.published:
            # ── publish ทุก topic ณ จุดหยุดจริง ─────────────────
            info = self._last_tag_info
            if info is not None:
                # ใช้ z จาก smoother ณ ตอนนี้ (ค่าล่าสุดที่หยุด)
                z_stop   = self.tag.z if self.tag.z is not None else info['z']
                bearing_stop = math.atan2(info['x'], max(z_stop, 0.05))

                self.plant_pub.publish(Int32(data=info['ab']))
                self.gap_pub.publish(Int32(data=info['c']))
                self.interval_pub.publish(Int32(data=info['de']))
                pm = Float32MultiArray()
                pm.data = [float(info['x']), 0.0, float(z_stop),
                           float(math.degrees(bearing_stop))]
                self.pose_pub.publish(pm)
                self.published = True
                self.get_logger().info(
                    f"[DONE] published  AB={info['ab']} C={info['c']} DE={info['de']}"
                    f"  z_stop={z_stop:.3f}m  bearing={math.degrees(bearing_stop):+.1f}°")
        if new_state == STATE_REVERSE:
            self._rv_t0 = self.get_clock().now().nanoseconds / 1e9
        if new_state == STATE_SCAN_BACK:
            self._sb_t0     = self.get_clock().now().nanoseconds / 1e9
            self._sb_last_t = self._sb_t0
            self._sb_turned = 0.0
            self._sb_dir    = -self._last_wdir
            self.get_logger().warn(
                f"[SCAN_BACK] dir={'R' if self._sb_dir > 0 else 'L'}"
                f"  max={SCAN_BACK_MAX_DEG:.0f}°")

    def _next_nav_state(self, z):
        if z <= Z_STOP:    return STATE_DONE
        if z <= Z_FORWARD: return STATE_FORWARD
        if z <= Z_ALIGN:   return STATE_ALIGN
        return STATE_APPROACH

    def _check_stuck(self, z):
        if z > STUCK_ZONE:
            self._stuck_z = None; self._stuck_t = None
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self._stuck_z is None:
            self._stuck_z = z; self._stuck_t = now
            return
        if now - self._stuck_t > STUCK_SEC:
            if self._stuck_z - z < STUCK_DELTA:
                self._stuck_n += 1
                self.get_logger().warn(
                    f"[STUCK #{self._stuck_n}] z={z:.3f}m → REVERSE")
                self._go(STATE_REVERSE, force=True)
            self._stuck_z = z; self._stuck_t = now

    # ── Control laws ─────────────────────────────────────────────
    def _control(self, x, z, yaw, bearing, cmd):
        s = self.state

        if s == STATE_SEARCH:
            cmd.linear.x  = 0.0
            cmd.angular.z = SEARCH_W
            self._last_wdir = 1.0

        elif s == STATE_APPROACH:
            raw_w = self.pid_b.update(bearing)
            cmd.angular.z = -raw_w
            if abs(cmd.angular.z) > 0.01:
                self._last_wdir = math.copysign(1.0, cmd.angular.z)
            align_factor  = max(0.2, math.cos(bearing))
            cmd.linear.x  = APPROACH_V_BASE * align_factor
            self.get_logger().info(
                f"[APPROACH] x={x:+.3f} z={z:.3f}"
                f"  bear={math.degrees(bearing):+.1f}°  cos={align_factor:.2f}"
                f"  P={self.pid_b.p:+.3f} D={self.pid_b.d:+.3f}"
                f"  ω={cmd.angular.z:+.3f} v={cmd.linear.x:.3f}",
                throttle_duration_sec=0.25)
            self._check_stuck(z)

        elif s == STATE_ALIGN:
            if abs(yaw) < ALIGN_YAW_DEAD:
                cmd.angular.z = 0.0
                self.pid_y.reset()
            else:
                raw_w = self.pid_y.update(yaw)
                cmd.angular.z = -raw_w
            if abs(cmd.angular.z) > 0.01:
                self._last_wdir = math.copysign(1.0, cmd.angular.z)
            yaw_factor   = max(0.0, math.cos(yaw))
            cmd.linear.x = ALIGN_V * yaw_factor
            self.get_logger().info(
                f"[ALIGN] yaw={math.degrees(yaw):+.1f}°  cos={yaw_factor:.2f}"
                f"  P={self.pid_y.p:+.3f} D={self.pid_y.d:+.3f}"
                f"  ω={cmd.angular.z:+.3f} v={cmd.linear.x:.3f}",
                throttle_duration_sec=0.25)
            self._check_stuck(z)

        elif s == STATE_FORWARD:
            cmd.linear.x  = FORWARD_V
            cmd.angular.z = 0.0
            self.get_logger().info(
                f"[FORWARD] z={z:.3f}  v={cmd.linear.x:.3f}",
                throttle_duration_sec=0.3)
            self._check_stuck(z)

        elif s == STATE_SCAN_BACK:
            cmd.linear.x = 0.0
            now  = self.get_clock().now().nanoseconds / 1e9
            elap = now - (self._sb_t0 or now)
            dt   = now - (self._sb_last_t or now)
            self._sb_last_t = now
            if elap > SCAN_BACK_TIMEOUT or \
               self._sb_turned >= math.radians(SCAN_BACK_MAX_DEG):
                cmd.angular.z = 0.0
                self.get_logger().warn("[SCAN_BACK] exhausted → SEARCH")
                self._go(STATE_SEARCH, force=True)
                return
            cmd.angular.z    = self._sb_dir * SCAN_BACK_W
            self._sb_turned += abs(cmd.angular.z) * max(dt, 0.0)
            self.get_logger().info(
                f"[SCAN_BACK] {math.degrees(self._sb_turned):.1f}°"
                f"/{SCAN_BACK_MAX_DEG:.0f}° t={elap:.1f}s",
                throttle_duration_sec=0.4)

        elif s == STATE_REVERSE:
            now  = self.get_clock().now().nanoseconds / 1e9
            elap = now - (self._rv_t0 or now)
            if elap < REVERSE_T:
                cmd.linear.x  = REVERSE_V
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f"[REVERSE] {elap:.1f}/{REVERSE_T:.1f}s",
                    throttle_duration_sec=0.5)
            else:
                cmd.linear.x = cmd.angular.z = 0.0
                self.tag.reset()
                self.n_stable = 0
                self.last_t   = None
                self.get_logger().info("[REVERSE] done → SEARCH")
                self._go(STATE_SEARCH, force=True)

        else:  # DONE
            cmd.linear.x = cmd.angular.z = 0.0

    # ── Camera frame overlay ─────────────────────────────────────
    def _overlay(self, frame, cmd, x, z, yaw, bearing):
        col = STATE_COLOR.get(self.state, (255, 255, 255))

        def ftxt(img, t, p, sc, c, th=1):
            cv2.putText(img, t, p, _FONT, sc, (0,0,0), th+4)
            cv2.putText(img, t, p, _FONT, sc, c, th)

        cv2.rectangle(frame, (0, 0), (self.W, 26), (0, 0, 0), -1)
        ftxt(frame, STATE_NAME[self.state], (4, 19), 0.50, col)

        if x is not None:
            ov = frame.copy()
            cv2.rectangle(ov, (0, 26), (self.W, 72), (0, 0, 0), -1)
            cv2.addWeighted(ov, 0.5, frame, 0.5, 0, frame)
            ftxt(frame, f"x={x:+.2f}",                       ( 4, 42), 0.36, (  0,220,220))
            ftxt(frame, f"z={z:.2f}",                         (80, 42), 0.36, ( 80,255, 80))
            ftxt(frame, f"bear={math.degrees(bearing):+.1f}°",(155, 42), 0.36, (  0,190,255))
            ftxt(frame, f"yaw={math.degrees(yaw):+.1f}°",    (285, 42), 0.36, (255,195, 30))
            ftxt(frame, f"v={cmd.linear.x:+.2f}",            ( 4, 60), 0.34, (200,200,200))
            ftxt(frame, f"ω={cmd.angular.z:+.2f}",           (80, 60), 0.34, (  0,155,255))
            bx = self.W - 10; by0 = 6; byh = self.H - 12
            cv2.line(frame, (bx, by0), (bx, by0+byh), (50,50,50), 2)
            zy = by0 + int(byh * (1.0 - min(z, 2.0) / 2.0))
            cv2.circle(frame, (bx, zy), 5, col, -1)
            for zt, tc in [(Z_ALIGN,  (60,140,255)),
                           (Z_FORWARD,(60,200, 80)),
                           (Z_STOP,   (30, 30,220))]:
                ty = by0 + int(byh * (1.0 - zt / 2.0))
                cv2.line(frame, (bx-5, ty), (bx+5, ty), tc, 1)

    # ── Main processing ──────────────────────────────────────────
    def _process_frame(self, frame):
        self.frame_cnt += 1
        cmd = Twist()
        x = z = yaw = bearing = None

        # Detection
        if self.frame_cnt % DETECT_EVERY == 0:
            result = self._detect(frame)
            if result is not None:
                rx, rz, ry = result
                self.tag.push(rx, rz, ry)
                self.n_stable += 1
                self.n_miss    = 0
                if self.state == STATE_SCAN_BACK and \
                   self.n_stable >= STABLE_REQUIRED:
                    self.get_logger().info(
                        f"[SCAN_BACK] tag found x={self.tag.x:+.3f}m → APPROACH")
                    self._go(STATE_APPROACH, force=True)
            else:
                self.n_miss += 1
                if self.n_miss > MISS_GRACE:
                    self.n_stable = 0

        # Lost-tag watchdog
        if self.state in (STATE_APPROACH, STATE_ALIGN, STATE_FORWARD):
            if self.last_t is not None:
                lost = (self.get_clock().now() - self.last_t).nanoseconds / 1e9
                if lost > LOST_TIMEOUT:
                    self.get_logger().warn(f"[LOST] {lost:.1f}s → SCAN_BACK")
                    self._go(STATE_SCAN_BACK, force=True)
                    self.n_stable = 0

        # Execute state
        if self.state == STATE_DONE:
            cmd.linear.x = cmd.angular.z = 0.0

        elif self.state in (STATE_REVERSE, STATE_SCAN_BACK):
            self._control(None, None, None, None, cmd)

        elif self.state == STATE_SEARCH:
            if self.tag.valid and self.n_stable >= STABLE_REQUIRED:
                side = "RIGHT" if self.tag.x > 0 else "LEFT"
                self.get_logger().info(
                    f"[SEARCH] tag found x={self.tag.x:+.3f}m ({side})"
                    f"  bear={math.degrees(self.tag.bearing):+.1f}° → APPROACH")
                self._go(STATE_APPROACH, force=True)
            else:
                self._control(None, None, None, None, cmd)

        elif self.tag.valid and self.n_stable >= STABLE_REQUIRED:
            x, z, yaw = self.tag.x, self.tag.z, self.tag.yaw
            bearing    = self.tag.bearing
            self._go(self._next_nav_state(z))
            self._control(x, z, yaw, bearing, cmd)

        self._overlay(frame, cmd, x, z, yaw, bearing)

        if self.use_x11:
            self._x11_push(cmd, x, z, yaw, bearing)
            if self.frame_cnt % X11_SKIP == 0:
                self._x11_draw(frame, cmd, x, z, yaw, bearing)

        self.cmd_pub.publish(cmd)

        dbg = Float32MultiArray()
        dbg.data = [
            float(self.state),
            float(x)       if x       is not None else -999.0,
            float(z)       if z       is not None else -999.0,
            float(yaw)     if yaw     is not None else -999.0,
            float(cmd.linear.x),
            float(cmd.angular.z),
            float(self.n_stable),
            float(self._stuck_n),
            float(bearing) if bearing is not None else -999.0,
        ]
        self.dbg_pub.publish(dbg)


# ════════════════════════════════════════════════════════════════
# Entry point
# ════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = AprilTagServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.use_x11:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()