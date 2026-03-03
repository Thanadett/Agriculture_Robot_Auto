#!/home/prukubt/yolo_env/bin/python

import math
import collections

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
from pupil_apriltags import Detector


# ════════════════════════════════════════════════════════════════
# STATES
# ════════════════════════════════════════════════════════════════
STATE_SEARCH             = 0
STATE_ALIGN_COARSE       = 1
STATE_CURVE_APPROACH     = 2
STATE_FINAL_ALIGN        = 3
STATE_FINAL_FORWARD      = 4
STATE_BLIND_FORWARD      = 5
STATE_DONE               = 6
STATE_REVERSE            = 7
STATE_RECOVER_TURN       = 8
STATE_REVERSE_REPOSITION = 9
STATE_PARALLEL_ALIGN     = 10   # ★ NEW: หมุนให้หุ่นขนาน tag (tag_yaw → 0)
STATE_SCAN_BACK          = 11   # ★ NEW: สแกนกลับทิศเมื่อหมุนเกินแล้วหา tag ไม่เจอ

STATE_NAME = {
    STATE_SEARCH:             "SEARCH",
    STATE_ALIGN_COARSE:       "ALIGN_COARSE",
    STATE_CURVE_APPROACH:     "CURVE_APPROACH",
    STATE_FINAL_ALIGN:        "FINAL_ALIGN",
    STATE_FINAL_FORWARD:      "FINAL_FORWARD",
    STATE_BLIND_FORWARD:      "BLIND_FORWARD",
    STATE_DONE:               "DONE",
    STATE_REVERSE:            "REVERSE",
    STATE_RECOVER_TURN:       "RECOVER_TURN",
    STATE_REVERSE_REPOSITION: "REPOSITION",
    STATE_PARALLEL_ALIGN:     "PARALLEL_ALIGN",   # ★
    STATE_SCAN_BACK:          "SCAN_BACK",         # ★
}

STATE_COLOR = {
    STATE_SEARCH:             (180, 180, 180),
    STATE_ALIGN_COARSE:       (0,   165, 255),
    STATE_CURVE_APPROACH:     (0,   200,   0),
    STATE_FINAL_ALIGN:        (255, 200,   0),
    STATE_FINAL_FORWARD:      (0,   255,   0),
    STATE_BLIND_FORWARD:      (0,   200, 255),
    STATE_DONE:               (0,   255, 128),
    STATE_REVERSE:            (0,   0,   255),
    STATE_RECOVER_TURN:       (200, 100, 255),
    STATE_REVERSE_REPOSITION: (255,  80,  80),
    STATE_PARALLEL_ALIGN:     (255, 220,  50),     # ★ สีเหลืองทอง
    STATE_SCAN_BACK:          (50,  150, 255),     # ★ สีฟ้า
}

# ── Thresholds ───────────────────────────────────────────────────
YAW_COARSE_THRESH      = 0.40
YAW_COARSE_EXIT_THRESH = 0.12
Z_SWITCH_THRESH        = 0.65
Z_STOP_THRESH          = 0.25

# ── CURVE_APPROACH ───────────────────────────────────────────────
CURVE_KP_BEARING       = 0.6
CURVE_W_MAX            = 0.25
CURVE_BEAR_DEAD        = 0.04

# ── X offset target ──────────────────────────────────────────────
# tag อยู่ที่ x ≈ 0.05m เมื่อหุ่นขนาน (วัดได้จริง)
# ใช้ใน FINAL_FORWARD เท่านั้น — CURVE_APPROACH ชี้ตรงหา tag (x=0)
X_TARGET_APPROACH      = 0.05

# ── Stuck / Reverse ──────────────────────────────────────────────
STUCK_CHECK_SEC        = 3.0
STUCK_Z_MIN_DELTA      = 0.03
STUCK_ZONE_M           = 1.50
REVERSE_SPEED          = -0.15
REVERSE_TIME_SEC       = 2.5

# ── REPOSITION ───────────────────────────────────────────────────
REPOSITION_BASE_DEG    = 10.0
REPOSITION_MAX_DEG     = 30.0
REPOSITION_W           = 0.15
REPOSITION_TIMEOUT_SEC = 4.0

# ── RECOVER_TURN ────────────────────────────────────────────────
RECOVER_W              = 0.18
RECOVER_BEAR_THRESH    = 0.15
RECOVER_TIMEOUT_SEC    = 6.0    # เพิ่มจาก 3s → 6s สำหรับ bearing ใหญ่
RECOVER_WAIT_FIRST_SEC = 1.0    # หยุดก่อน 1s เผื่อ tag กลับมาเอง (vibration)

# ── PARALLEL_ALIGN ★ ─────────────────────────────────────────────
# tag_yaw คือมุมที่ tag หันเทียบกับกล้อง
# ถ้า tag_yaw ≈ 0  →  หุ่นขนานกับ tag แล้ว
# เป้าหมาย: หมุนจนกระทั่ง |tag_yaw| < PARA_YAW_DONE_THRESH
PARA_KP             = 0.55       # P-gain สำหรับ tag_yaw error
PARA_W_MAX          = 0.20       # ความเร็วหมุนสูงสุด (rad/s)
PARA_W_MIN          = 0.08       # floor speed (ไม่ให้แผ่วลงจนหยุด)
PARA_YAW_DEAD       = 0.04       # deadband (rad) ≈ 2.3°
PARA_YAW_DONE_THRESH = 0.08      # เงื่อนไขออกจาก state (rad) ≈ 4.6°
PARA_TIMEOUT_SEC    = 8.0        # timeout ป้องกันวนไม่สิ้นสุด

# ── SCAN_BACK ★ ──────────────────────────────────────────────────
# เมื่ออยู่ใน PARALLEL_ALIGN หรือ SEARCH แล้วหมุนจน tag หาย
# ให้หมุนกลับทิศเดิมอย่างช้าๆ เพื่อหา tag คืน
SCAN_BACK_W            = 0.12       # ความเร็วหมุนกลับ (rad/s)
SCAN_BACK_MAX_DEG      = 60.0       # หมุนกลับสูงสุด 60° แล้วถ้าไม่เจอ → SEARCH
SCAN_BACK_TIMEOUT_SEC  = 8.0

# ── Detection ────────────────────────────────────────────────────
TAG_INTERVAL        = 3
TAG_STABLE_REQUIRED = 5
LOST_TIMEOUT_SEC    = 8.0

# ── X11 ──────────────────────────────────────────────────────────
WIN_NAME  = "AprilTag Follower — Debug v9"
WIN_W     = 960
WIN_H     = 540
HIST_LEN  = 200

CAM_W, CAM_H = 480, 270
MAP_W, MAP_H = 240, 270
CHART_W      = WIN_W - CAM_W - MAP_W

MAP_SCALE    = 90
MAP_ROBOT_X  = MAP_W // 2
MAP_ROBOT_Y  = MAP_H - 30

X11_RENDER_EVERY = 2


# ════════════════════════════════════════════════════════════════
# HELPERS
# ════════════════════════════════════════════════════════════════
class PID:
    def __init__(self, kp, ki, kd, out_min=-1.0, out_max=1.0):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.out_min = out_min; self.out_max = out_max
        self._integral = 0.0
        self._prev_err = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0

    def update(self, error, dt=0.05):
        self._integral += error * dt
        derivative      = (error - self._prev_err) / max(dt, 1e-6)
        self._prev_err  = error
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, out))


class EMATag:
    def __init__(self, alpha=0.4, median_window=5):
        self.alpha         = alpha
        self.median_window = median_window
        self._xb   = collections.deque(maxlen=median_window)
        self._zb   = collections.deque(maxlen=median_window)
        self._yawb = collections.deque(maxlen=median_window)
        self.x = self.z = self.yaw = None

    def update(self, x, z, yaw):
        self._xb.append(x)
        self._zb.append(z)
        self._yawb.append(yaw)
        mx   = float(np.median(self._xb))
        mz   = float(np.median(self._zb))
        myaw = float(np.median(self._yawb))
        if self.x is None:
            self.x, self.z, self.yaw = mx, mz, myaw
        else:
            a = self.alpha
            self.x   = a * mx   + (1 - a) * self.x
            self.z   = a * mz   + (1 - a) * self.z
            self.yaw = a * myaw + (1 - a) * self.yaw

    def reset(self):
        self._xb.clear(); self._zb.clear(); self._yawb.clear()
        self.x = self.z = self.yaw = None

    @property
    def valid(self):
        return len(self._yawb) >= self.median_window and self.x is not None


class RingBuf:
    def __init__(self, n, default=0.0):
        self._d = collections.deque([default] * n, maxlen=n)

    def push(self, v):
        self._d.append(float(v))

    def arr(self):
        return np.array(self._d, dtype=np.float32)


# ════════════════════════════════════════════════════════════════
# X11 DRAWING
# ════════════════════════════════════════════════════════════════
def _strip_chart(canvas, x, y, w, h, buf, label, y_min, y_max, color=(0, 220, 0)):
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (28, 28, 28), -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (65, 65, 65),  1)
    data = buf.arr()
    span = float(y_max - y_min) or 1.0
    if y_min < 0 < y_max:
        zy = y + h - int((0.0 - y_min) / span * h)
        cv2.line(canvas, (x, zy), (x + w, zy), (70, 70, 70), 1)
    n = len(data)
    pts = []
    for i, v in enumerate(data):
        px = x + int(i / max(n - 1, 1) * (w - 1))
        py = y + h - int(np.clip((v - y_min) / span, 0, 1) * (h - 2))
        pts.append((px, py))
    for i in range(1, len(pts)):
        cv2.line(canvas, pts[i - 1], pts[i], color, 1)
    cv2.putText(canvas, f"{label}: {float(data[-1]):.3f}",
        (x + 4, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.36, color, 1)
    cv2.putText(canvas, f"[{y_min:.2f}, {y_max:.2f}]",
        (x + 4, y + h - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.29, (100, 100, 100), 1)


def _top_view(canvas, x_off, y_off, tag_x, tag_z, tag_yaw, state, trail):
    cv2.rectangle(canvas,
        (x_off, y_off), (x_off + MAP_W, y_off + MAP_H), (18, 18, 18), -1)
    cv2.rectangle(canvas,
        (x_off, y_off), (x_off + MAP_W, y_off + MAP_H), (55, 55, 55),  1)
    rx = x_off + MAP_ROBOT_X
    ry = y_off + MAP_ROBOT_Y
    for zm in np.arange(0.0, 2.5, 0.5):
        gy = ry - int(zm * MAP_SCALE)
        if y_off < gy < y_off + MAP_H:
            cv2.line(canvas, (x_off, gy), (x_off + MAP_W, gy), (38, 38, 38), 1)
            cv2.putText(canvas, f"{zm:.1f}m",
                (x_off + 4, gy - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (70, 70, 70), 1)
    for i in range(1, len(trail)):
        cv2.line(canvas, trail[i - 1], trail[i], (50, 90, 50), 1)
    col = STATE_COLOR.get(state, (200, 200, 200))
    if tag_z is not None:
        tx = rx + int(tag_x * MAP_SCALE)
        ty = ry - int(tag_z * MAP_SCALE)
        cv2.circle(canvas, (tx, ty), 8, col, -1)
        ax = tx + int(math.sin(tag_yaw) * 24)
        ay = ty - int(math.cos(tag_yaw) * 24)
        cv2.arrowedLine(canvas, (tx, ty), (ax, ay), (255, 220, 0), 2, tipLength=0.35)
        cv2.line(canvas, (rx, ry), (tx, ty), (55, 55, 55), 1)
        cv2.putText(canvas, "TAG",
            (tx + 10, ty - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
    cv2.circle(canvas, (rx, ry), 8, (0, 180, 255), -1)
    cv2.arrowedLine(canvas, (rx, ry), (rx, ry - 22), (0, 220, 255), 2, tipLength=0.35)
    cv2.putText(canvas, "TOP VIEW",
        (x_off + 4, y_off + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (110, 110, 110), 1)


# ════════════════════════════════════════════════════════════════
# MAIN NODE
# ════════════════════════════════════════════════════════════════
class AprilTagFollower(Node):

    def __init__(self):
        super().__init__('apriltag_follower')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('camera_id',     0)
        self.declare_parameter('image_width',   640)
        self.declare_parameter('image_height',  480)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('use_x11_debug', True)
        self.declare_parameter('invert_yaw',    False)

        # Camera intrinsics (calibrated at 640×480)
        self.declare_parameter('fx',      651.50491737)
        self.declare_parameter('fy',      650.39077601)
        self.declare_parameter('cx',      320.62707882)
        self.declare_parameter('cy',      236.91812436)
        self.declare_parameter('dist_k1',  0.21581633)
        self.declare_parameter('dist_k2', -1.09508649)
        self.declare_parameter('dist_p1', -0.00213472)
        self.declare_parameter('dist_p2',  0.00169510)
        self.declare_parameter('dist_k3',  1.64003200)
        self.declare_parameter('tag_size', 0.042)

        self.declare_parameter('max_linear',  0.30)
        self.declare_parameter('max_angular', 0.60)

        self.declare_parameter('kp_yaw', 0.8);  self.declare_parameter('ki_yaw', 0.00);  self.declare_parameter('kd_yaw', 0.15)
        self.declare_parameter('kp_z',   0.8);  self.declare_parameter('ki_z',   0.005); self.declare_parameter('kd_z',   0.05)
        self.declare_parameter('kp_x',   0.3);  self.declare_parameter('ki_x',   0.0);   self.declare_parameter('kd_x',   0.03)

        self.declare_parameter('z_target',               Z_STOP_THRESH)
        self.declare_parameter('x_target',               0.0)
        self.declare_parameter('yaw_coarse_thresh',      YAW_COARSE_THRESH)
        self.declare_parameter('yaw_coarse_exit_thresh', YAW_COARSE_EXIT_THRESH)
        self.declare_parameter('z_switch_thresh',        Z_SWITCH_THRESH)
        self.declare_parameter('z_stop_thresh',          Z_STOP_THRESH)
        self.declare_parameter('tag_stable_required',    TAG_STABLE_REQUIRED)
        self.declare_parameter('lost_timeout_sec',       LOST_TIMEOUT_SEC)
        self.declare_parameter('cmd_topic',              '/cmd_vel_pid')

        # ── ★ NEW: Parallel align parameter ──────────────────────────
        self.declare_parameter('enable_parallel_align', True)
        self.declare_parameter('para_yaw_done_thresh', PARA_YAW_DONE_THRESH)
        # sign convention ขึ้นกับการติดตั้งกล้อง
        # จาก log: tag_yaw=+34° แต่หุ่นหมุนขวา (w=-0.2) แล้ว yaw ไม่ลด
        # → แสดงว่าต้องหมุนซ้าย (w=+) เพื่อลด yaw
        # → invert_parallel = True (flip sign)
        self.declare_parameter('invert_parallel', True)

        # ── Read params ──────────────────────────────────────────────
        cam_id             = self.get_parameter('camera_id').value
        self.W             = self.get_parameter('image_width').value
        self.H             = self.get_parameter('image_height').value
        self.use_x11_debug = self.get_parameter('use_x11_debug').value
        self.invert_yaw    = self.get_parameter('invert_yaw').value
        self.max_linear    = self.get_parameter('max_linear').value
        self.max_angular   = self.get_parameter('max_angular').value
        self.z_target      = self.get_parameter('z_target').value
        self.x_target      = self.get_parameter('x_target').value
        self.yaw_coarse_thresh      = self.get_parameter('yaw_coarse_thresh').value
        self.yaw_coarse_exit_thresh = self.get_parameter('yaw_coarse_exit_thresh').value
        self.z_switch_thresh        = self.get_parameter('z_switch_thresh').value
        self.z_stop_thresh          = self.get_parameter('z_stop_thresh').value
        self.tag_stable_required    = self.get_parameter('tag_stable_required').value
        self.lost_timeout           = self.get_parameter('lost_timeout_sec').value
        self.enable_parallel_align  = self.get_parameter('enable_parallel_align').value
        self.para_yaw_done_thresh   = self.get_parameter('para_yaw_done_thresh').value
        self.invert_parallel        = self.get_parameter('invert_parallel').value

        # ── Camera intrinsics: 640×480 ────────────
        self.fx     = self.get_parameter('fx').value
        self.fy     = self.get_parameter('fy').value
        self.cx_cam = self.get_parameter('cx').value
        self.cy_cam = self.get_parameter('cy').value
        self.camera_matrix = np.array(
            [[self.fx, 0, self.cx_cam],
             [0, self.fy, self.cy_cam],
             [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.array([[
            self.get_parameter('dist_k1').value,
            self.get_parameter('dist_k2').value,
            self.get_parameter('dist_p1').value,
            self.get_parameter('dist_p2').value,
            self.get_parameter('dist_k3').value,
        ]], dtype=np.float64)
        self.tag_size = self.get_parameter('tag_size').value
        self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs,
            (self.W, self.H), 1, (self.W, self.H))

        # ── PIDs ──────────────────────────────────────────────────────
        self.pid_yaw = PID(
            self.get_parameter('kp_yaw').value,
            self.get_parameter('ki_yaw').value,
            self.get_parameter('kd_yaw').value,
            out_min=-self.max_angular, out_max=self.max_angular)
        self.pid_z = PID(
            self.get_parameter('kp_z').value,
            self.get_parameter('ki_z').value,
            self.get_parameter('kd_z').value,
            out_min=0.0, out_max=self.max_linear)
        self.pid_x = PID(
            self.get_parameter('kp_x').value,
            self.get_parameter('ki_x').value,
            self.get_parameter('kd_x').value,
            out_min=-math.pi / 4, out_max=math.pi / 4)

        # ── AprilTag detector ─────────────────────────────────────────
        self.at_detector = Detector(
            families="tagStandard52h13",
            nthreads=2,
            quad_decimate=1.5,
            refine_edges=1)

        # ── ROS I/O ───────────────────────────────────────────────────
        _cmd_topic = self.get_parameter('cmd_topic').value
        self.cmd_pub      = self.create_publisher(Twist,            _cmd_topic,  1)
        self.debug_pub    = self.create_publisher(Float32MultiArray, '/vision_debug', 5)
        self.plant_pub    = self.create_publisher(Int32,             '/apriltag/planting_distance', 5)
        self.gap_pub      = self.create_publisher(Int32,             '/apriltag/gap_type',           5)
        self.interval_pub = self.create_publisher(Int32,             '/apriltag/cabbage_interval',   5)
        self.pose_pub     = self.create_publisher(Float32MultiArray, '/apriltag/pose',               1)
        self.bridge = CvBridge()

        self.declare_parameter('ticks_to_meter', 1.0)
        self._ticks_to_m  = self.get_parameter('ticks_to_meter').value
        self._wheel_ticks = [0.0, 0.0, 0.0, 0.0]
        self.wheel_sub = self.create_subscription(
            Float32MultiArray, '/wheel_ticks',
            lambda msg: setattr(self, '_wheel_ticks', list(msg.data)), 5)

        # ── Camera ────────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # ── Runtime state ─────────────────────────────────────────────
        self.state         = STATE_SEARCH
        self.frame_count   = 0
        self.smoother      = EMATag(alpha=0.6, median_window=3)
        self.stable_count  = 0
        self.last_tag_time = None
        self.tag_published = False
        self.last_tag_data = None

        self._state_frame      = 0
        self._state_min_frames = 4

        # Stuck detection
        self._stuck_last_z    = None
        self._stuck_last_time = None
        self._stuck_count     = 0

        # Reverse (timer-based)
        self._reverse_start_time = None

        # REPOSITION
        self._reposition_start_time  = None
        self._reposition_turned_rad  = 0.0
        self._reposition_target_rad  = 0.0
        self._reposition_dir         = 1.0
        self._reposition_last_t      = None

        # RECOVER_TURN
        self._last_known_bearing    = None
        self._last_known_z          = None
        self._recover_start_time    = None
        self._recover_turned_rad    = 0.0
        self._recover_last_cmd_time = None

        # ── ★ PARALLEL_ALIGN state vars ──────────────────────────────
        # ใช้ tag_normal_yaw = atan2(R[0,2], R[2,2]) เป็น error signal
        # ค่านี้ stable ไม่ flip เพราะวัดจาก normal vector ของ tag
        # = 0   → ขนานแล้ว
        # > 0   → tag normal ชี้ขวา → หมุนขวา (angular.z < 0)
        # < 0   → tag normal ชี้ซ้าย → หมุนซ้าย (angular.z > 0)
        self._para_start_time    = None
        self._para_yaw_ema       = None   # EMA smooth บน tag_normal_yaw
        self._PARA_EMA_ALPHA     = 0.30   # smoothing factor
        self._para_entered_zone  = False  # hysteresis: เคยเข้า done-zone แล้วหรือยัง
        self._latest_tag_yaw     = 0.0    # ★ fix: init ก่อนใช้
        self._latest_para_err    = 0.0    # ★ parallel_error (unsigned radians)
        self._miss_count         = 0      # ★ consecutive detection misses

        # ── ★ SCAN_BACK state vars ────────────────────────────────────
        # ใช้เมื่อหมุนหา tag แล้วหาย → หมุนกลับทิศเดิม
        self._scan_back_start_time  = None
        self._scan_back_dir_sign    = 0.0    # ทิศกลับ (ตรงข้ามกับที่เคยหมุน)
        self._scan_back_turned_rad  = 0.0
        self._scan_back_last_t      = None
        # จำทิศล่าสุดที่หมุน (เพื่อส่งให้ SCAN_BACK กลับทิศ)
        self._last_rotate_dir_sign  = 0.0

        if self.use_x11_debug:
            self._x11_init()

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info(
            f"AprilTagFollower v9 | res={self.W}×{self.H} "
            f"| fx={self.fx:.1f} cx={self.cx_cam:.1f} "
            f"| z_stop={self.z_stop_thresh}m "
            f"| stuck_delta={STUCK_Z_MIN_DELTA}m "
            f"| parallel_align={'ON' if self.enable_parallel_align else 'OFF'} "
            f"| invert_parallel={self.invert_parallel} "
            f"| cmd_topic={_cmd_topic}")

    # ════════════════════════════════════════════════════════════════
    # X11 DEBUG
    # ════════════════════════════════════════════════════════════════
    def _x11_init(self):
        n = HIST_LEN
        self._dbg_v   = RingBuf(n)
        self._dbg_w   = RingBuf(n)
        self._dbg_x   = RingBuf(n)
        self._dbg_z   = RingBuf(n, default=2.0)
        self._dbg_yaw = RingBuf(n)
        self._dbg_trail: list[tuple[int, int]] = []
        cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_NAME, WIN_W, WIN_H)
        self.get_logger().info("X11 debug window open  [Q/Esc=quit  C=clear trail]")

    def _x11_push(self, v, w, x, z, yaw):
        self._dbg_v.push(v)
        self._dbg_w.push(w)
        self._dbg_x.push(x   if x   is not None else 0.0)
        self._dbg_z.push(z   if z   is not None else 0.0)
        self._dbg_yaw.push(yaw if yaw is not None else 0.0)
        if x is not None and z is not None:
            tx = MAP_ROBOT_X + int(x * MAP_SCALE)
            ty = MAP_ROBOT_Y - int(z * MAP_SCALE)
            if not self._dbg_trail or self._dbg_trail[-1] != (tx, ty):
                self._dbg_trail.append((tx, ty))
                if len(self._dbg_trail) > HIST_LEN:
                    self._dbg_trail.pop(0)

    def _x11_render(self, frame, cmd, x, z, yaw):
        canvas = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
        col    = STATE_COLOR.get(self.state, (200, 200, 200))

        cam = cv2.resize(frame, (CAM_W, CAM_H))
        cv2.rectangle(cam, (0, 0), (CAM_W, 28), (0, 0, 0), -1)
        cv2.putText(cam, STATE_NAME.get(self.state, "?"),
            (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
        canvas[0:CAM_H, 0:CAM_W] = cam

        trail_abs = [(CAM_W + px, py) for px, py in self._dbg_trail]
        _top_view(canvas, CAM_W, 0,
            x   if x   is not None else 0.0,
            z   if z   is not None else None,
            yaw if yaw is not None else 0.0,
            self.state, trail_abs)

        rx   = CAM_W + MAP_W
        ch_h = (WIN_H - 100) // 5
        charts = [
            (self._dbg_v,   "v  m/s",  -0.05, 0.35, (0,   220,   0)),
            (self._dbg_w,   "ω  r/s",  -0.65, 0.65, (0,   160, 255)),
            (self._dbg_x,   "x  m",    -0.8,  0.8,  (255, 180,   0)),
            (self._dbg_z,   "z  m",     0.0,  2.0,  (0,   220, 220)),
            (self._dbg_yaw, "yaw rad", -1.0,  1.0,  (220,   0, 180)),
        ]
        for i, (buf, lbl, lo, hi, c) in enumerate(charts):
            _strip_chart(canvas, rx, i * ch_h, CHART_W - 2, ch_h - 2,
                         buf, lbl, lo, hi, c)

        bar_y = WIN_H - 100
        cv2.rectangle(canvas, (0, bar_y), (WIN_W, WIN_H), (14, 14, 14), -1)
        cv2.line(canvas, (0, bar_y), (WIN_W, bar_y), (55, 55, 55), 1)
        cv2.rectangle(canvas, (10, bar_y + 6), (200, bar_y + 42), col, -1)
        cv2.putText(canvas, STATE_NAME.get(self.state, "?"),
            (14, bar_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 0), 2)

        xv = x   if x   is not None else float('nan')
        zv = z   if z   is not None else float('nan')
        yv = yaw if yaw is not None else float('nan')
        fields = [
            ("x",       f"{xv:.4f} m"),
            ("z",       f"{zv:.4f} m"),
            ("bearing", f"{math.degrees(yv):.2f}°" if not math.isnan(yv) else "---"),
            ("v",       f"{cmd.linear.x:.4f} m/s"),
            ("ω",       f"{cmd.angular.z:.4f} r/s"),
            ("stuck#",  f"{self._stuck_count}"),
        ]
        gx = 210
        for i, (lbl, val) in enumerate(fields):
            cx2 = (i % 3) * 170 + gx
            cy2 = (i // 3) * 34
            cv2.putText(canvas, lbl,
                (cx2, bar_y + 18 + cy2), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (120, 120, 120), 1)
            cv2.putText(canvas, val,
                (cx2, bar_y + 34 + cy2), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (225, 225, 225), 1)

        cv2.imshow(WIN_NAME, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            raise KeyboardInterrupt
        elif key == ord('c'):
            self._dbg_trail.clear()

    # ════════════════════════════════════════════════════════════════
    # APRILTAG DETECTION
    # ════════════════════════════════════════════════════════════════
    def detect_tag(self, frame):
        undist = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs,
                               None, self.new_camera_matrix)
        gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)
        try:
            detections = self.at_detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(self.fx, self.fy, self.cx_cam, self.cy_cam),
                tag_size=self.tag_size,
            )
        except Exception as e:
            self.get_logger().warn(f"AT detect: {e}")
            return None

        if not detections:
            return None

        best  = max(detections, key=lambda d: cv2.contourArea(d.corners.astype(int)))
        tx, ty, tz = best.pose_t.flatten()
        x_m   = float(tx)
        z_m   = float(tz)

        # ── คำนวณ tag orientation ─────────────────────────────────────
        # pose_R: rotation ของ tag frame เทียบกับ camera frame (ติดกับหุ่น)
        #
        # tag_normal_yaw = atan2(R[0,2], R[2,2])
        # คือมุมที่ normal ของ tag หันสัมพัทธ์กับ Z-axis ของกล้อง
        # ⚠️  เมื่อหุ่นหมุน กล้องก็หมุนด้วย → tag_normal_yaw ไม่เปลี่ยน!
        # ดังนั้นใช้เป็น error signal ใน PARALLEL_ALIGN ไม่ได้โดยตรง
        #
        # วิธีที่ถูกต้องสำหรับ "หุ่นขนาน tag":
        #   เมื่อหุ่นขนาน tag → normal ของ tag ชี้ตรงมาที่กล้อง (nz ≈ 1, nx ≈ 0)
        #   → tx ≈ X_TARGET_APPROACH (lateral offset คงที่ตาม geometry)
        #   → ใช้ tx เป็น proxy: ถ้า tx เคลื่อนเข้าหา target = กำลังขนานขึ้น
        #
        # แต่ยังต้องการ tag_normal_yaw สำหรับ:
        #   1) รู้ว่า "ขนานแล้วหรือยัง" (|tag_normal_yaw| < thresh)
        #   2) parallel contribution ใน CURVE_APPROACH
        R = best.pose_R
        nx  = float(R[0, 2])
        nz  = float(R[2, 2])
        tag_normal_yaw = math.atan2(nx, nz)   # range (-π, π], ≈0 = ขนาน

        # parallel_error: unsigned magnitude [0°..180°], 0° = ขนาน
        cos_a = float(np.clip(nz, -1.0, 1.0))
        parallel_error = math.acos(cos_a)

        # bearing ของ tag ตำแหน่ง (ใช้ navigate ไปหา tag)
        bearing_r = math.atan2(x_m, z_m)

        # ── diagnostic (เฟรมแรกเท่านั้น) ─────────────────────────────
        if self.stable_count == 0:
            self.get_logger().info(
                f"[DIAG] tx={x_m:.3f}m  nz={nz:.3f}  nx={nx:.3f}  "
                f"tag_normal_yaw={math.degrees(tag_normal_yaw):.1f}°  "
                f"parallel_error={math.degrees(parallel_error):.1f}°")

        tag_id = best.tag_id
        ab = tag_id // 1000
        c  = (tag_id // 100) % 10
        de = tag_id % 100
        self.last_tag_data = (ab, c, de, x_m, float(ty), z_m, math.degrees(bearing_r))
        self.last_tag_time = self.get_clock().now()

        corners = best.corners.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                     (255, 0, 255), 2)
        cv2.putText(frame,
            f"TAG:{tag_id} x={x_m:.3f}m z={z_m:.3f}m "
            f"bear={math.degrees(bearing_r):.1f}° "
            f"para={math.degrees(parallel_error):.1f}°",
            (10, self.H - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1)

        if self.stable_count >= self.tag_stable_required and not self.tag_published:
            self.plant_pub.publish(Int32(data=ab))
            self.gap_pub.publish(Int32(data=c))
            self.interval_pub.publish(Int32(data=de))
            pose_msg = Float32MultiArray()
            pose_msg.data = [x_m, float(ty), z_m, math.degrees(bearing_r)]
            self.pose_pub.publish(pose_msg)
            self.tag_published = True
            self.get_logger().info(f"[Tag] Published AB={ab} C={c} DE={de} | z={z_m:.3f}m")

        # return: x, z, bearing (navigate), tag_normal_yaw (parallel sign), parallel_error (magnitude)
        return x_m, z_m, bearing_r, tag_normal_yaw, parallel_error

    # ════════════════════════════════════════════════════════════════
    # STATE MACHINE
    # ════════════════════════════════════════════════════════════════
    def _reset_pids(self):
        self.pid_yaw.reset(); self.pid_z.reset(); self.pid_x.reset()

    def _transition(self, new_state, force=False):
        if new_state == self.state:
            self._state_frame += 1
            return
        if not force:
            holdable = self.state not in (STATE_SEARCH, STATE_DONE,
                                          STATE_BLIND_FORWARD, STATE_RECOVER_TURN,
                                          STATE_REVERSE_REPOSITION,
                                          STATE_PARALLEL_ALIGN, STATE_SCAN_BACK)
            if holdable and self._state_frame < self._state_min_frames:
                return
        self.get_logger().info(
            f"[STATE] {STATE_NAME[self.state]} -> {STATE_NAME[new_state]} "
            f"(held {self._state_frame} frames{'  FORCE' if force else ''})")
        prev_state        = self.state
        self.state        = new_state
        self._state_frame = 0
        self._reset_pids()
        # reset stuck detector on every transition
        self._stuck_last_z    = None
        self._stuck_last_time = None

        if prev_state == STATE_REVERSE and new_state != STATE_REVERSE:
            self._reverse_start_time = None

        if new_state == STATE_REVERSE:
            self._reverse_start_time = self.get_clock().now().nanoseconds / 1e9

        if new_state == STATE_REVERSE_REPOSITION:
            deg = min(REPOSITION_BASE_DEG * self._stuck_count, REPOSITION_MAX_DEG)
            self._reposition_target_rad = math.radians(deg)
            self._reposition_dir = 1.0 if (self._stuck_count % 2 == 1) else -1.0
            self._reposition_start_time = self.get_clock().now().nanoseconds / 1e9
            self._reposition_turned_rad = 0.0
            self._reposition_last_t     = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().warn(
                f"[REPOSITION] stuck#{self._stuck_count} → หมุน {deg:.0f}° "
                f"({'ขวา' if self._reposition_dir > 0 else 'ซ้าย'})")

        if new_state == STATE_RECOVER_TURN:
            self._recover_start_time    = self.get_clock().now().nanoseconds / 1e9
            self._recover_turned_rad    = 0.0
            self._recover_last_cmd_time = self.get_clock().now().nanoseconds / 1e9

        # ── ★ init PARALLEL_ALIGN ────────────────────────────────────
        if new_state == STATE_PARALLEL_ALIGN:
            self._para_start_time   = self.get_clock().now().nanoseconds / 1e9
            self._para_yaw_ema      = None    # reset EMA
            self._para_entered_zone = False   # reset hysteresis

        # ── ★ init SCAN_BACK ─────────────────────────────────────────
        if new_state == STATE_SCAN_BACK:
            self._scan_back_start_time = self.get_clock().now().nanoseconds / 1e9
            self._scan_back_dir_sign   = -self._last_rotate_dir_sign  # กลับทิศ
            self._scan_back_turned_rad = 0.0
            self._scan_back_last_t     = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().warn(
                f"[SCAN_BACK] หมุนกลับ {'ขวา' if self._scan_back_dir_sign > 0 else 'ซ้าย'} "
                f"สูงสุด {SCAN_BACK_MAX_DEG:.0f}°")

    def next_state(self, x, z, yaw, tag_yaw, para_err):
        """
        para_err = parallel_error (unsigned radians, 0=ขนาน)
        tag_yaw  = tag_normal_yaw (signed, ใช้ใน CURVE steering เท่านั้น)
        """
        # ── ★ ตรวจ parallel ก่อนทุก state ──────────────────────────
        # เฉพาะตอนไกล (z > Z_SWITCH_THRESH) เท่านั้น
        # ตอนใกล้ให้ FINAL_FORWARD จัดการ ไม่ต้องหมุน in-place
        if (self.enable_parallel_align
                and z is not None and z > Z_SWITCH_THRESH
                and self.state not in (STATE_FINAL_FORWARD, STATE_DONE,
                                       STATE_REVERSE, STATE_REVERSE_REPOSITION,
                                       STATE_RECOVER_TURN, STATE_SCAN_BACK,
                                       STATE_PARALLEL_ALIGN)):
            THRESH_ENTER = self.para_yaw_done_thresh + 0.04   # 0.12 rad ≈ 6.9°
            if para_err > THRESH_ENTER:
                return STATE_PARALLEL_ALIGN

        # ── navigation logic เดิม ────────────────────────────────────
        if self.state == STATE_ALIGN_COARSE:
            if abs(yaw) > self.yaw_coarse_exit_thresh:
                return STATE_ALIGN_COARSE
        else:
            if abs(yaw) > self.yaw_coarse_thresh:
                return STATE_ALIGN_COARSE

        if   z > self.z_switch_thresh:  return STATE_CURVE_APPROACH
        elif z > self.z_stop_thresh:    return STATE_FINAL_FORWARD
        else:                           return STATE_DONE

    def compute_cmd(self, x, z, yaw, tag_yaw, para_err, cmd):
        s        = self.state
        yaw_used = -yaw if (self.invert_yaw and yaw is not None) else yaw

        # ── ALIGN_COARSE ─────────────────────────────────────────────
        if s == STATE_ALIGN_COARSE:
            cmd.linear.x = 0.0
            YAW_DEADBAND = 0.05
            MIN_W        = 0.20
            KP_ALIGN     = 0.6
            if abs(yaw_used) < YAW_DEADBAND:
                cmd.angular.z = 0.0
            else:
                w = -KP_ALIGN * yaw_used
                w = math.copysign(max(abs(w), MIN_W), w)
                w = max(-self.max_angular, min(self.max_angular, w))
                cmd.angular.z = w
            self.get_logger().info(
                f"[ALIGN] bearing={math.degrees(yaw_used):.1f}° w={cmd.angular.z:.3f}",
                throttle_duration_sec=0.3)

        # ── CURVE_APPROACH ───────────────────────────────────────────
        # หลักการ: เดินตรงเข้าหา tag ก่อน เมื่อใกล้ค่อยแก้ lateral
        #
        # ปัญหาเดิม: X_TARGET_APPROACH ทำให้ bearing เล็กแต่ sign สลับ
        # → หุ่นส่ายซ้ายขวา → tag หลุด frame
        #
        # แก้: ขยาย deadband ตอนไกล, แก้ x เฉพาะตอนใกล้
        elif s == STATE_CURVE_APPROACH:
            # ── deadband ขยายตาม z (ไกล = ใจกว้าง, ใกล้ = แม่นยำ) ──────
            # ตอน z > 1m: deadband 5° ไม่ต้องแก้ lateral เล็กน้อย
            # ตอน z < 0.5m: deadband 2° แก้ละเอียดขึ้น
            bear_dead_z = CURVE_BEAR_DEAD + 0.06 * max(0.0, min(1.0,
                (z - self.z_stop_thresh) / (1.0 - self.z_stop_thresh)))
            # bear_dead_z ≈ 0.04 rad (2°) ใกล้, 0.10 rad (6°) ไกล

            # CURVE ชี้ตรงหา tag (x=0), FINAL_FORWARD จัดการ lateral offset
            x_err     = x   # aim at x=0
            bearing_c = math.atan2(x_err, max(z, 0.1))
            bc_used   = -bearing_c if self.invert_yaw else bearing_c

            # ── KP scale ลดลงตอนไกล เพื่อไม่ให้ overshoot ──────────────
            kp_z_scale = 0.6 + 0.4 * max(0.0, min(1.0,
                (Z_SWITCH_THRESH - z) / Z_SWITCH_THRESH))
            # kp_z_scale ≈ 0.6 ตอนไกล, 1.0 ตอนใกล้ z_stop

            # ── distance-dependent gain ──────────────────────────────────
            z_gain = 1.0 + max(0.0, 1.5 * (Z_SWITCH_THRESH - z))
            z_gain = min(z_gain, 3.0)

            if abs(bc_used) < bear_dead_z:
                w = 0.0
            else:
                w = -CURVE_KP_BEARING * kp_z_scale * z_gain * bc_used
                w = max(-CURVE_W_MAX, min(CURVE_W_MAX, w))

            v_raw = self.pid_z.update(z - self.z_target)
            v_raw = max(v_raw, 0.0)

            # curvature cap เฉพาะตอน bearing ใหญ่จริง ๆ
            BEAR_CAP_THRESH = math.radians(5)
            if abs(bc_used) > BEAR_CAP_THRESH and abs(w) > 1e-3:
                MAX_RADIUS_FAR  = 3.0
                MAX_RADIUS_NEAR = 0.5
                t_z = max(0.0, min(1.0,
                    (z - self.z_stop_thresh) / (Z_SWITCH_THRESH - self.z_stop_thresh)))
                max_radius = MAX_RADIUS_NEAR + (MAX_RADIUS_FAR - MAX_RADIUS_NEAR) * t_z
                cmd.linear.x = min(v_raw, abs(w) * max_radius)
            else:
                cmd.linear.x = v_raw

            cmd.angular.z = w
            cur_r = cmd.linear.x / (abs(w) + 1e-4)
            self.get_logger().info(
                f"[CURVE] x={x:.3f} z={z:.3f}"
                f" bear={math.degrees(bearing_c):.1f}°"
                f" dead={math.degrees(bear_dead_z):.1f}°"
                f" R={cur_r:.2f}m v={cmd.linear.x:.3f} w={cmd.angular.z:.3f}",
                throttle_duration_sec=0.5)

            # ── Stuck detection (ใน CURVE ด้วย) ──────────────────────────
            now = self.get_clock().now().nanoseconds / 1e9
            if z < STUCK_ZONE_M:
                if self._stuck_last_z is None:
                    self._stuck_last_z    = z
                    self._stuck_last_time = now
                elif (now - self._stuck_last_time) > STUCK_CHECK_SEC:
                    if (self._stuck_last_z - z) < STUCK_Z_MIN_DELTA:
                        self._stuck_count += 1
                        self.get_logger().warn(
                            f"[STUCK #{self._stuck_count}] CURVE z={z:.3f}m"
                            f" delta<{STUCK_Z_MIN_DELTA}m → REVERSE")
                        self._transition(STATE_REVERSE)
                    self._stuck_last_z    = z
                    self._stuck_last_time = now
            else:
                self._stuck_last_z    = None
                self._stuck_last_time = None

        # ── FINAL_ALIGN ──────────────────────────────────────────────
        elif s == STATE_FINAL_ALIGN:
            cmd.linear.x  = self.pid_z.update(z - self.z_target)
            cmd.angular.z = 0.0

        # ── FINAL_FORWARD ─────────────────────────────────────────────
        # เดินตรงเข้าหา tag ช้าๆ (z ≤ Z_SWITCH_THRESH)
        # แก้ lateral ด้วย x error (ไม่ใช้ bearing ซึ่ง noise เยอะตอนใกล้)
        elif s == STATE_FINAL_FORWARD:
            v_raw = self.pid_z.update(z - self.z_target)
            MIN_V_FINAL = 0.08
            if v_raw > 0.01:
                cmd.linear.x = max(v_raw, MIN_V_FINAL)
            else:
                cmd.linear.x = 0.0

            # lateral correction: แก้ x ให้เข้า X_TARGET_APPROACH
            x_err_ff = x - X_TARGET_APPROACH
            X_DEAD   = 0.05   # deadband 5cm
            KP_X_FF  = 0.5
            if abs(x_err_ff) > X_DEAD:
                cmd.angular.z = max(-0.15, min(0.15, -KP_X_FF * x_err_ff))
            else:
                cmd.angular.z = 0.0

            self.get_logger().info(
                f"[FINAL] x={x:.3f} z={z:.3f}"
                f" x_err={x_err_ff:+.3f} v={cmd.linear.x:.3f} w={cmd.angular.z:.3f}",
                throttle_duration_sec=0.3)

            now = self.get_clock().now().nanoseconds / 1e9
            if z < STUCK_ZONE_M:
                if self._stuck_last_z is None:
                    self._stuck_last_z    = z
                    self._stuck_last_time = now
                elif (now - self._stuck_last_time) > STUCK_CHECK_SEC:
                    if (self._stuck_last_z - z) < STUCK_Z_MIN_DELTA:
                        self._stuck_count += 1
                        self.get_logger().warn(
                            f"[STUCK #{self._stuck_count}] z={z:.3f}m delta < {STUCK_Z_MIN_DELTA}m → REVERSE")
                        self._transition(STATE_REVERSE)
                    self._stuck_last_z    = z
                    self._stuck_last_time = now
            else:
                self._stuck_last_z    = None
                self._stuck_last_time = None

        # ── ★ PARALLEL_ALIGN ──────────────────────────────────────────
        # เป้า: หมุน in-place จนหุ่นขนาน tag (parallel_error → 0°)
        #
        # ✅ Physics ที่ถูกต้อง:
        #   tag_normal_yaw ไม่เปลี่ยนเมื่อหุ่นหมุน (camera-relative)
        #   แต่ tx (lateral position) จะเปลี่ยนเมื่อหุ่นหมุน:
        #     - tag อยู่ขวา (tx > X_TARGET) → หุ่นต้องหมุนขวา (angular.z < 0)
        #     - tag อยู่ซ้าย (tx < X_TARGET) → หุ่นต้องหมุนซ้าย (angular.z > 0)
        #
        #   parallel_error (arccos) จะลดเมื่อหุ่นขนานขึ้น → ใช้เป็น done condition
        #
        # หลักการ:
        #   error = tx - X_TARGET_APPROACH
        #   หมุนเพื่อลด |error| → tx → X_TARGET_APPROACH
        #   ออกจาก state เมื่อ parallel_error < threshold
        elif s == STATE_PARALLEL_ALIGN:
            cmd.linear.x = 0.0
            now_sec = self.get_clock().now().nanoseconds / 1e9
            elapsed = now_sec - (self._para_start_time or now_sec)

            # timeout guard
            if elapsed > PARA_TIMEOUT_SEC:
                cmd.angular.z = 0.0
                self.get_logger().warn(
                    f"[PARALLEL] timeout {elapsed:.1f}s "
                    f"para_err={math.degrees(para_err):.1f}° → ไปต่อ")
                self._transition(
                    self._next_after_parallel(x, z, yaw), force=True)
                return

            # EMA smooth บน para_err (unsigned, แค่ magnitude สำหรับ done check)
            if self._para_yaw_ema is None:
                self._para_yaw_ema = para_err
            else:
                self._para_yaw_ema = (self._PARA_EMA_ALPHA * para_err
                                      + (1.0 - self._PARA_EMA_ALPHA) * self._para_yaw_ema)

            # ── Done condition ────────────────────────────────────────
            PARA_THRESH_ENTER = self.para_yaw_done_thresh          # 0.08 rad ≈ 4.6°
            PARA_THRESH_EXIT  = self.para_yaw_done_thresh + 0.04   # 0.12 rad ≈ 6.9°

            if self._para_yaw_ema < PARA_THRESH_ENTER:
                self._para_entered_zone = True

            if self._para_entered_zone and self._para_yaw_ema < PARA_THRESH_EXIT:
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f"[PARALLEL] ✅ done  para_err_ema={math.degrees(self._para_yaw_ema):.1f}° → ไปต่อ")
                self._transition(
                    self._next_after_parallel(x, z, yaw), force=True)
                return

            # ── Steering: ใช้ tx error ────────────────────────────────
            # tx > X_TARGET → tag ยังอยู่ขวาเกิน → หมุนขวา (angular.z < 0)
            # tx < X_TARGET → tag อยู่ซ้ายเกิน  → หมุนซ้าย (angular.z > 0)
            tx_err = x - X_TARGET_APPROACH   # signed: + = tag ยังขวาเกิน

            if abs(tx_err) < 0.01:   # deadband 1cm
                cmd.angular.z = 0.0
            else:
                # angular.z < 0 หมุนขวา (ลด tx ที่เป็น +)
                sign_inv = 1.0 if self.invert_parallel else -1.0
                w = sign_inv * PARA_KP * tx_err
                w = math.copysign(max(abs(w), PARA_W_MIN), w)
                w = max(-PARA_W_MAX, min(PARA_W_MAX, w))
                cmd.angular.z = w
                self._last_rotate_dir_sign = math.copysign(1.0, w)

            self.get_logger().info(
                f"[PARALLEL] tx={x:.3f}m tx_err={x - X_TARGET_APPROACH:+.3f}m"
                f" para={math.degrees(para_err):.1f}°(ema={math.degrees(self._para_yaw_ema):.1f}°)"
                f" w={cmd.angular.z:+.3f} t={elapsed:.1f}s",
                throttle_duration_sec=0.3)

        # ── ★ SCAN_BACK ───────────────────────────────────────────────
        # หมุนกลับทิศเดิมช้าๆ เพื่อหา tag คืน
        elif s == STATE_SCAN_BACK:
            cmd.linear.x = 0.0
            now_sec  = self.get_clock().now().nanoseconds / 1e9
            elapsed  = now_sec - (self._scan_back_start_time or now_sec)
            dt       = now_sec - (self._scan_back_last_t or now_sec)
            self._scan_back_last_t = now_sec

            max_rad = math.radians(SCAN_BACK_MAX_DEG)

            if elapsed > SCAN_BACK_TIMEOUT_SEC or self._scan_back_turned_rad >= max_rad:
                cmd.angular.z = 0.0
                self.get_logger().warn("[SCAN_BACK] สิ้นสุด → SEARCH")
                self._transition(STATE_SEARCH, force=True)
                return

            if self._scan_back_dir_sign == 0.0:
                # fallback กรณีไม่รู้ทิศ
                cmd.angular.z = SCAN_BACK_W
            else:
                cmd.angular.z = self._scan_back_dir_sign * SCAN_BACK_W

            # ★ ใช้ค่าจริงที่ส่งออก (ไม่ใช่ constant) เพื่อความถูกต้อง
            self._scan_back_turned_rad += abs(cmd.angular.z) * max(dt, 0.0)
            self.get_logger().info(
                f"[SCAN_BACK] {math.degrees(self._scan_back_turned_rad):.1f}°"
                f"/{SCAN_BACK_MAX_DEG:.0f}° dir={'R' if self._scan_back_dir_sign > 0 else 'L'}"
                f" t={elapsed:.1f}s",
                throttle_duration_sec=0.4)

        # ── RECOVER_TURN ─────────────────────────────────────────────
        elif s == STATE_RECOVER_TURN:
            cmd.linear.x = 0.0
            now_sec  = self.get_clock().now().nanoseconds / 1e9
            elapsed  = now_sec - (self._recover_start_time or now_sec)

            if elapsed > RECOVER_TIMEOUT_SEC or self._last_known_bearing is None:
                cmd.angular.z = 0.0
                self.get_logger().warn(f"[RECOVER] timeout {elapsed:.1f}s -> SEARCH")
                self._last_known_bearing = None
                self._transition(STATE_SEARCH, force=True)
                return

            # ★ Phase 1: หยุดนิ่ง RECOVER_WAIT_FIRST_SEC วินาทีก่อน
            # เผื่อ tag กลับมาเองจาก vibration (ประหยัดการหมุน)
            if elapsed < RECOVER_WAIT_FIRST_SEC:
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f"[RECOVER] waiting for tag {elapsed:.1f}/{RECOVER_WAIT_FIRST_SEC:.1f}s...",
                    throttle_duration_sec=0.4)
                return

            # ★ Phase 2: หมุนกลับทิศ bearing เดิม
            target_rad = abs(self._last_known_bearing)
            WAIT_SEC   = 1.5

            dt = now_sec - (self._recover_last_cmd_time or now_sec)
            self._recover_last_cmd_time = now_sec

            # นับเวลาหมุนจาก Phase 2 เริ่ม
            turn_elapsed = elapsed - RECOVER_WAIT_FIRST_SEC

            if self._recover_turned_rad < target_rad:
                w = -RECOVER_W * math.copysign(1.0, self._last_known_bearing)
                cmd.angular.z = w
                self._recover_turned_rad += RECOVER_W * max(dt, 0.0)
                self.get_logger().info(
                    f"[RECOVER] turning {math.degrees(self._recover_turned_rad):.1f}°"
                    f"/{math.degrees(target_rad):.1f}° w={w:.2f} t={turn_elapsed:.1f}s",
                    throttle_duration_sec=0.4)
            else:
                cmd.angular.z = 0.0
                wait_elapsed = turn_elapsed - (target_rad / RECOVER_W)
                self.get_logger().info(
                    f"[RECOVER] waiting {wait_elapsed:.1f}/{WAIT_SEC:.1f}s for tag...",
                    throttle_duration_sec=0.5)
                if wait_elapsed > WAIT_SEC:
                    self.get_logger().warn("[RECOVER] tag not found -> SEARCH")
                    self._last_known_bearing = None
                    self._transition(STATE_SEARCH, force=True)

        # ── REVERSE ──────────────────────────────────────────────────
        elif s == STATE_REVERSE:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if self._reverse_start_time is None:
                self._reverse_start_time = now_sec
            elapsed_rev = now_sec - self._reverse_start_time
            if elapsed_rev < REVERSE_TIME_SEC:
                cmd.linear.x  = REVERSE_SPEED
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f"[REVERSE] {elapsed_rev:.1f}s / {REVERSE_TIME_SEC:.1f}s",
                    throttle_duration_sec=0.5)
            else:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self.smoother.reset()
                self.stable_count     = 0
                self.last_tag_time    = None
                self._stuck_last_z    = None
                self._stuck_last_time = None
                self._transition(STATE_REVERSE_REPOSITION, force=True)
                self.get_logger().info("[REVERSE] done → REPOSITION")

        # ── REVERSE_REPOSITION ───────────────────────────────────────
        elif s == STATE_REVERSE_REPOSITION:
            cmd.linear.x = 0.0
            now_sec  = self.get_clock().now().nanoseconds / 1e9
            elapsed  = now_sec - (self._reposition_start_time or now_sec)
            dt       = now_sec - (self._reposition_last_t or now_sec)
            self._reposition_last_t = now_sec

            if elapsed > REPOSITION_TIMEOUT_SEC:
                cmd.angular.z = 0.0
                self.get_logger().warn("[REPOSITION] timeout → SEARCH")
                self._transition(STATE_SEARCH, force=True)
                return

            if self._reposition_turned_rad < self._reposition_target_rad:
                w = self._reposition_dir * REPOSITION_W
                cmd.angular.z = w
                self._reposition_turned_rad += REPOSITION_W * max(dt, 0.0)
                self.get_logger().info(
                    f"[REPOSITION] {math.degrees(self._reposition_turned_rad):.1f}°"
                    f"/{math.degrees(self._reposition_target_rad):.1f}°",
                    throttle_duration_sec=0.4)
            else:
                cmd.angular.z = 0.0
                self.get_logger().info("[REPOSITION] done → SEARCH")
                self._transition(STATE_SEARCH, force=True)

        else:  # SEARCH / DONE / BLIND_FORWARD
            cmd.linear.x = cmd.angular.z = 0.0
            if s == STATE_BLIND_FORWARD:
                self._transition(STATE_DONE)

    def _next_after_parallel(self, x, z, yaw):
        """State ถัดไปหลังจาก PARALLEL_ALIGN เสร็จ (ใช้ navigation logic เดิม)"""
        if self.state == STATE_ALIGN_COARSE:
            if abs(yaw) > self.yaw_coarse_exit_thresh:
                return STATE_ALIGN_COARSE
        else:
            if abs(yaw) > self.yaw_coarse_thresh:
                return STATE_ALIGN_COARSE
        if   z > self.z_switch_thresh:  return STATE_CURVE_APPROACH
        elif z > self.z_stop_thresh:    return STATE_FINAL_FORWARD
        else:                           return STATE_DONE

    # ════════════════════════════════════════════════════════════════
    # CAMERA OVERLAY
    # ════════════════════════════════════════════════════════════════
    def draw_overlay(self, frame, cmd, x, z, yaw, tag_yaw=None):
        col = STATE_COLOR.get(self.state, (255, 255, 255))

        def txt(img, text, pos, scale, color, thick=1):
            cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), thick+4)
            cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color,   thick)

        cv2.rectangle(frame, (0, 0), (self.W, 28), (0, 0, 0), -1)
        txt(frame, STATE_NAME[self.state], (4, 20), 0.50, col, 1)

        if x is not None:
            yaw_deg     = math.degrees(yaw)
            tag_yaw_deg = math.degrees(tag_yaw) if tag_yaw is not None else float('nan')
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 28), (self.W, 78), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
            txt(frame, f"x={x:+.2f}m",             ( 4, 45), 0.38, (  0, 255, 255), 1)
            txt(frame, f"z={z:.2f}m",               (90, 45), 0.38, ( 80, 255,  80), 1)
            txt(frame, f"b={yaw_deg:+.1f}d",       (170, 45), 0.38, (  0, 215, 255), 1)
            txt(frame, f"tyaw={tag_yaw_deg:+.1f}d",(265, 45), 0.38, (255, 220,  50), 1)  # ★
            txt(frame, f"v={cmd.linear.x:+.2f}",   ( 4, 62), 0.36, (230, 230, 230), 1)
            txt(frame, f"w={cmd.angular.z:+.2f}",  (90, 62), 0.36, (  0, 165, 255), 1)
            txt(frame, f"stk={self._stuck_count}",  (190, 62), 0.36, (255, 100, 100), 1)

            bh = self.H - 12
            def z2y(zv, zm=2.0):
                return int(bh * (1 - min(zv, zm) / zm)) + 6
            cv2.line(frame, (self.W - 10, 6), (self.W - 10, bh), (60, 60, 60), 2)
            cv2.circle(frame, (self.W - 10, z2y(z)), 5, col, -1)
            for thresh, tc in [
                (self.z_switch_thresh, (80, 200, 255)),
                (self.z_stop_thresh,   (0,  80, 255)),
            ]:
                ty = z2y(thresh)
                cv2.line(frame, (self.W - 16, ty), (self.W - 4, ty), tc, 1)

    # ════════════════════════════════════════════════════════════════
    # MAIN LOOP
    # ════════════════════════════════════════════════════════════════
    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1
        cmd        = Twist()
        tag_result = None

        is_detection_frame = (self.frame_count % TAG_INTERVAL == 0)
        if is_detection_frame:
            tag_result = self.detect_tag(frame)

        if tag_result is not None:
            # unpack 5 ค่า: x, z, bearing, tag_normal_yaw, parallel_error
            x_raw, z_raw, bearing_raw, tag_yaw_raw, para_err_raw = tag_result
            self.smoother.update(x_raw, z_raw, bearing_raw)
            self._latest_tag_yaw   = tag_yaw_raw
            self._latest_para_err  = para_err_raw
            self.stable_count += 1
            self._miss_count = 0   # ★ reset miss counter เมื่อ detect ได้
            if self.state not in (STATE_RECOVER_TURN, STATE_SCAN_BACK):
                self._last_known_bearing = bearing_raw
                self._last_known_z       = z_raw
            if self.state in (STATE_SEARCH, STATE_ALIGN_COARSE):
                self._stuck_count = 0

            # ── ★ ถ้ากำลัง SCAN_BACK แล้ว tag กลับมา → ออก ───────────
            if self.state == STATE_SCAN_BACK:
                if self.stable_count >= self.tag_stable_required:
                    self.get_logger().info("[SCAN_BACK] พบ tag → PARALLEL_ALIGN")
                    self._transition(STATE_PARALLEL_ALIGN, force=True)

        elif is_detection_frame:
            # ★ ไม่ reset stable_count ทันที — ให้ miss ได้สูงสุด MISS_GRACE ครั้งก่อน reset
            # ป้องกัน vibration ทำให้ tag หลุด 1-2 frame แล้ว stable_count归零
            MISS_GRACE = 5   # ยอมให้ miss detection ได้กี่ frame ติดกัน
            self._miss_count = getattr(self, '_miss_count', 0) + 1
            if self._miss_count > MISS_GRACE:
                self.stable_count = 0

        # lost-tag check
        tag_lost = False
        if self.last_tag_time is not None:
            elapsed  = (self.get_clock().now() - self.last_tag_time).nanoseconds / 1e9
            tag_lost = elapsed > self.lost_timeout

        x = z = yaw = tag_yaw = None

        if self.state == STATE_DONE:
            cmd.linear.x = cmd.angular.z = 0.0

        elif self.state == STATE_RECOVER_TURN:
            if self.smoother.valid and self.stable_count >= self.tag_stable_required:
                self.get_logger().info("[RECOVER] tag found -> CURVE_APPROACH")
                self._transition(STATE_CURVE_APPROACH, force=True)
            else:
                self.compute_cmd(None, None, None, None, 0.0, cmd)

        elif self.state in (STATE_REVERSE, STATE_REVERSE_REPOSITION):
            self.compute_cmd(None, None, None, None, 0.0, cmd)

        # ── ★ SCAN_BACK: tag ยังหาย → หมุนกลับต่อ ──────────────────
        elif self.state == STATE_SCAN_BACK:
            self.compute_cmd(None, None, None, None, 0.0, cmd)

        elif tag_lost or not self.smoother.valid:
            cmd.linear.x = cmd.angular.z = 0.0
            if self.state not in (STATE_SEARCH, STATE_DONE, STATE_RECOVER_TURN,
                                  STATE_REVERSE_REPOSITION, STATE_SCAN_BACK):
                elapsed_since = 0.0
                if self.last_tag_time is not None:
                    elapsed_since = (
                        self.get_clock().now() - self.last_tag_time
                    ).nanoseconds / 1e9

                # ── ★ ถ้า tag หายระหว่าง PARALLEL_ALIGN → SCAN_BACK ──
                if self.state == STATE_PARALLEL_ALIGN:
                    self.get_logger().warn(
                        f"[LOST] {elapsed_since:.1f}s ระหว่าง PARALLEL_ALIGN → SCAN_BACK")
                    self._transition(STATE_SCAN_BACK, force=True)
                else:
                    can_recover = (
                        self._last_known_bearing is not None
                        and self.state in (STATE_CURVE_APPROACH, STATE_FINAL_FORWARD,
                                           STATE_ALIGN_COARSE, STATE_FINAL_ALIGN)
                    )
                    if can_recover:
                        self.get_logger().warn(
                            f"[LOST] {elapsed_since:.1f}s -> RECOVER_TURN "
                            f"(last_bear={math.degrees(self._last_known_bearing):.1f}°)")
                        self._transition(STATE_RECOVER_TURN, force=True)
                    else:
                        self.get_logger().warn(
                            f"[LOST] {elapsed_since:.1f}s -> SEARCH")
                        self._transition(STATE_SEARCH, force=True)
                self.stable_count = 0

        elif self.stable_count < self.tag_stable_required:
            cmd.linear.x = cmd.angular.z = 0.0

        elif self.state == STATE_BLIND_FORWARD:
            self.compute_cmd(None, None, None, None, 0.0, cmd)

        else:
            x, z, yaw = self.smoother.x, self.smoother.z, self.smoother.yaw
            tag_yaw   = getattr(self, '_latest_tag_yaw', 0.0)
            para_err  = getattr(self, '_latest_para_err', 0.0)
            self._transition(self.next_state(x, z, yaw, tag_yaw, para_err))
            self.compute_cmd(x, z, yaw, tag_yaw, para_err, cmd)

        self.draw_overlay(frame, cmd, x, z, yaw, tag_yaw)

        if self.use_x11_debug:
            self._x11_push(cmd.linear.x, cmd.angular.z, x, z, yaw)
            if self.frame_count % X11_RENDER_EVERY == 0:
                self._x11_render(frame, cmd, x, z, yaw)

        self.cmd_pub.publish(cmd)

        dbg = Float32MultiArray()
        dbg.data = [
            float(self.state),
            float(x)       if x       is not None else -999.0,
            float(z)       if z       is not None else -999.0,
            float(yaw)     if yaw     is not None else -999.0,
            float(cmd.linear.x),
            float(cmd.angular.z),
            float(self.stable_count),
            float(self._stuck_count),
            float(tag_yaw) if tag_yaw is not None else -999.0,   # ★ เพิ่ม tag_yaw
        ]
        self.debug_pub.publish(dbg)


# ════════════════════════════════════════════════════════════════
# ENTRY POINT
# ════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.use_x11_debug:
            cv2.destroyAllWindows()
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()