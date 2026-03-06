#!/home/prukubt/yolo_env/bin/python

import math
import collections

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32
from cv_bridge import CvBridge

import cv2
import numpy as np
from pupil_apriltags import Detector


# ── States ──────────────────────────────────────────────────────
STATE_SEARCH         = 0   # spin to find tag; detect left/right from x
STATE_CURVE_APPROACH = 1   # PID hard-curve toward tag to arrive parallel
STATE_PARALLEL_ALIGN = 2   # rotate in-place until robot is parallel to tag
STATE_FINAL_FORWARD  = 3   # z < Z_FINAL_THRESH: drive in slowly
STATE_DONE           = 4   # z < Z_DONE_THRESH: stop
STATE_SCAN_BACK      = 5   # tag lost: rotate back to recover
STATE_REVERSE        = 6   # stuck: reverse then re-search

STATE_NAME = {
    STATE_SEARCH:         "SEARCH",
    STATE_CURVE_APPROACH: "CURVE_APPROACH",
    STATE_PARALLEL_ALIGN: "PARALLEL_ALIGN",
    STATE_FINAL_FORWARD:  "FINAL_FORWARD",
    STATE_DONE:           "DONE",
    STATE_SCAN_BACK:      "SCAN_BACK",
    STATE_REVERSE:        "REVERSE",
}

STATE_COLOR = {
    STATE_SEARCH:         (180, 180, 180),
    STATE_CURVE_APPROACH: (0,   200,   0),
    STATE_PARALLEL_ALIGN: (255, 220,  50),
    STATE_FINAL_FORWARD:  (0,   255,   0),
    STATE_DONE:           (0,   255, 128),
    STATE_SCAN_BACK:      (50,  150, 255),
    STATE_REVERSE:        (0,     0, 255),
}

# ── Distance thresholds ─────────────────────────────────────────
Z_SLOW_THRESH  = 0.60   # m — slow curve begins (reduce v as z decreases)
Z_FINAL_THRESH = 0.30   # m — enter PARALLEL_ALIGN then FINAL_FORWARD
Z_DONE_THRESH  = 0.15   # m — DONE

# ── SEARCH ──────────────────────────────────────────────────────
SEARCH_W = 0.20         # rad/s — spin speed

# ── CURVE_APPROACH (PID + yaw blend) ───────────────────────────
CURVE_W_MAX       = 0.60   # rad/s — max angular speed
CURVE_V_MIN       = 0.08   # m/s  — floor forward speed
CURVE_YAW_MIX     = 0.35   # blend weight for tag_yaw term
CURVE_YAW_MIX_MAX = 0.35   # rad/s — clamp on yaw_term contribution

# ── PARALLEL_ALIGN ──────────────────────────────────────────────
PARA_KP          = 0.90
PARA_W_MAX       = 0.30
PARA_W_MIN       = 0.08
PARA_YAW_THRESH  = 0.08    # rad (~4.6°) — done threshold
PARA_EMA_ALPHA   = 0.30
PARA_TIMEOUT_SEC = 8.0

# ── FINAL_FORWARD ───────────────────────────────────────────────
FINAL_V           = 0.12   # m/s — base speed (reduced when misaligned)
FINAL_KP_X        = 0.25
FINAL_X_DEAD      = 0.03   # m — lateral deadband
X_TARGET_PARALLEL = 0.05   # m — expected x offset when robot is parallel to tag

# ── REVERSE ─────────────────────────────────────────────────────
REVERSE_SPEED    = -0.15
REVERSE_TIME_SEC = 2.5

# ── SCAN_BACK ───────────────────────────────────────────────────
SCAN_BACK_W       = 0.15    # rad/s
SCAN_BACK_MAX_DEG = 60.0
SCAN_BACK_TIMEOUT = 8.0     # s

# ── Stuck detection ─────────────────────────────────────────────
STUCK_CHECK_SEC   = 3.0
STUCK_Z_MIN_DELTA = 0.03    # m — minimum z progress per check interval
STUCK_ZONE_M      = 1.50    # m — only check when closer than this

# ── Detection ───────────────────────────────────────────────────
TAG_INTERVAL        = 3     # run detector every N frames
TAG_STABLE_REQUIRED = 5     # detections before trusting the tag
LOST_TIMEOUT_SEC    = 5.0
MISS_GRACE          = 5     # allowed consecutive missed frames before reset

# ── X11 debug ───────────────────────────────────────────────────
WIN_NAME     = "AprilTag Follower"
WIN_W, WIN_H = 960, 540
HIST_LEN     = 200
CAM_W, CAM_H = 480, 270
MAP_W, MAP_H = 240, 270
CHART_W      = WIN_W - CAM_W - MAP_W
MAP_SCALE    = 90
MAP_ROBOT_X  = MAP_W // 2
MAP_ROBOT_Y  = MAP_H - 30
X11_RENDER_EVERY = 2


# ── Helpers ─────────────────────────────────────────────────────
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
        derivative     = (error - self._prev_err) / max(dt, 1e-6)
        self._prev_err = error
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
        self._xb.append(x); self._zb.append(z); self._yawb.append(yaw)
        mx, mz, myaw = (float(np.median(b)) for b in (self._xb, self._zb, self._yawb))
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

    def push(self, v): self._d.append(float(v))
    def arr(self):     return np.array(self._d, dtype=np.float32)


# ── X11 drawing ─────────────────────────────────────────────────
def _strip_chart(canvas, x, y, w, h, buf, label, y_min, y_max, color=(0, 220, 0)):
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (28, 28, 28), -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (65, 65, 65),  1)
    data = buf.arr()
    span = float(y_max - y_min) or 1.0
    if y_min < 0 < y_max:
        zy = y + h - int((0.0 - y_min) / span * h)
        cv2.line(canvas, (x, zy), (x + w, zy), (70, 70, 70), 1)
    n   = len(data)
    pts = [(x + int(i / max(n - 1, 1) * (w - 1)),
            y + h - int(np.clip((v - y_min) / span, 0, 1) * (h - 2)))
           for i, v in enumerate(data)]
    for i in range(1, len(pts)):
        cv2.line(canvas, pts[i - 1], pts[i], color, 1)
    cv2.putText(canvas, f"{label}: {float(data[-1]):.3f}",
        (x + 4, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.36, color, 1)
    cv2.putText(canvas, f"[{y_min:.2f}, {y_max:.2f}]",
        (x + 4, y + h - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.29, (100, 100, 100), 1)


def _top_view(canvas, x_off, y_off, tag_x, tag_z, tag_yaw, state, trail):
    cv2.rectangle(canvas, (x_off, y_off), (x_off + MAP_W, y_off + MAP_H), (18, 18, 18), -1)
    cv2.rectangle(canvas, (x_off, y_off), (x_off + MAP_W, y_off + MAP_H), (55, 55, 55),  1)
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
        cv2.putText(canvas, "TAG", (tx + 10, ty - 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
    cv2.circle(canvas, (rx, ry), 8, (0, 180, 255), -1)
    cv2.arrowedLine(canvas, (rx, ry), (rx, ry - 22), (0, 220, 255), 2, tipLength=0.35)
    cv2.putText(canvas, "TOP VIEW", (x_off + 4, y_off + 14),
        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (110, 110, 110), 1)


# ── Main node ───────────────────────────────────────────────────
class AprilTagFollower(Node):

    def __init__(self):
        super().__init__('apriltag_follower')
        self._declare_parameters()

        cam_id               = self.get_parameter('camera_id').value
        self.W               = self.get_parameter('image_width').value
        self.H               = self.get_parameter('image_height').value
        self.use_x11_debug   = self.get_parameter('use_x11_debug').value
        self.invert_yaw      = self.get_parameter('invert_yaw').value
        self.invert_parallel = self.get_parameter('invert_parallel').value
        self.max_linear      = self.get_parameter('max_linear').value
        self.max_angular     = self.get_parameter('max_angular').value
        self.tag_stable_required = self.get_parameter('tag_stable_required').value
        self.lost_timeout        = self.get_parameter('lost_timeout_sec').value

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

        # PID used only in CURVE_APPROACH
        self.pid_bearing = PID(
            self.get_parameter('kp_bearing').value,
            self.get_parameter('ki_bearing').value,
            self.get_parameter('kd_bearing').value,
            out_min=-CURVE_W_MAX, out_max=CURVE_W_MAX)
        self.pid_z = PID(
            self.get_parameter('kp_z').value,
            self.get_parameter('ki_z').value,
            self.get_parameter('kd_z').value,
            out_min=CURVE_V_MIN, out_max=self.max_linear)

        self.at_detector = Detector(
            families="tagStandard52h13",
            nthreads=2, quad_decimate=1.5, refine_edges=1)

        _cmd_topic = self.get_parameter('cmd_topic').value
        self.cmd_pub      = self.create_publisher(Twist,            _cmd_topic,          1)
        self.debug_pub    = self.create_publisher(Float32MultiArray, '/vision_debug',      5)
        self.plant_pub    = self.create_publisher(Int32,             '/apriltag/planting_distance', 5)
        self.gap_pub      = self.create_publisher(Int32,             '/apriltag/gap_type',           5)
        self.interval_pub = self.create_publisher(Int32,             '/apriltag/cabbage_interval',   5)
        self.pose_pub     = self.create_publisher(Float32MultiArray, '/apriltag/pose',               1)
        self.bridge = CvBridge()

        self._wheel_ticks = [0.0, 0.0, 0.0, 0.0]
        self.wheel_sub = self.create_subscription(
            Float32MultiArray, '/wheel_ticks',
            lambda msg: setattr(self, '_wheel_ticks', list(msg.data)), 5)

        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._init_state()

        if self.use_x11_debug:
            self._x11_init()

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info(
            f"AprilTagFollower | res={self.W}×{self.H}"
            f" | z_final={Z_FINAL_THRESH}m  z_done={Z_DONE_THRESH}m"
            f" | invert_parallel={self.invert_parallel}"
            f" | cmd={_cmd_topic}")

    def _declare_parameters(self):
        self.declare_parameter('camera_id',       0)
        self.declare_parameter('image_width',     640)
        self.declare_parameter('image_height',    480)
        self.declare_parameter('use_x11_debug',   True)
        self.declare_parameter('invert_yaw',      False)
        self.declare_parameter('invert_parallel', True)

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

        # PID gains for CURVE_APPROACH
        self.declare_parameter('kp_bearing', 1.50)
        self.declare_parameter('ki_bearing', 0.00)
        self.declare_parameter('kd_bearing', 0.10)
        self.declare_parameter('kp_z',       1.00)
        self.declare_parameter('ki_z',       0.005)
        self.declare_parameter('kd_z',       0.05)

        self.declare_parameter('tag_stable_required', TAG_STABLE_REQUIRED)
        self.declare_parameter('lost_timeout_sec',    LOST_TIMEOUT_SEC)
        self.declare_parameter('cmd_topic',           '/cmd_vel_pid')
        self.declare_parameter('ticks_to_meter',      1.0)

    def _init_state(self):
        self.state         = STATE_SEARCH
        self.frame_count   = 0
        self.smoother      = EMATag(alpha=0.6, median_window=3)
        self.stable_count  = 0
        self.last_tag_time = None
        self.tag_published = False
        self.last_tag_data = None

        self._state_frame      = 0
        self._state_min_frames = 3

        self._tag_side_sign        = 1.0   # +1 = right, -1 = left (from SEARCH)
        self._last_rotate_dir_sign = 1.0   # for SCAN_BACK direction

        self._latest_tag_yaw  = 0.0
        self._latest_para_err = 0.0
        self._miss_count      = 0

        self._para_start_time = None
        self._para_yaw_ema    = None

        self._scan_back_start_time = None
        self._scan_back_dir_sign   = 0.0
        self._scan_back_turned_rad = 0.0
        self._scan_back_last_t     = None

        self._reverse_start_time = None

        self._stuck_last_z    = None
        self._stuck_last_time = None
        self._stuck_count     = 0

    # ── X11 debug ───────────────────────────────────────────────
    def _x11_init(self):
        n = HIST_LEN
        self._dbg_v    = RingBuf(n)
        self._dbg_w    = RingBuf(n)
        self._dbg_x    = RingBuf(n)
        self._dbg_z    = RingBuf(n, default=2.0)
        self._dbg_para = RingBuf(n)
        self._dbg_trail: list[tuple[int, int]] = []
        cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_NAME, WIN_W, WIN_H)
        self.get_logger().info("X11 debug window open  [Q/Esc=quit  C=clear trail]")

    def _x11_push(self, v, w, x, z, para_err):
        self._dbg_v.push(v)
        self._dbg_w.push(w)
        self._dbg_x.push(x        if x        is not None else 0.0)
        self._dbg_z.push(z        if z        is not None else 0.0)
        self._dbg_para.push(para_err if para_err is not None else 0.0)
        if x is not None and z is not None:
            tx = MAP_ROBOT_X + int(x * MAP_SCALE)
            ty = MAP_ROBOT_Y - int(z * MAP_SCALE)
            if not self._dbg_trail or self._dbg_trail[-1] != (tx, ty):
                self._dbg_trail.append((tx, ty))
                if len(self._dbg_trail) > HIST_LEN:
                    self._dbg_trail.pop(0)

    def _x11_render(self, frame, cmd, x, z, yaw, para_err):
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
            (self._dbg_v,    "v  m/s",    -0.05, 0.35, (0,   220,   0)),
            (self._dbg_w,    "ω  r/s",    -0.65, 0.65, (0,   160, 255)),
            (self._dbg_x,    "x  m",      -0.8,  0.8,  (255, 180,   0)),
            (self._dbg_z,    "z  m",       0.0,  2.0,  (0,   220, 220)),
            (self._dbg_para, "para rad",   0.0,  1.6,  (220,   0, 180)),
        ]
        for i, (buf, lbl, lo, hi, c) in enumerate(charts):
            _strip_chart(canvas, rx, i * ch_h, CHART_W - 2, ch_h - 2, buf, lbl, lo, hi, c)

        bar_y = WIN_H - 100
        cv2.rectangle(canvas, (0, bar_y), (WIN_W, WIN_H), (14, 14, 14), -1)
        cv2.line(canvas, (0, bar_y), (WIN_W, bar_y), (55, 55, 55), 1)
        cv2.rectangle(canvas, (10, bar_y + 6), (200, bar_y + 42), col, -1)
        cv2.putText(canvas, STATE_NAME.get(self.state, "?"),
            (14, bar_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 0), 2)

        xv = x        if x        is not None else float('nan')
        zv = z        if z        is not None else float('nan')
        pv = para_err if para_err is not None else float('nan')
        fields = [
            ("x",      f"{xv:.4f} m"),
            ("z",      f"{zv:.4f} m"),
            ("para",   f"{math.degrees(pv):.1f}°" if not math.isnan(pv) else "---"),
            ("v",      f"{cmd.linear.x:.4f} m/s"),
            ("ω",      f"{cmd.angular.z:.4f} r/s"),
            ("stuck#", f"{self._stuck_count}"),
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

    # ── AprilTag detection ──────────────────────────────────────
    def detect_tag(self, frame):
        undist = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs,
                               None, self.new_camera_matrix)
        gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)
        try:
            detections = self.at_detector.detect(
                gray, estimate_tag_pose=True,
                camera_params=(self.fx, self.fy, self.cx_cam, self.cy_cam),
                tag_size=self.tag_size)
        except Exception as e:
            self.get_logger().warn(f"AT detect: {e}")
            return None

        if not detections:
            return None

        best       = max(detections, key=lambda d: cv2.contourArea(d.corners.astype(int)))
        tx, ty, tz = best.pose_t.flatten()
        x_m, z_m   = float(tx), float(tz)

        R  = best.pose_R
        nx = float(R[0, 2])
        nz = float(R[2, 2])
        tag_normal_yaw = math.atan2(nx, nz)                           # signed; 0 = parallel
        parallel_error = math.acos(float(np.clip(nz, -1.0, 1.0)))    # unsigned; 0 = parallel
        bearing_r      = math.atan2(x_m, z_m)

        tag_id = best.tag_id
        ab = tag_id // 1000
        c  = (tag_id // 100) % 10
        de = tag_id % 100
        self.last_tag_data = (ab, c, de, x_m, float(ty), z_m, math.degrees(bearing_r))
        self.last_tag_time = self.get_clock().now()

        corners = best.corners.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (255, 0, 255), 2)
        cv2.putText(frame,
            f"TAG:{tag_id}  x={x_m:.3f}m  z={z_m:.3f}m"
            f"  bear={math.degrees(bearing_r):.1f}°  para={math.degrees(parallel_error):.1f}°",
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

        return x_m, z_m, bearing_r, tag_normal_yaw, parallel_error

    # ── State machine ───────────────────────────────────────────
    def _transition(self, new_state, force=False):
        if new_state == self.state:
            self._state_frame += 1
            return
        if not force:
            holdable = self.state not in (STATE_SEARCH, STATE_DONE, STATE_SCAN_BACK)
            if holdable and self._state_frame < self._state_min_frames:
                return
        self.get_logger().info(
            f"[STATE] {STATE_NAME[self.state]} → {STATE_NAME[new_state]}"
            f" (held {self._state_frame}{'  FORCE' if force else ''})")
        self.state        = new_state
        self._state_frame = 0
        self.pid_bearing.reset()
        self.pid_z.reset()
        self._stuck_last_z    = None
        self._stuck_last_time = None

        if new_state == STATE_SEARCH:
            self.tag_published = False

        if new_state == STATE_REVERSE:
            self._reverse_start_time = self.get_clock().now().nanoseconds / 1e9

        if new_state == STATE_PARALLEL_ALIGN:
            self._para_start_time = self.get_clock().now().nanoseconds / 1e9
            self._para_yaw_ema    = None

        if new_state == STATE_SCAN_BACK:
            self._scan_back_start_time  = self.get_clock().now().nanoseconds / 1e9
            self._scan_back_dir_sign    = -self._last_rotate_dir_sign
            self._scan_back_turned_rad  = 0.0
            self._scan_back_last_t      = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().warn(
                f"[SCAN_BACK] dir={'R' if self._scan_back_dir_sign > 0 else 'L'}"
                f"  max={SCAN_BACK_MAX_DEG:.0f}°")

    def next_state(self, x, z, para_err):
        """
        Primary navigation flow:
          CURVE_APPROACH → (z ≤ Z_FINAL) → PARALLEL_ALIGN → FINAL_FORWARD → DONE
        PARALLEL_ALIGN and FINAL_FORWARD manage their own exits via _transition().
        """
        if z <= Z_DONE_THRESH:
            return STATE_DONE
        if z <= Z_FINAL_THRESH:
            if self.state == STATE_CURVE_APPROACH:
                return STATE_PARALLEL_ALIGN
            return self.state   # PARALLEL_ALIGN / FINAL_FORWARD self-exit
        return STATE_CURVE_APPROACH

    def _check_stuck(self, z):
        """Trigger REVERSE if the robot isn't making progress toward the tag."""
        if self.state not in (STATE_CURVE_APPROACH, STATE_FINAL_FORWARD):
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if z < STUCK_ZONE_M:
            if self._stuck_last_z is None:
                self._stuck_last_z    = z
                self._stuck_last_time = now
            elif now - self._stuck_last_time > STUCK_CHECK_SEC:
                if self._stuck_last_z - z < STUCK_Z_MIN_DELTA:
                    self._stuck_count += 1
                    self.get_logger().warn(
                        f"[STUCK #{self._stuck_count}] z={z:.3f}m → REVERSE")
                    self._transition(STATE_REVERSE, force=True)
                self._stuck_last_z    = z
                self._stuck_last_time = now
        else:
            self._stuck_last_z    = None
            self._stuck_last_time = None

    def compute_cmd(self, x, z, yaw, tag_yaw, para_err, cmd):
        s = self.state

        # ── SEARCH ────────────────────────────────────────────────
        if s == STATE_SEARCH:
            cmd.linear.x          = 0.0
            cmd.angular.z         = SEARCH_W
            self._last_rotate_dir_sign = math.copysign(1.0, SEARCH_W)

        # ── CURVE_APPROACH ─────────────────────────────────────────
        # Bearing PID steers toward tag; clamped tag_yaw term biases
        # the arc so the robot arrives more parallel.
        # Slow zone: linear speed tapers as z drops below Z_SLOW_THRESH.
        elif s == STATE_CURVE_APPROACH:
            bearing = math.atan2(x, max(z, 0.1))
            if self.invert_yaw:
                bearing = -bearing

            yaw_term = CURVE_YAW_MIX * tag_yaw
            yaw_term = max(-CURVE_YAW_MIX_MAX, min(CURVE_YAW_MIX_MAX, yaw_term))

            cmd.angular.z = -(self.pid_bearing.update(bearing) + yaw_term)
            cmd.angular.z = max(-CURVE_W_MAX, min(CURVE_W_MAX, cmd.angular.z))

            # Slow zone: scale v linearly from full speed at Z_SLOW_THRESH to
            # CURVE_V_MIN at Z_FINAL_THRESH, preventing overshoot on approach.
            v_full = self.pid_z.update(z - Z_FINAL_THRESH)
            if z < Z_SLOW_THRESH:
                slow_ratio = max(0.0, (z - Z_FINAL_THRESH) / (Z_SLOW_THRESH - Z_FINAL_THRESH))
                v_full     = CURVE_V_MIN + (v_full - CURVE_V_MIN) * slow_ratio
            cmd.linear.x = max(v_full, CURVE_V_MIN)

            if cmd.angular.z != 0.0:
                self._last_rotate_dir_sign = math.copysign(1.0, cmd.angular.z)

            self.get_logger().info(
                f"[CURVE] x={x:.3f} z={z:.3f}"
                f" bear={math.degrees(bearing):.1f}°"
                f" yaw_term={math.degrees(yaw_term):.1f}°"
                f" v={cmd.linear.x:.3f} w={cmd.angular.z:.3f}",
                throttle_duration_sec=0.3)

        # ── PARALLEL_ALIGN ─────────────────────────────────────────
        # P-control on EMA-smoothed tag_yaw (orientation error).
        # tag_yaw = 0 means robot is parallel to tag.
        # Exits when |EMA| < PARA_YAW_THRESH or timeout.
        elif s == STATE_PARALLEL_ALIGN:
            cmd.linear.x = 0.0
            now_sec = self.get_clock().now().nanoseconds / 1e9
            elapsed = now_sec - (self._para_start_time or now_sec)

            if self._para_yaw_ema is None:
                self._para_yaw_ema = tag_yaw
            else:
                self._para_yaw_ema = (PARA_EMA_ALPHA * tag_yaw
                                      + (1.0 - PARA_EMA_ALPHA) * self._para_yaw_ema)

            done = abs(self._para_yaw_ema) < PARA_YAW_THRESH or elapsed > PARA_TIMEOUT_SEC
            if done:
                cmd.angular.z = 0.0
                reason = "aligned" if abs(self._para_yaw_ema) < PARA_YAW_THRESH else f"timeout {elapsed:.1f}s"
                self.get_logger().info(
                    f"[PARALLEL] {reason}"
                    f"  ema={math.degrees(self._para_yaw_ema):.1f}° → FINAL_FORWARD")
                self._transition(STATE_FINAL_FORWARD, force=True)
                return

            # Use EMA (not raw tag_yaw) for smoother, less reactive control
            yaw_err = self._para_yaw_ema
            if self.invert_parallel:
                yaw_err = -yaw_err

            w = PARA_KP * yaw_err
            w = max(-PARA_W_MAX, min(PARA_W_MAX, w))
            if abs(w) > 1e-3:
                w = math.copysign(max(abs(w), PARA_W_MIN), w)
            cmd.angular.z = w
            if w != 0.0:
                self._last_rotate_dir_sign = math.copysign(1.0, w)

            self.get_logger().info(
                f"[PARALLEL] tag_yaw={math.degrees(tag_yaw):+.1f}°"
                f"  ema={math.degrees(self._para_yaw_ema):+.1f}°"
                f"  w={cmd.angular.z:+.3f}  t={elapsed:.1f}s",
                throttle_duration_sec=0.3)

        # ── FINAL_FORWARD ──────────────────────────────────────────
        # Speed scales down proportionally with lateral error,
        # so the robot slows when misaligned (e.g. x_err=0.05 → 80% speed).
        elif s == STATE_FINAL_FORWARD:
            x_err = x - X_TARGET_PARALLEL
            cmd.linear.x  = FINAL_V * max(0.4, 1.0 - abs(x_err) * 4.0)
            if abs(x_err) < FINAL_X_DEAD:
                w = 0.0
            else:
                w = -FINAL_KP_X * x_err
                w = max(-0.12, min(0.12, w))
            cmd.angular.z = w
            self.get_logger().info(
                f"[FINAL] x={x:.3f} z={z:.3f}"
                f"  x_err={x_err:+.3f}  v={cmd.linear.x:.3f} w={cmd.angular.z:.3f}",
                throttle_duration_sec=0.3)

        # ── SCAN_BACK ──────────────────────────────────────────────
        elif s == STATE_SCAN_BACK:
            cmd.linear.x = 0.0
            now_sec = self.get_clock().now().nanoseconds / 1e9
            elapsed = now_sec - (self._scan_back_start_time or now_sec)
            dt      = now_sec - (self._scan_back_last_t or now_sec)
            self._scan_back_last_t = now_sec

            if (elapsed > SCAN_BACK_TIMEOUT
                    or self._scan_back_turned_rad >= math.radians(SCAN_BACK_MAX_DEG)):
                cmd.angular.z = 0.0
                self.get_logger().warn("[SCAN_BACK] max reached → SEARCH")
                self._transition(STATE_SEARCH, force=True)
                return

            cmd.angular.z = (self._scan_back_dir_sign or 1.0) * SCAN_BACK_W
            self._scan_back_turned_rad += abs(cmd.angular.z) * max(dt, 0.0)
            self.get_logger().info(
                f"[SCAN_BACK] {math.degrees(self._scan_back_turned_rad):.1f}°"
                f"/{SCAN_BACK_MAX_DEG:.0f}°  t={elapsed:.1f}s",
                throttle_duration_sec=0.4)

        # ── REVERSE ────────────────────────────────────────────────
        elif s == STATE_REVERSE:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if self._reverse_start_time is None:
                self._reverse_start_time = now_sec
            elapsed = now_sec - self._reverse_start_time

            if elapsed < REVERSE_TIME_SEC:
                cmd.linear.x  = REVERSE_SPEED
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f"[REVERSE] {elapsed:.1f}s / {REVERSE_TIME_SEC:.1f}s",
                    throttle_duration_sec=0.5)
            else:
                cmd.linear.x = cmd.angular.z = 0.0
                self.smoother.reset()
                self.stable_count  = 0
                self.last_tag_time = None
                self.get_logger().info("[REVERSE] done → SEARCH")
                self._transition(STATE_SEARCH, force=True)

        else:  # DONE
            cmd.linear.x = cmd.angular.z = 0.0

    # ── Camera overlay ──────────────────────────────────────────
    def draw_overlay(self, frame, cmd, x, z, para_err):
        col = STATE_COLOR.get(self.state, (255, 255, 255))

        def txt(img, text, pos, scale, color, thick=1):
            cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 4)
            cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)

        cv2.rectangle(frame, (0, 0), (self.W, 28), (0, 0, 0), -1)
        txt(frame, STATE_NAME[self.state], (4, 20), 0.50, col, 1)

        if x is not None:
            para_deg = math.degrees(para_err) if para_err is not None else float('nan')
            overlay  = frame.copy()
            cv2.rectangle(overlay, (0, 28), (self.W, 78), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
            txt(frame, f"x={x:+.2f}m",           ( 4, 45), 0.38, (  0, 255, 255), 1)
            txt(frame, f"z={z:.2f}m",             (90, 45), 0.38, ( 80, 255,  80), 1)
            txt(frame, f"para={para_deg:.1f}°",   (170, 45), 0.38, (255, 220,  50), 1)
            txt(frame, f"v={cmd.linear.x:+.2f}",  (  4, 62), 0.36, (230, 230, 230), 1)
            txt(frame, f"w={cmd.angular.z:+.2f}", ( 90, 62), 0.36, (  0, 165, 255), 1)
            txt(frame, f"stk={self._stuck_count}",(190, 62), 0.36, (255, 100, 100), 1)

            bh = self.H - 12
            def z2y(zv, zm=2.0):
                return int(bh * (1 - min(zv, zm) / zm)) + 6
            cv2.line(frame, (self.W - 10, 6), (self.W - 10, bh), (60, 60, 60), 2)
            cv2.circle(frame, (self.W - 10, z2y(z)), 5, col, -1)
            for thresh, tc in [(Z_FINAL_THRESH, (80, 200, 255)), (Z_DONE_THRESH, (0, 80, 255))]:
                cv2.line(frame, (self.W - 16, z2y(thresh)), (self.W - 4, z2y(thresh)), tc, 1)

    # ── Main loop ───────────────────────────────────────────────
    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1
        cmd = Twist()
        x = z = yaw = tag_yaw = para_err = None

        # Detection pass
        if self.frame_count % TAG_INTERVAL == 0:
            result = self.detect_tag(frame)
            if result is not None:
                x_r, z_r, b_r, ty_r, pe_r = result
                self.smoother.update(x_r, z_r, b_r)
                self._latest_tag_yaw  = ty_r
                self._latest_para_err = pe_r
                self.stable_count    += 1
                self._miss_count      = 0
                if self.state == STATE_SEARCH:
                    self._tag_side_sign = math.copysign(1.0, x_r)
                if self.state == STATE_SCAN_BACK and self.stable_count >= self.tag_stable_required:
                    self.get_logger().info("[SCAN_BACK] tag found → CURVE_APPROACH")
                    self._transition(STATE_CURVE_APPROACH, force=True)
            else:
                self._miss_count += 1
                if self._miss_count > MISS_GRACE:
                    self.stable_count = 0

        # Tag lost check (navigation states)
        NAV_STATES = (STATE_CURVE_APPROACH, STATE_PARALLEL_ALIGN, STATE_FINAL_FORWARD)
        if self.state in NAV_STATES and self.last_tag_time is not None:
            lost_sec = (self.get_clock().now() - self.last_tag_time).nanoseconds / 1e9
            if lost_sec > self.lost_timeout:
                self.get_logger().warn(f"[LOST] {lost_sec:.1f}s → SCAN_BACK")
                self._transition(STATE_SCAN_BACK, force=True)
                self.stable_count = 0

        # Execute current state
        if self.state == STATE_DONE:
            cmd.linear.x = cmd.angular.z = 0.0

        elif self.state in (STATE_REVERSE, STATE_SCAN_BACK):
            self.compute_cmd(None, None, None, None, 0.0, cmd)

        elif self.state == STATE_SEARCH:
            if self.smoother.valid and self.stable_count >= self.tag_stable_required:
                side = 'R' if self._tag_side_sign > 0 else 'L'
                self.get_logger().info(f"[SEARCH] tag found ({side}) → CURVE_APPROACH")
                self._transition(STATE_CURVE_APPROACH, force=True)
            else:
                self.compute_cmd(None, None, None, None, 0.0, cmd)

        elif self.smoother.valid and self.stable_count >= self.tag_stable_required:
            x, z, yaw = self.smoother.x, self.smoother.z, self.smoother.yaw
            tag_yaw   = self._latest_tag_yaw
            para_err  = self._latest_para_err
            self._transition(self.next_state(x, z, para_err))
            self.compute_cmd(x, z, yaw, tag_yaw, para_err, cmd)
            self._check_stuck(z)
        # else: not enough stable detections — stay still

        self.draw_overlay(frame, cmd, x, z, para_err)

        if self.use_x11_debug:
            self._x11_push(cmd.linear.x, cmd.angular.z, x, z, para_err)
            if self.frame_count % X11_RENDER_EVERY == 0:
                self._x11_render(frame, cmd, x, z, yaw, para_err)

        self.cmd_pub.publish(cmd)

        dbg = Float32MultiArray()
        dbg.data = [
            float(self.state),
            float(x)        if x        is not None else -999.0,
            float(z)        if z        is not None else -999.0,
            float(yaw)      if yaw      is not None else -999.0,
            float(cmd.linear.x),
            float(cmd.angular.z),
            float(self.stable_count),
            float(self._stuck_count),
            float(para_err) if para_err is not None else -999.0,
        ]
        self.debug_pub.publish(dbg)


# ── Entry point ─────────────────────────────────────────────────
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