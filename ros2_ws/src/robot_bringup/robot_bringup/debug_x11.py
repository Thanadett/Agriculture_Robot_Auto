#!/usr/bin/env python3
"""
Debug X11 Node — Unified Visualization
============================================================
แสดงข้อมูลจากทุก node ในหน้าต่างเดียว:
  • ซ้าย  : ภาพกล้องพร้อม overlay
  • กลาง  : Top-down map + heading lock indicator
  • ขวา   : Mission state timeline + params
  • ล่าง  : Real-time charts (z, bearing, v, ω, heading_error, heading_u)

Layout (1440 × 800):
  ┌─[Camera 560×420]──┬─[Map 300×420]──┬─[Mission 580×420]─┐
  └─────────────────── Charts (1440×380) ────────────────────┘

Topics subscribed:
  /camera/image_raw/compressed  (CompressedImage)
  /vision_debug                 (Float32MultiArray)
      [state, x, z, yaw, v, w, n_stable, stuck, bearing]
  /cmd_vel_pid                  (Twist)   — final robot command
  /pid_debug                    (Float32MultiArray)
      [error, u, deriv, yaw_ref, yaw]
  /msg                          (String)  — mission events
  /mission_debug                (Int32)   — current MS_* state
"""

import math
import collections
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Int32, String

import cv2
import numpy as np

# ════════════════════════════════════════════════════════════════
# Vision states (mirror of apriltag_servo)
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
    STATE_REVERSE:   (80,   80, 255),
}

# ════════════════════════════════════════════════════════════════
# Mission states (mirror of mission_controller)
# ════════════════════════════════════════════════════════════════
MS_IDLE = 0; MS_WAIT_0 = 1; MS_APPROACH_M = 2; MS_REVERSE_M = 3
MS_WAIT_REVERSE = 4; MS_PLANT_1 = 5; MS_WAIT_1 = 6; MS_PLANT_2 = 7
MS_WAIT_2 = 8; MS_GAP = 9; MS_WAIT_3 = 10; MS_INTERVAL_1 = 11
MS_WAIT_4A = 12; MS_INTERVAL_2 = 13; MS_WAIT_4B = 14; MS_FINISH = 15

MS_NAME = {
    0:"IDLE", 1:"WAIT_0", 2:"APPROACH", 3:"REVERSE", 4:"WAIT_REV",
    5:"PLANT_1", 6:"WAIT_1", 7:"PLANT_2", 8:"WAIT_2",
    9:"GAP", 10:"WAIT_3", 11:"INTERVAL_1", 12:"WAIT_4A",
    13:"INTERVAL_2", 14:"WAIT_4B", 15:"FINISH",
}

MS_COLOR = {
    0: (100,100,100), 1: (180,180, 50),
    2: (  0,200,200), 3: (200, 50, 50), 4: (180,180, 50),
    5: ( 50,220,100), 6: (200,200, 50), 7: ( 50,220,100), 8: (200,200, 50),
    9: (200,100,250), 10:(200,200, 50), 11:(250,150, 50), 12:(200,200, 50),
    13:(250,150, 50), 14:(200,200, 50), 15:(  0,255,255),
}

# ════════════════════════════════════════════════════════════════
# Window layout
# ════════════════════════════════════════════════════════════════
WIN_W  = 1440
WIN_H  = 800
CAM_W  = 560
CAM_H  = 420
MAP_X  = CAM_W
MAP_W  = 300
MAP_H  = 420
INFO_X = CAM_W + MAP_W
INFO_W = WIN_W - INFO_X
INFO_H = 420
CHT_Y  = 420
CHT_H  = WIN_H - CHT_Y

HIST_N   = 300
MAP_SCALE= 110
MAP_CX   = MAP_W // 2
MAP_CY   = MAP_H - 30

Z_ALIGN  = 0.58
Z_FORWARD= 0.40
Z_STOP   = 0.25

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_MONO = cv2.FONT_HERSHEY_PLAIN


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
# Drawing helpers
# ════════════════════════════════════════════════════════════════
def _txt(img, text, pos, scale, color, thick=1, shadow=True):
    if shadow:
        cv2.putText(img, text, pos, _FONT, scale, (0,0,0), thick+3)
    cv2.putText(img, text, pos, _FONT, scale, color, thick)


def draw_chart(canvas, x, y, w, h, ring, label, lo, hi, color,
               fill_alpha=0.18):
    """Draw a single scrolling strip with filled area under curve."""
    cv2.rectangle(canvas, (x,y), (x+w, y+h), (18,18,18), -1)
    cv2.rectangle(canvas, (x,y), (x+w, y+h), (38,38,38),  1)
    span = float(hi - lo) or 1.0

    # zero line
    if lo < 0 < hi:
        zy = y + h - int(-lo / span * h)
        cv2.line(canvas, (x, zy), (x+w, zy), (50,50,50), 1)

    data = ring.arr(); n = len(data)
    if n < 2: return

    # Build polyline
    pts = []
    for i, v in enumerate(data):
        px = x + int(i / max(n-1,1) * (w-1))
        py = y + h - int(np.clip((v-lo)/span, 0, 1) * (h-2))
        pts.append((px, py))

    # Filled polygon under curve
    if fill_alpha > 0:
        baseline_y = y + h - int(-lo / span * h) if lo < 0 < hi else y + h
        poly = np.array(pts + [(pts[-1][0], baseline_y),
                                (pts[0][0],  baseline_y)], dtype=np.int32)
        overlay = canvas.copy()
        cv2.fillPoly(overlay, [poly], color)
        cv2.addWeighted(overlay, fill_alpha, canvas, 1-fill_alpha, 0, canvas)

    # Line
    for i in range(1, len(pts)):
        cv2.line(canvas, pts[i-1], pts[i], color, 1)

    val = ring.last()
    _txt(canvas, f"{label}  {val:+.3f}",
         (x+6, y+14), 0.34, color)
    cv2.putText(canvas, f"[{lo:.1f},{hi:.1f}]",
                (x+6, y+h-5), _FONT, 0.26, (55,55,55), 1)

    # Current value indicator (dot on right edge)
    lv = pts[-1]
    cv2.circle(canvas, lv, 3, color, -1)


def draw_topview(canvas, ox, oy, tag_x, tag_z, tag_yaw, state, trail):
    cv2.rectangle(canvas, (ox,oy), (ox+MAP_W, oy+MAP_H), (12,12,18), -1)
    cv2.rectangle(canvas, (ox,oy), (ox+MAP_W, oy+MAP_H), (45,45,60),  1)
    rx = ox + MAP_CX; ry = oy + MAP_CY

    # Grid lines
    for zm in np.arange(0.2, 2.6, 0.2):
        gy  = ry - int(zm * MAP_SCALE)
        col = (38,38,45) if round(zm*5)%5 else (58,58,70)
        if oy < gy < oy+MAP_H:
            cv2.line(canvas, (ox, gy), (ox+MAP_W, gy), col, 1)

    # Zone lines
    for zm, lbl, col in [
            (Z_ALIGN,   "ALIGN",   (60,100,200)),
            (Z_FORWARD, "FORWARD", (50,170, 60)),
            (Z_STOP,    "STOP",    (160, 50, 50))]:
        gy = ry - int(zm * MAP_SCALE)
        if oy < gy < oy+MAP_H:
            cv2.line(canvas, (ox,gy), (ox+MAP_W,gy), col, 1)
            cv2.putText(canvas, f"{lbl} {zm:.2f}m",
                        (ox+MAP_W-75, gy-2), _FONT, 0.24, col, 1)

    # Trail
    for i in range(1, len(trail)):
        cv2.line(canvas, trail[i-1], trail[i], (30,65,35), 1)

    scol = STATE_COLOR.get(state, (180,180,180))
    # Tag marker
    if tag_z is not None and tag_z > 0:
        tx = rx + int(tag_x * MAP_SCALE)
        ty = ry - int(tag_z * MAP_SCALE)
        tx = max(ox+5, min(ox+MAP_W-5, tx))
        ty = max(oy+5, min(oy+MAP_H-5, ty))
        cv2.circle(canvas, (tx,ty), 10, scol, -1)
        cv2.circle(canvas, (tx,ty), 10, (255,255,255), 1)
        # Yaw arrow
        ax = tx + int(math.sin(tag_yaw) * 28)
        ay = ty - int(math.cos(tag_yaw) * 28)
        cv2.arrowedLine(canvas, (tx,ty), (ax,ay), (255,200,0), 2, tipLength=0.4)
        # Distance label
        cv2.line(canvas, (rx,ry), (tx,ty), (50,50,60), 1)
        _txt(canvas, f"z={tag_z:.2f}m", (tx+8,ty-8), 0.32, scol)

    # Robot (fixed at bottom)
    cv2.circle(canvas, (rx,ry), 9, (0,180,255), -1)
    cv2.circle(canvas, (rx,ry), 9, (255,255,255), 1)
    cv2.arrowedLine(canvas, (rx,ry), (rx,ry-26), (0,230,255), 2, tipLength=0.4)
    _txt(canvas, "MAP", (ox+5,oy+14), 0.32, (70,70,90))


def draw_mission_panel(canvas, ox, oy, ms_state, ms_events,
                       plant_ok, capture_ok):
    """Right panel: mission state progress bar + event log."""
    cv2.rectangle(canvas, (ox,oy), (ox+INFO_W, oy+INFO_H), (10,10,14), -1)
    cv2.rectangle(canvas, (ox,oy), (ox+INFO_W, oy+INFO_H), (40,40,55),  1)

    _txt(canvas, "MISSION", (ox+8, oy+18), 0.50, (160,160,200), 2)

    # Progress bar — 16 states
    bar_total = 16
    bx  = ox + 8; by = oy + 30
    bw  = (INFO_W - 16) // bar_total
    bh  = 28
    for i in range(bar_total):
        x0 = bx + i*bw; y0 = by
        col = MS_COLOR.get(i, (80,80,80))
        if i == ms_state:
            cv2.rectangle(canvas, (x0,y0), (x0+bw-2, y0+bh), col, -1)
            cv2.rectangle(canvas, (x0,y0), (x0+bw-2, y0+bh), (255,255,255), 1)
        elif i < ms_state:
            cv2.rectangle(canvas, (x0,y0), (x0+bw-2, y0+bh),
                          tuple(c//4 for c in col), -1)
        else:
            cv2.rectangle(canvas, (x0,y0), (x0+bw-2, y0+bh), (25,25,30), -1)

    # Current state label (large)
    cur_name = MS_NAME.get(ms_state, f"MS{ms_state}")
    cur_col  = MS_COLOR.get(ms_state, (200,200,200))
    _txt(canvas, cur_name, (ox+8, oy+85), 0.65, cur_col, 2)

    # Feedback status
    pk_col = (0,230,100) if plant_ok else (80,80,80)
    ck_col = (0,230,100) if capture_ok else (80,80,80)
    cv2.circle(canvas, (ox+8+6,  oy+108), 6, pk_col, -1)
    cv2.circle(canvas, (ox+8+26, oy+108), 6, ck_col, -1)
    _txt(canvas, "plant", (ox+18,  oy+113), 0.30, pk_col)
    _txt(canvas, "capture",(ox+36, oy+113), 0.30, ck_col)

    # Event log (last 12 events)
    _txt(canvas, "EVENT LOG", (ox+8, oy+132), 0.32, (100,100,140))
    log_y = oy + 148
    for i, (ts, ev) in enumerate(ms_events):
        col = (0,255,200) if "FINISH" in ev else \
              (0,200,100) if "planting" in ev else \
              (200,150,50) if "cabbage" in ev else (160,160,160)
        _txt(canvas, f"{ts:>6.1f}s  {ev}", (ox+8, log_y + i*17),
             0.31, col, shadow=False)


# ════════════════════════════════════════════════════════════════
# Node
# ════════════════════════════════════════════════════════════════
class DebugX11(Node):

    def __init__(self):
        super().__init__('debug_x11')
        self.declare_parameter('skip_frames', 2)
        self.declare_parameter('window_name', 'Robot Debug')
        self._skip = self.get_parameter('skip_frames').value
        self._win  = self.get_parameter('window_name').value

        self._build_ros()
        self._init_data()
        self._init_rings()

        cv2.namedWindow(self._win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._win, WIN_W, WIN_H)
        self.get_logger().info(
            f"DebugX11 ready — window '{self._win}'  [Q/Esc=quit  C=clear trail]")

        self._render_timer = self.create_timer(0.05, self._render)  # 20 Hz

    # ── ROS interfaces ───────────────────────────────────────────
    def _build_ros(self):
        self.create_subscription(CompressedImage,
            '/camera/image_raw/compressed', self._cb_img, 10)
        self.create_subscription(Float32MultiArray,
            '/vision_debug', self._cb_vision, 5)
        self.create_subscription(Twist,
            '/cmd_vel_pid', self._cb_cmd, 1)
        self.create_subscription(Float32MultiArray,
            '/pid_debug', self._cb_pid, 10)
        self.create_subscription(String,
            '/msg', self._cb_msg, 10)
        self.create_subscription(Int32,
            '/mission_debug', self._cb_ms, 5)

    # ── State ────────────────────────────────────────────────────
    def _init_data(self):
        self._frame       = None
        self._frame_cnt   = 0

        # vision_debug: [state, x, z, yaw, v, w, n_stable, stuck, bearing]
        self._v_state   = float(STATE_SEARCH)
        self._v_x       = None
        self._v_z       = None
        self._v_yaw     = None
        self._v_v       = 0.0
        self._v_w       = 0.0
        self._v_stable  = 0
        self._v_stuck   = 0
        self._v_bearing = None

        # cmd_vel_pid
        self._cmd_v = 0.0
        self._cmd_w = 0.0

        # pid_debug: [error, u, deriv, yaw_ref, yaw]
        self._pid_error = 0.0
        self._pid_u     = 0.0
        self._pid_deriv = 0.0
        self._pid_yaw_ref = 0.0
        self._pid_yaw     = 0.0

        # mission
        self._ms_state   = 0
        self._ms_events  = collections.deque(maxlen=12)   # (ts, text)
        self._t0         = time.monotonic()
        self._plant_ok   = False
        self._capture_ok = False

        self._trail: list[tuple[int,int]] = []

    def _init_rings(self):
        n = HIST_N
        self.rb_z    = Ring(n, 2.0)
        self.rb_x    = Ring(n)
        self.rb_bear = Ring(n)
        self.rb_yaw  = Ring(n)
        self.rb_v    = Ring(n)
        self.rb_w    = Ring(n)
        self.rb_herr = Ring(n)   # heading error
        self.rb_hu   = Ring(n)   # heading PID output
        self.rb_cmdv = Ring(n)
        self.rb_cmdw = Ring(n)

    # ── Callbacks ────────────────────────────────────────────────
    def _cb_img(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self._frame = frame
                self._frame_cnt += 1
        except Exception:
            pass

    def _cb_vision(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) >= 9:
            self._v_state   = d[0]
            self._v_x       = d[1] if d[1] != -999.0 else None
            self._v_z       = d[2] if d[2] != -999.0 else None
            self._v_yaw     = d[3] if d[3] != -999.0 else None
            self._v_v       = d[4]
            self._v_w       = d[5]
            self._v_stable  = int(d[6])
            self._v_stuck   = int(d[7])
            self._v_bearing = d[8] if d[8] != -999.0 else None
            # push to rings
            self.rb_z.push(self._v_z if self._v_z is not None else 2.0)
            self.rb_x.push(self._v_x if self._v_x is not None else 0.0)
            self.rb_v.push(self._v_v)
            self.rb_w.push(self._v_w)
            if self._v_bearing is not None:
                self.rb_bear.push(self._v_bearing)
            if self._v_yaw is not None:
                self.rb_yaw.push(self._v_yaw)
            # trail update
            if self._v_x is not None and self._v_z is not None:
                px = MAP_X + MAP_CX + int(self._v_x * MAP_SCALE)
                py = MAP_CY - int(self._v_z * MAP_SCALE)
                if not self._trail or self._trail[-1] != (px, py):
                    self._trail.append((px, py))
                    if len(self._trail) > HIST_N:
                        self._trail.pop(0)

    def _cb_cmd(self, msg: Twist):
        self._cmd_v = msg.linear.x
        self._cmd_w = msg.angular.z
        self.rb_cmdv.push(self._cmd_v)
        self.rb_cmdw.push(self._cmd_w)

    def _cb_pid(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) >= 5:
            self._pid_error   = d[0]
            self._pid_u       = d[1]
            self._pid_deriv   = d[2]
            self._pid_yaw_ref = d[3]
            self._pid_yaw     = d[4]
            self.rb_herr.push(d[0])
            self.rb_hu.push(d[1])

    def _cb_msg(self, msg: String):
        ts = time.monotonic() - self._t0
        self._ms_events.append((ts, msg.data))
        # Update feedback state flags visually
        if "planting" in msg.data:
            self._plant_ok = True
        if "cabbage" in msg.data:
            self._capture_ok = True
        if "FINISH" in msg.data:
            self._plant_ok = self._capture_ok = False

    def _cb_ms(self, msg: Int32):
        self._ms_state = msg.data
        # reset feedback indicators on state entry
        if msg.data in (5, 7):   # PLANT_x start
            self._plant_ok = False
        if msg.data in (9, 11, 13):   # GAP/INTERVAL start
            self._capture_ok = False

    # ── Render ───────────────────────────────────────────────────
    def _render(self):
        canvas = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)

        v_state   = int(self._v_state)
        tag_x     = self._v_x     if self._v_x     is not None else 0.0
        tag_z     = self._v_z
        tag_yaw   = self._v_yaw   if self._v_yaw   is not None else 0.0
        bearing   = self._v_bearing if self._v_bearing is not None else 0.0
        scol      = STATE_COLOR.get(v_state, (180,180,180))

        # ── Camera panel ─────────────────────────────────────────
        if self._frame is not None:
            cam = cv2.resize(self._frame, (CAM_W, CAM_H))
        else:
            cam = np.zeros((CAM_H, CAM_W, 3), dtype=np.uint8)
            _txt(cam, "NO CAMERA", (CAM_W//2-60, CAM_H//2), 0.7, (60,60,80), 2)

        # Camera header bar
        cv2.rectangle(cam, (0,0), (CAM_W, 32), (0,0,0), -1)
        _txt(cam, STATE_NAME.get(v_state,"?"), (6,23), 0.65, scol, 2)
        _txt(cam, f"stable:{self._v_stable:3d}  stuck:{self._v_stuck:2d}",
             (180, 23), 0.38, (120,120,120))

        # Tag overlay bar
        if tag_z is not None:
            cv2.rectangle(cam, (0, CAM_H-36), (CAM_W, CAM_H), (0,0,0,128), -1)
            _txt(cam, f"x={tag_x:+.3f}m", (6, CAM_H-21), 0.38, (0,220,220))
            _txt(cam, f"z={tag_z:.3f}m",  (110,CAM_H-21), 0.38, (80,255,80))
            _txt(cam, f"bear={math.degrees(bearing):+.1f}°",
                 (210, CAM_H-21), 0.38, (0,190,255))
            _txt(cam, f"yaw={math.degrees(tag_yaw):+.1f}°",
                 (350, CAM_H-21), 0.38, (255,200,50))

        canvas[0:CAM_H, 0:CAM_W] = cam

        # ── Map panel ────────────────────────────────────────────
        draw_topview(canvas, MAP_X, 0,
                     tag_x, tag_z, tag_yaw, v_state, self._trail)

        # HeadingPID lock indicator on map
        lock_active = abs(self._cmd_v) > 0.02 and abs(self._cmd_w) < 0.05
        lx = MAP_X + MAP_W - 80; ly = 10
        lc = (0,230,100) if lock_active else (60,60,80)
        cv2.rectangle(canvas, (lx,ly), (lx+76, ly+22), (0,0,0), -1)
        cv2.rectangle(canvas, (lx,ly), (lx+76, ly+22), lc, 1)
        _txt(canvas, "HDG LOCK" if lock_active else "HDG FREE",
             (lx+4, ly+16), 0.35, lc)

        # HeadingPID mini display
        hx = MAP_X + 4; hy = MAP_H - 70
        cv2.rectangle(canvas, (hx,hy), (hx+MAP_W-8, hy+64), (0,0,0), -1)
        _txt(canvas, "HeadingPID", (hx+4, hy+13), 0.32, (120,120,160))
        _txt(canvas, f"ref={math.degrees(self._pid_yaw_ref):+.1f}°"
                     f"  cur={math.degrees(self._pid_yaw):+.1f}°",
             (hx+4, hy+28), 0.32, (160,160,200))
        _txt(canvas, f"err={math.degrees(self._pid_error):+.2f}°"
                     f"  u={self._pid_u:+.3f}",
             (hx+4, hy+43), 0.32,
             (0,230,100) if abs(self._pid_error) < math.radians(3) else (255,160,50))
        # Error bar
        bar_w = MAP_W - 20
        bar_x = hx + 4; bar_y = hy + 52
        cv2.rectangle(canvas, (bar_x,bar_y), (bar_x+bar_w, bar_y+8), (30,30,30), -1)
        norm = min(abs(self._pid_error) / math.radians(20), 1.0)
        bfc  = (0,200,80) if norm < 0.2 else (200,180,0) if norm < 0.5 else (220,60,60)
        cv2.rectangle(canvas, (bar_x,bar_y),
                      (bar_x+int(bar_w*norm), bar_y+8), bfc, -1)

        # ── Mission panel ────────────────────────────────────────
        draw_mission_panel(canvas, INFO_X, 0,
                           self._ms_state,
                           list(self._ms_events)[-12:],
                           self._plant_ok, self._capture_ok)

        # Final cmd overlay on mission panel (bottom-right)
        cx = INFO_X + 8; cy = INFO_H - 44
        cv2.rectangle(canvas, (cx,cy), (cx+INFO_W-16, cy+40), (0,0,0), -1)
        _txt(canvas, "FINAL CMD", (cx+4, cy+13), 0.32, (100,100,140))
        v_col = (0,220,100) if self._cmd_v > 0.02 else \
                (200,80,80) if self._cmd_v < -0.02 else (60,60,80)
        w_col = (0,190,255) if abs(self._cmd_w) > 0.02 else (60,60,80)
        _txt(canvas, f"v={self._cmd_v:+.3f} m/s", (cx+4,  cy+30), 0.40, v_col)
        _txt(canvas, f"ω={self._cmd_w:+.3f} r/s", (cx+160, cy+30), 0.40, w_col)

        # ── Charts panel ─────────────────────────────────────────
        n_charts = 6
        cw = WIN_W // n_charts
        charts = [
            (self.rb_z,    "z (m)",            0.0,  2.0,  (  0,220,200)),
            (self.rb_bear, "bearing (rad)",    -1.5,  1.5,  (  0,190,255)),
            (self.rb_yaw,  "yaw (rad)",        -1.5,  1.5,  (255,200, 50)),
            (self.rb_cmdv, "cmd v (m/s)",      -0.1,  0.4,  (  0,210,  0)),
            (self.rb_cmdw, "cmd ω (r/s)",      -0.6,  0.6,  ( 80,130,255)),
            (self.rb_herr, "hdg err (rad)",    -0.4,  0.4,  (255,140, 60)),
        ]
        for i, (ring, lbl, lo, hi, col) in enumerate(charts):
            draw_chart(canvas, i*cw, CHT_Y, cw-1, CHT_H-1,
                       ring, lbl, lo, hi, col)

        # Bottom border with FPS
        fps_txt = f"frame #{self._frame_cnt}"
        cv2.putText(canvas, fps_txt, (WIN_W-120, WIN_H-6),
                    _FONT, 0.28, (50,50,60), 1)

        cv2.imshow(self._win, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            raise KeyboardInterrupt
        if key == ord('c'):
            self._trail.clear()
            self.get_logger().info("[DEBUG] trail cleared")


# ════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = DebugX11()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()