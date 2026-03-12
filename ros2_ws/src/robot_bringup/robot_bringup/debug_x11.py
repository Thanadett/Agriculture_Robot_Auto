#!/usr/bin/env python3
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
# Vision states
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
# Mission states
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

    if lo < 0 < hi:
        zy = y + h - int(-lo / span * h)
        cv2.line(canvas, (x, zy), (x+w, zy), (50,50,50), 1)

    data = ring.arr(); n = len(data)
    if n < 2: return

    pts = []
    for i, v in enumerate(data):
        px = x + int(i / max(n-1,1) * (w-1))
        py = y + h - int(np.clip((v-lo)/span, 0, 1) * (h-2))
        pts.append((px, py))

    if fill_alpha > 0:
        baseline_y = y + h - int(-lo / span * h) if lo < 0 < hi else y + h
        poly = np.array(pts + [(pts[-1][0], baseline_y),
                                (pts[0][0],  baseline_y)], dtype=np.int32)
        overlay = canvas.copy()
        cv2.fillPoly(overlay, [poly], color)
        cv2.addWeighted(overlay, fill_alpha, canvas, 1-fill_alpha, 0, canvas)

    for i in range(1, len(pts)):
        cv2.line(canvas, pts[i-1], pts[i], color, 1)

    val = ring.last()
    _txt(canvas, f"{label}  {val:+.3f}",
         (x+6, y+14), 0.34, color)
    cv2.putText(canvas, f"[{lo:.1f},{hi:.1f}]",
                (x+6, y+h-5), _FONT, 0.26, (55,55,55), 1)

    lv = pts[-1]
    cv2.circle(canvas, lv, 3, color, -1)


# ════════════════════════════════════════════════════════════════
# World-frame top-down map  (ใหม่ — แทน draw_topview เดิม)
# ════════════════════════════════════════════════════════════════
def draw_worldmap(canvas, ox, oy,
                  robot_wx, robot_wy, robot_yaw,
                  plant_wx, plant_wy,
                  world_trail, state):
    """
    World-frame top-down map:
      • Plant (tag) วาดเป็น green circle
      • Robot วาดเป็น blue circle + red yaw arrow
      • Trail  วาดเส้น path ที่ผ่านมา
      • Grid + axis labels คล้าย matplotlib
    """
    # ── Background ───────────────────────────────────────────────
    cv2.rectangle(canvas, (ox, oy), (ox+MAP_W, oy+MAP_H), (15, 15, 20), -1)
    cv2.rectangle(canvas, (ox, oy), (ox+MAP_W, oy+MAP_H), (40, 40, 55),  1)

    # ── Plot area margins ────────────────────────────────────────
    PAD_L = 28; PAD_B = 18; PAD_T = 24; PAD_R = 8
    PX = ox + PAD_L
    PY = oy + PAD_T
    PW = MAP_W - PAD_L - PAD_R
    PH = MAP_H - PAD_T - PAD_B

    # ── World range (auto-fit + margin) ─────────────────────────
    all_wx = [plant_wx]
    all_wy = [plant_wy]
    if robot_wx is not None:
        all_wx.append(robot_wx)
        all_wy.append(robot_wy)
    for pt in world_trail:
        all_wx.append(pt[0])
        all_wy.append(pt[1])

    span = max(max(all_wx) - min(all_wx), max(all_wy) - min(all_wy), 1.0)
    margin = span * 0.30 + 0.3
    wx_min = min(all_wx) - margin
    wx_max = max(all_wx) + margin
    wy_min = min(all_wy) - margin
    wy_max = max(all_wy) + margin

    # Keep aspect ratio square
    wx_span = wx_max - wx_min
    wy_span = wy_max - wy_min
    if wx_span > wy_span:
        diff = wx_span - wy_span
        wy_min -= diff / 2; wy_max += diff / 2
    else:
        diff = wy_span - wx_span
        wx_min -= diff / 2; wx_max += diff / 2

    wx_span = wx_max - wx_min
    wy_span = wy_max - wy_min

    def w2p(wx, wy):
        """World coords → canvas pixel."""
        px = PX + int((wx - wx_min) / wx_span * PW)
        py = PY + PH - int((wy - wy_min) / wy_span * PH)
        return (int(px), int(py))

    # ── Dashed grid lines at 0.5 m intervals ────────────────────
    def _nice_ticks(lo, hi, step=0.5):
        start = math.floor(lo / step) * step
        t = start
        while t <= hi + 1e-6:
            yield round(t, 2)
            t += step

    DASH, GAP = 5, 4
    for gx in _nice_ticks(wx_min, wx_max):
        p1 = w2p(gx, wy_min); p2 = w2p(gx, wy_max)
        col = (65, 65, 85) if abs(gx) > 1e-3 else (90, 90, 110)
        y = p1[1]
        while y > p2[1]:
            cv2.line(canvas, (p1[0], y), (p1[0], max(y-DASH, p2[1])), col, 1)
            y -= DASH + GAP
        # X tick label
        lp = w2p(gx, wy_min)
        cv2.putText(canvas, f"{gx:.1f}", (lp[0]-8, oy+MAP_H-4),
                    _FONT, 0.22, (80, 80, 105), 1)

    for gy in _nice_ticks(wy_min, wy_max):
        p1 = w2p(wx_min, gy); p2 = w2p(wx_max, gy)
        col = (65, 65, 85) if abs(gy) > 1e-3 else (90, 90, 110)
        x = p1[0]
        while x < p2[0]:
            cv2.line(canvas, (x, p1[1]), (min(x+DASH, p2[0]), p1[1]), col, 1)
            x += DASH + GAP
        # Y tick label
        lp = w2p(wx_min, gy)
        cv2.putText(canvas, f"{gy:.1f}", (ox+1, lp[1]+4),
                    _FONT, 0.22, (80, 80, 105), 1)

    # Solid zero axes
    if wy_min < 0 < wy_max:
        p1 = w2p(wx_min, 0); p2 = w2p(wx_max, 0)
        cv2.line(canvas, p1, p2, (80, 80, 100), 1)
    if wx_min < 0 < wx_max:
        p1 = w2p(0, wy_min); p2 = w2p(0, wy_max)
        cv2.line(canvas, p1, p2, (80, 80, 100), 1)

    # ── Trail (robot path) ───────────────────────────────────────
    trail_pts = [w2p(pt[0], pt[1]) for pt in world_trail]
    for i in range(1, len(trail_pts)):
        alpha = 0.4 + 0.6 * (i / max(len(trail_pts)-1, 1))
        col = (int(30*alpha), int(180*alpha), int(80*alpha))
        cv2.line(canvas, trail_pts[i-1], trail_pts[i], col, 1)

    # ── Plant (target) green circle ──────────────────────────────
    pp = w2p(plant_wx, plant_wy)
    cv2.circle(canvas, pp, 13, (0, 170, 55), -1)
    cv2.circle(canvas, pp, 13, (80, 255, 120), 2)
    _txt(canvas, "PLANT", (pp[0]+16, pp[1]+4), 0.30, (80, 255, 120))

    # ── Robot blue circle + red yaw arrow ───────────────────────
    scol = STATE_COLOR.get(state, (180, 180, 180))
    if robot_wx is not None and robot_wy is not None:
        rp = w2p(robot_wx, robot_wy)
        rp = (max(PX+6, min(PX+PW-6, rp[0])),
              max(PY+6,  min(PY+PH-6, rp[1])))

        # Line robot → plant (dashed guide)
        pp_c = w2p(plant_wx, plant_wy)
        cv2.line(canvas, rp, pp_c, (50, 50, 70), 1)

        # Distance label at midpoint
        dist = math.sqrt((robot_wx - plant_wx)**2 + (robot_wy - plant_wy)**2)
        mid  = ((rp[0]+pp_c[0])//2, (rp[1]+pp_c[1])//2)
        _txt(canvas, f"{dist:.2f}m", (mid[0]+4, mid[1]-4), 0.28, (120, 120, 160))

        # Robot circle (blue)
        cv2.circle(canvas, rp, 8, (220, 110, 40), -1)
        cv2.circle(canvas, rp, 8, (255, 255, 255), 1)

        # Yaw arrow (red, like matplotlib)
        yaw = robot_yaw if robot_yaw is not None else 0.0
        arrow_len = max(18, int(PW * 0.10))
        ax = rp[0] + int(math.sin(yaw) * arrow_len)
        ay = rp[1] - int(math.cos(yaw) * arrow_len)
        # Clamp arrow tip
        ax = max(PX, min(PX+PW, ax)); ay = max(PY, min(PY+PH, ay))
        cv2.arrowedLine(canvas, rp, (ax, ay), (50, 80, 220), 2, tipLength=0.35)

        # Coord label near robot
        lx = rp[0] + 10
        ly = rp[1] - 10
        if lx + 70 > PX + PW: lx = rp[0] - 72
        _txt(canvas, f"X={robot_wx:+.2f}", (lx, ly),      0.28, scol)
        _txt(canvas, f"Y={robot_wy:+.2f}", (lx, ly + 13), 0.28, scol)

    # ── Header title ─────────────────────────────────────────────
    yaw_deg = math.degrees(robot_yaw) if robot_yaw is not None else 0.0
    rwx = robot_wx if robot_wx is not None else 0.0
    rwy = robot_wy if robot_wy is not None else 0.0
    title = f"Pos: X={rwx:+.2f}, Y={rwy:+.2f}, Yaw={yaw_deg:+.1f}"
    _txt(canvas, title, (ox+4, oy+16), 0.30, (170, 170, 210))

    # ── HeadingPID lock indicator (top-right corner of map) ──────
    # (ยังคงไว้ด้านบนขวาของ MAP panel เหมือนเดิม)
    return w2p   # return helper so caller can use if needed


def draw_mission_panel(canvas, ox, oy, ms_state, ms_events,
                       plant_ok, capture_ok):
    """Right panel: mission state progress bar + event log."""
    cv2.rectangle(canvas, (ox,oy), (ox+INFO_W, oy+INFO_H), (10,10,14), -1)
    cv2.rectangle(canvas, (ox,oy), (ox+INFO_W, oy+INFO_H), (40,40,55),  1)

    _txt(canvas, "MISSION", (ox+8, oy+18), 0.50, (160,160,200), 2)

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

    cur_name = MS_NAME.get(ms_state, f"MS{ms_state}")
    cur_col  = MS_COLOR.get(ms_state, (200,200,200))
    _txt(canvas, cur_name, (ox+8, oy+85), 0.65, cur_col, 2)

    pk_col = (0,230,100) if plant_ok else (80,80,80)
    ck_col = (0,230,100) if capture_ok else (80,80,80)
    cv2.circle(canvas, (ox+8+6,  oy+108), 6, pk_col, -1)
    cv2.circle(canvas, (ox+8+26, oy+108), 6, ck_col, -1)
    _txt(canvas, "plant", (ox+18,  oy+113), 0.30, pk_col)
    _txt(canvas, "capture",(ox+36, oy+113), 0.30, ck_col)

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

        self._v_state   = float(STATE_SEARCH)
        self._v_x       = None
        self._v_z       = None
        self._v_yaw     = None
        self._v_v       = 0.0
        self._v_w       = 0.0
        self._v_stable  = 0
        self._v_stuck   = 0
        self._v_bearing = None

        self._cmd_v = 0.0
        self._cmd_w = 0.0

        self._pid_error = 0.0
        self._pid_u     = 0.0
        self._pid_deriv = 0.0
        self._pid_yaw_ref = 0.0
        self._pid_yaw     = 0.0

        self._ms_state   = 0
        self._ms_events  = collections.deque(maxlen=12)
        self._t0         = time.monotonic()
        self._plant_ok   = False
        self._capture_ok = False

        # ── World-frame position ─────────────────────────────────
        # Plant (tag) ตรึงที่ world (0, 0)
        # Robot อยู่ที่ (-tag_x, -tag_z) เมื่อเห็น tag
        self._robot_wx:  float | None = None   # world X
        self._robot_wy:  float | None = None   # world Y
        self._robot_yaw: float | None = None   # robot heading (from tag yaw)

        # Trail เก็บ (world_x, world_y) เป็น float
        self._world_trail: list[tuple[float, float]] = []

    def _init_rings(self):
        n = HIST_N
        self.rb_z    = Ring(n, 2.0)
        self.rb_x    = Ring(n)
        self.rb_bear = Ring(n)
        self.rb_yaw  = Ring(n)
        self.rb_v    = Ring(n)
        self.rb_w    = Ring(n)
        self.rb_herr = Ring(n)
        self.rb_hu   = Ring(n)
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

            # Push to rings
            self.rb_z.push(self._v_z if self._v_z is not None else 2.0)
            self.rb_x.push(self._v_x if self._v_x is not None else 0.0)
            self.rb_v.push(self._v_v)
            self.rb_w.push(self._v_w)
            if self._v_bearing is not None:
                self.rb_bear.push(self._v_bearing)
            if self._v_yaw is not None:
                self.rb_yaw.push(self._v_yaw)

            # ── World-frame position update ──────────────────────
            # Plant อยู่ที่ (0,0) ในกรอบโลก
            # Robot = (-tag_x, -tag_z)  [tag_x=ซ้าย/ขวา, tag_z=ระยะหน้า]
            if self._v_x is not None and self._v_z is not None:
                wx = -float(self._v_x)
                wy = -float(self._v_z)
                self._robot_wx  = wx
                self._robot_wy  = wy
                self._robot_yaw = float(self._v_yaw) if self._v_yaw is not None else 0.0

                # Append ถ้าขยับพอ (threshold 0.01 m) เพื่อไม่ให้ trail หนาแน่นเกิน
                MIN_DIST = 0.01
                if (not self._world_trail or
                        math.hypot(wx - self._world_trail[-1][0],
                                   wy - self._world_trail[-1][1]) >= MIN_DIST):
                    self._world_trail.append((wx, wy))
                    if len(self._world_trail) > HIST_N:
                        self._world_trail.pop(0)

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
        if "planting" in msg.data:
            self._plant_ok = True
        if "cabbage" in msg.data:
            self._capture_ok = True
        if "FINISH" in msg.data:
            self._plant_ok = self._capture_ok = False

    def _cb_ms(self, msg: Int32):
        self._ms_state = msg.data
        if msg.data in (5, 7):
            self._plant_ok = False
        if msg.data in (9, 11, 13):
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

        cv2.rectangle(cam, (0,0), (CAM_W, 32), (0,0,0), -1)
        _txt(cam, STATE_NAME.get(v_state,"?"), (6,23), 0.65, scol, 2)
        _txt(cam, f"stable:{self._v_stable:3d}  stuck:{self._v_stuck:2d}",
             (180, 23), 0.38, (120,120,120))

        if tag_z is not None:
            cv2.rectangle(cam, (0, CAM_H-36), (CAM_W, CAM_H), (0,0,0,128), -1)
            _txt(cam, f"x={tag_x:+.3f}m", (6, CAM_H-21), 0.38, (0,220,220))
            _txt(cam, f"z={tag_z:.3f}m",  (110,CAM_H-21), 0.38, (80,255,80))
            _txt(cam, f"bear={math.degrees(bearing):+.1f}°",
                 (210, CAM_H-21), 0.38, (0,190,255))
            _txt(cam, f"yaw={math.degrees(tag_yaw):+.1f}°",
                 (350, CAM_H-21), 0.38, (255,200,50))

        canvas[0:CAM_H, 0:CAM_W] = cam

        # ── World-frame Map panel ─────────────────────────────────
        draw_worldmap(
            canvas, MAP_X, 0,
            robot_wx  = self._robot_wx,
            robot_wy  = self._robot_wy,
            robot_yaw = self._robot_yaw,
            plant_wx  = 0.0,
            plant_wy  = 0.0,
            world_trail = self._world_trail,
            state       = v_state,
        )

        # HeadingPID lock indicator (วางที่มุมบนขวาของ map panel)
        lock_active = abs(self._cmd_v) > 0.02 and abs(self._cmd_w) < 0.05
        lx = MAP_X + MAP_W - 80; ly = 10
        lc = (0,230,100) if lock_active else (60,60,80)
        cv2.rectangle(canvas, (lx,ly), (lx+76, ly+22), (0,0,0), -1)
        cv2.rectangle(canvas, (lx,ly), (lx+76, ly+22), lc, 1)
        _txt(canvas, "HDG LOCK" if lock_active else "HDG FREE",
             (lx+4, ly+16), 0.35, lc)

        # HeadingPID mini display (ล่างของ map panel)
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

        cv2.putText(canvas, f"frame #{self._frame_cnt}",
                    (WIN_W-120, WIN_H-6), _FONT, 0.28, (50,50,60), 1)

        cv2.imshow(self._win, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            raise KeyboardInterrupt
        if key == ord('c'):
            self._world_trail.clear()
            self._robot_wx = self._robot_wy = None
            self.get_logger().info("[DEBUG] world trail cleared")


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