#!/usr/bin/env python3


import math
import collections

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Int32

import cv2
import numpy as np
from pupil_apriltags import Detector

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

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

# ════════════════════════════════════════════════════════════════
# Tuning
# ════════════════════════════════════════════════════════════════
Z_ALIGN   = 0.47
Z_FORWARD = 0.47
Z_STOP    = 0.25

APPROACH_KP        = 0.80
APPROACH_KI        = 0.00
APPROACH_KD        = 0.12
APPROACH_W_MAX     = 0.60
APPROACH_V_BASE    = 0.50
APPROACH_BEAR_DEAD = math.radians(2.0)

ALIGN_KP       = 0.30
ALIGN_KI       = 0.00
ALIGN_KD       = 0.15
ALIGN_W_MAX    = 0.22
ALIGN_V        = 0.10
ALIGN_YAW_DEAD = math.radians(1.2)
ALIGN_BRAKE_ZONE = math.radians(8.0)
ALIGN_STABLE_REQUIRED = 6
ALIGN_YAW_TIMEOUT     = 1.5

FORWARD_V = 0.20
SEARCH_W  = 0.00

SCAN_BACK_W       = 0.18
SCAN_BACK_MAX_DEG = 120.0
SCAN_BACK_TIMEOUT = 10.0

REVERSE_V = -0.12
REVERSE_T = 2.0

STUCK_SEC   = 3.0
STUCK_DELTA = 0.03
STUCK_ZONE  = 1.50
FORWARD_STUCK_SEC   = 2.0
FORWARD_STUCK_DELTA = 0.01

DETECT_EVERY    = 1
STABLE_REQUIRED = 2
LOST_TIMEOUT    = 4.0
MISS_GRACE      = 5

LOCK_LOST_MAX    = 30
PRELOCK_REQUIRED = 2

YOLO_EVERY      = 3
YOLO_CONF       = 0.25
YOLO_MARGIN     = 80
YOLO_STALE_MAX  = 15
YOLO_CLASS_NAME = None

SPATIAL_LOCK_PIXEL_THRESH = 120
SPATIAL_LOCK_Z_THRESH     = Z_ALIGN + 0.10

_FONT = cv2.FONT_HERSHEY_SIMPLEX


# ════════════════════════════════════════════════════════════════
# PID
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

    @property
    def z_raw(self):
        return float(np.median(self._zb)) if self._zb else None


# ════════════════════════════════════════════════════════════════
# Spatial Lock
# ════════════════════════════════════════════════════════════════
class SpatialLock:
    def __init__(self, pixel_thresh=SPATIAL_LOCK_PIXEL_THRESH):
        self.pixel_thresh = pixel_thresh
        self.cx = None
        self.cy = None
        self.active = False

    def reset(self):
        self.cx = None; self.cy = None; self.active = False

    def update_from_det(self, det):
        c = det.corners.astype(float)
        self.cx = float(np.mean(c[:, 0]))
        self.cy = float(np.mean(c[:, 1]))
        self.active = True

    def pick_closest(self, dets):
        if not self.active or not dets:
            return None, float('inf')
        best_det  = None
        best_dist = float('inf')
        for det in dets:
            c  = det.corners.astype(float)
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))
            d  = math.hypot(cx - self.cx, cy - self.cy)
            if d < best_dist:
                best_dist = d
                best_det  = det
        if best_dist <= self.pixel_thresh:
            return best_det, best_dist
        return None, best_dist


# ════════════════════════════════════════════════════════════════
# YOLO Guard
# ════════════════════════════════════════════════════════════════
class YOLOGuard:
    def __init__(self, model_path, conf=YOLO_CONF, margin=YOLO_MARGIN,
                 stale_max=YOLO_STALE_MAX, class_name=YOLO_CLASS_NAME):
        self.conf = conf; self.margin = margin
        self.stale_max = stale_max; self.class_name = class_name
        self.bbox = None; self.bbox_age = 0
        self.yolo_ok = False; self.disabled = False

        if not YOLO_AVAILABLE:
            self.disabled = True; return
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            print(f"[YOLOGuard] load failed: {e}")
            self.disabled = True

    def update(self, frame):
        if self.disabled: return None
        results = self.model(frame, conf=self.conf, verbose=False)
        best_box = None; best_conf = 0.0
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0]); conf = float(box.conf[0])
                if self.class_name is not None:
                    if self.model.names.get(cls_id, '') != self.class_name: continue
                if conf > best_conf:
                    best_conf = conf
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    best_box = (int(x1), int(y1), int(x2), int(y2))
        if best_box:
            self.bbox = best_box; self.bbox_age = 0; self.yolo_ok = True
        else:
            self.bbox_age += 1
            if self.bbox_age > self.stale_max:
                self.bbox = None; self.yolo_ok = False
        return self.bbox

    def tick(self):
        if self.disabled: return
        self.bbox_age += 1
        if self.bbox_age > self.stale_max:
            self.bbox = None; self.yolo_ok = False

    def score_tag_proximity(self, corners):
        info = {'score': float('inf'), 'reason': ''}
        if self.disabled: info['reason'] = 'disabled'; return 0.0, info
        if self.bbox is None: info['reason'] = 'no_bbox'; return float('inf'), info
        x1, y1, x2, y2 = self.bbox; m = self.margin
        centroid_x = float(np.mean(corners[:, 0]))
        centroid_y = float(np.mean(corners[:, 1]))
        if not (x1 - m <= centroid_x <= x2 + m):
            info['reason'] = 'x_out'; return float('inf'), info
        score = abs(centroid_y - y1)
        info['score'] = score; info['reason'] = 'ok'
        return score, info


# ════════════════════════════════════════════════════════════════
# Node
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
        self.get_logger().info(
            f"AprilTagServo | YOLO={'ON' if not self.yolo.disabled else 'OFF'}"
            f" | cmd→{self._cmd_topic} | tag_id={self.target_tag_id}"
            f" | x_offset={self._target_x_offset:.3f}m"
            f" | align_x_thresh={self._align_x_thresh:.3f}m"
            f" | target_yaw={math.degrees(self._align_target_yaw):.1f}°"
            f" | Z_ALIGN={Z_ALIGN:.2f}m")

    # ── Parameters ───────────────────────────────────────────────
    def _declare_params(self):
        d = self.declare_parameter
        d('image_width',  640); d('image_height', 480)
        d('invert_x',   False); d('invert_yaw',   False)
        d('tag_size',   0.042)
        d('max_linear', 0.60);  d('max_angular',  1.0)
        d('cmd_topic',  '/cmd_vel_vision')
        d('yolo_model', 'best.pt')
        d('target_tag_id', -1)
        d('fx', 535.7020623190093)
        d('fy', 536.2286757565952)
        d('cx', 328.92740826965303)
        d('cy', 244.82018265933337)
        d('dist_k1',  0.2364165900471923)
        d('dist_k2', -0.6070316832395167)
        d('dist_p1',  0.0018016917089935594)
        d('dist_p2',  0.0076473276069728155)
        d('dist_k3',  0.5044612186719518)
        d('target_x_offset', 0.05)
        d('align_x_thresh', 0.05)
        d('align_target_yaw_deg', 0.5)

    def _load_params(self):
        g = self.get_parameter
        self.W = g('image_width').value; self.H = g('image_height').value
        self.invert_x    = g('invert_x').value
        self.invert_yaw  = g('invert_yaw').value
        self.tag_size    = g('tag_size').value
        self.max_linear  = g('max_linear').value
        self.max_angular = g('max_angular').value
        self._cmd_topic        = g('cmd_topic').value
        self._yolo_model       = g('yolo_model').value
        self.target_tag_id     = int(g('target_tag_id').value)
        self.fx  = g('fx').value;  self.fy  = g('fy').value
        self.cx0 = g('cx').value;  self.cy0 = g('cy').value
        self._target_x_offset  = g('target_x_offset').value
        self._align_x_thresh   = g('align_x_thresh').value
        self._align_target_yaw = math.radians(g('align_target_yaw_deg').value)

    def _build_camera_model(self):
        g = self.get_parameter
        self.K = np.array(
            [[self.fx, 0, self.cx0], [0, self.fy, self.cy0], [0, 0, 1]],
            np.float64)
        self.D = np.array([[
            g('dist_k1').value, g('dist_k2').value,
            g('dist_p1').value, g('dist_p2').value,
            g('dist_k3').value]], np.float64)
        self.K_new, _ = cv2.getOptimalNewCameraMatrix(
            self.K, self.D, (self.W, self.H), 1, (self.W, self.H))
        self.detector = Detector(
            families="tagStandard52h13",
            nthreads=2,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25
        )

    def _build_pids(self):
        self.pid_b = PID(APPROACH_KP, APPROACH_KI, APPROACH_KD,
                         lo=-APPROACH_W_MAX, hi=APPROACH_W_MAX)
        self.pid_y = PID(ALIGN_KP, ALIGN_KI, ALIGN_KD,
                         lo=-ALIGN_W_MAX, hi=ALIGN_W_MAX)

    def _build_yolo(self):
        self.yolo = YOLOGuard(self._yolo_model)

    def _build_ros(self):
        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self._image_callback, 10)
        self.cmd_pub      = self.create_publisher(Twist,             self._cmd_topic,               1)
        self.dbg_pub      = self.create_publisher(Float32MultiArray, '/vision_debug',               5)
        self.plant_pub    = self.create_publisher(Int32,             '/apriltag/planting_distance', 5)
        self.gap_pub      = self.create_publisher(Int32,             '/apriltag/gap_type',          5)
        self.interval_pub = self.create_publisher(Int32,             '/apriltag/cabbage_interval',  5)
        self.pose_pub     = self.create_publisher(Float32MultiArray, '/apriltag/pose',              1)

    def _init_state(self):
        self.state    = STATE_SEARCH; self.frame_cnt = 0
        self.tag      = TagSmoother(alpha=0.55, window=3)
        self.n_stable = 0; self.n_miss = 0; self.last_t = None
        self.published = False; self._last_tag_info = None
        self._sframe = 0; self._last_wdir = 1.0
        self._sb_t0 = None; self._sb_last_t = None
        self._sb_turned = 0.0; self._sb_dir = 1.0
        self._rv_t0 = None
        self._stuck_z = None; self._stuck_t = None; self._stuck_n = 0
        self._fwd_stuck_z = None; self._fwd_stuck_t = None
        self._yolo_frame = 0
        self.locked_tag_id = None; self.lock_lost_frames = 0
        self.prelock_id = None; self.prelock_count = 0
        self._align_stable_frames = 0
        self._align_yaw_sign = -1.0
        self._align_yaw_ok_t = None
        self.spatial = SpatialLock(pixel_thresh=SPATIAL_LOCK_PIXEL_THRESH)

    # ── Helpers ──────────────────────────────────────────────────
    def _full_unlock(self, frame=None):
        self.locked_tag_id    = None
        self.lock_lost_frames = 0
        self.prelock_id       = None
        self.prelock_count    = 0
        self._yolo_frame      = 0
        self.spatial.reset()
        if frame is not None:
            self.yolo.update(frame)

    def _should_use_spatial(self):
        if not self.spatial.active:
            return False
        z_now = self.tag.z_raw
        if z_now is not None and z_now <= SPATIAL_LOCK_Z_THRESH:
            return True
        if self.state in (STATE_ALIGN, STATE_FORWARD):
            return True
        return False

    # ── Detection ────────────────────────────────────────────────
    def _detect(self, frame):
        undist = cv2.undistort(frame, self.K, self.D)
        gray   = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)
        try:
            dets = self.detector.detect(
                gray, estimate_tag_pose=True,
                camera_params=(self.fx, self.fy, self.cx0, self.cy0),
                tag_size=self.tag_size)
        except Exception as e:
            self.get_logger().warn(f"detect: {e}"); return None

        if not dets:
            if self.locked_tag_id is not None:
                self.lock_lost_frames += 1
                if self.lock_lost_frames > LOCK_LOST_MAX:
                    self.get_logger().warn(
                        f"[LOCK] tag {self.locked_tag_id} lost → unlock")
                    self._full_unlock(frame)
            return None

        if self.locked_tag_id is not None:
            same_id = [d for d in dets if d.tag_id == self.locked_tag_id]
            if not same_id:
                self.lock_lost_frames += 1
                if self.lock_lost_frames > LOCK_LOST_MAX:
                    self.get_logger().warn(
                        f"[LOCK] tag {self.locked_tag_id} lost → unlock")
                    self._full_unlock(frame)
                return None

            self.lock_lost_frames = 0

            if len(same_id) == 1:
                best = same_id[0]
                self.get_logger().info(
                    "[SPATIAL] single det → direct lock",
                    throttle_duration_sec=1.0)
            elif self._should_use_spatial():
                best, dist_px = self.spatial.pick_closest(same_id)
                if best is None:
                    best = min(same_id,
                               key=lambda d: float(d.pose_t.flatten()[2]))
                    self.get_logger().warn(
                        f"[SPATIAL] jump {dist_px:.0f}px > "
                        f"{SPATIAL_LOCK_PIXEL_THRESH}px → fallback nearest-z",
                        throttle_duration_sec=0.5)
                else:
                    self.get_logger().info(
                        f"[SPATIAL] locked | dist={dist_px:.1f}px"
                        f" | dets={len(same_id)}",
                        throttle_duration_sec=1.0)
            else:
                best = min(same_id, key=lambda d: float(d.pose_t.flatten()[2]))

            self.spatial.update_from_det(best)

        else:
            if self.target_tag_id >= 0:
                target = [d for d in dets if d.tag_id == self.target_tag_id]
                if not target: return None
                best = min(target, key=lambda d: float(d.pose_t.flatten()[2]))
            else:
                self._yolo_frame += 1
                if self._yolo_frame % YOLO_EVERY == 0:
                    self.yolo.update(frame)
                else:
                    self.yolo.tick()

                scored = []
                for det in dets:
                    score, info = self.yolo.score_tag_proximity(
                        det.corners.astype(int))
                    if score < float('inf'):
                        scored.append((score, det))

                self.get_logger().info(
                    f"[DBG] dets={len(dets)} scored={len(scored)}"
                    f" yolo_bbox={self.yolo.bbox}"
                    f" yolo_age={self.yolo.bbox_age}",
                    throttle_duration_sec=1.0)

                if not scored:
                    if self.yolo.disabled or self.yolo.bbox is None:
                        best = min(dets, key=lambda d: float(d.pose_t.flatten()[2]))
                        self.get_logger().info(
                            f"[DBG] YOLO fallback → tag_id={best.tag_id}"
                            f" z={float(best.pose_t.flatten()[2]):.3f}m",
                            throttle_duration_sec=1.0)
                    else:
                        return None
                else:
                    scored.sort(key=lambda t: t[0])
                    best = scored[0][1]

            if self.prelock_id == best.tag_id:
                self.prelock_count += 1
            else:
                self.prelock_id = best.tag_id; self.prelock_count = 1
            if self.prelock_count < PRELOCK_REQUIRED:
                return None

            self.locked_tag_id    = best.tag_id
            self.lock_lost_frames = 0
            self.prelock_id = None; self.prelock_count = 0
            self.spatial.update_from_det(best)

            tx_lock = float(best.pose_t.flatten()[0]) * (-1.0 if self.invert_x else 1.0)
            if tx_lock > self._align_x_thresh:
                self._align_yaw_sign = 1.0; _side = "LEFT"
            else:
                self._align_yaw_sign = -1.0; _side = "RIGHT/CENTER"
            self.get_logger().info(
                f"[LOCK] ✓ tag_id={self.locked_tag_id}"
                f"  z={float(best.pose_t.flatten()[2]):.3f}m"
                f"  tx={tx_lock:.3f}m → {_side}"
                f"  yaw_sign={self._align_yaw_sign:+.0f}")

        tx, _, tz = best.pose_t.flatten()
        x_raw = float(tx) * (-1.0 if self.invert_x else 1.0)
        z_m   = float(tz)
        R     = best.pose_R
        yaw   = math.atan2(float(R[0, 2]), float(R[2, 2])) \
                * (-1.0 if self.invert_yaw else 1.0)

        x_m     = x_raw - self._target_x_offset
        bearing = math.atan2(x_m, max(z_m + 0.25, 0.05))

        tag_id = best.tag_id
        self._last_tag_info = {
            'ab': tag_id // 1000, 'c': (tag_id // 100) % 10, 'de': tag_id % 100,
            'x': x_raw, 'z': z_m, 'bearing': bearing,
        }
        self.last_t = self.get_clock().now()
        return x_m, z_m, yaw

    # ── State machine ────────────────────────────────────────────
    def _go(self, new_state, force=False):
        if new_state == self.state:
            self._sframe += 1; return
        if not force:
            if self.state not in (STATE_SEARCH, STATE_DONE,
                                  STATE_SCAN_BACK, STATE_REVERSE) \
               and self._sframe < 2:
                return
        self.get_logger().info(
            f"[STATE] {STATE_NAME[self.state]} → {STATE_NAME[new_state]}"
            f"  ({self._sframe} fr{'  FORCE' if force else ''})")
        self.state = new_state; self._sframe = 0
        self._stuck_z = None; self._stuck_t = None

        if new_state == STATE_APPROACH:
            self.pid_b.reset()

        if new_state == STATE_FORWARD:
            self._fwd_stuck_z = None; self._fwd_stuck_t = None

        if new_state == STATE_ALIGN:
            self.pid_y.reset()
            self._align_stable_frames = 0
            self._align_yaw_ok_t = None
            self._stuck_n = 0          # ← reset stuck counter ป้องกัน REVERSE ทันที
            self._stuck_z = None
            self._stuck_t = None
            self.get_logger().info(
                f"[ALIGN] entry  yaw_sign={self._align_yaw_sign:+.0f}"
                f"  z={self.tag.z:.3f}m  tag_x={self.tag.x:.3f}m"
                f"  spatial={'ACTIVE' if self.spatial.active else 'OFF'}")

        if new_state == STATE_SEARCH:
            self.published = False
            self._full_unlock()
            self._align_yaw_sign = -1.0

        if new_state == STATE_DONE and not self.published:
            info = self._last_tag_info
            if info is not None:
                z_stop       = self.tag.z if self.tag.z is not None else info['z']
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
                    f"[DONE] AB={info['ab']} C={info['c']} DE={info['de']}"
                    f"  z={z_stop:.3f}m  x_raw={info['x']:.3f}m"
                    f"  offset={self._target_x_offset:.3f}m")

        if new_state == STATE_REVERSE:
            self._rv_t0 = self.get_clock().now().nanoseconds / 1e9
        if new_state == STATE_SCAN_BACK:
            self._sb_t0     = self.get_clock().now().nanoseconds / 1e9
            self._sb_last_t = self._sb_t0
            self._sb_turned = 0.0; self._sb_dir = -self._last_wdir

    def _next_nav_state(self, z):
        z_raw = self.tag.z_raw if self.tag.z_raw is not None else z
        z_ema = self.tag.z     if self.tag.z     is not None else z
        z_dec = min(z_raw, z_ema)

        if z_dec <= Z_STOP:
            return STATE_DONE
        if self.state == STATE_FORWARD:
            return STATE_FORWARD
        if self.state == STATE_ALIGN:
            return STATE_ALIGN
        if z_dec <= Z_ALIGN:
            return STATE_ALIGN
        return STATE_APPROACH

    def _check_stuck(self, z):
        if z > STUCK_ZONE:
            self._stuck_z = None; self._stuck_t = None; return
        now = self.get_clock().now().nanoseconds / 1e9
        if self._stuck_z is None:
            self._stuck_z = z; self._stuck_t = now; return
        if now - self._stuck_t > STUCK_SEC:
            if self._stuck_z - z < STUCK_DELTA:
                self._stuck_n += 1
                self.get_logger().warn(f"[STUCK #{self._stuck_n}] → REVERSE")
                self._go(STATE_REVERSE, force=True)
            self._stuck_z = z; self._stuck_t = now

    def _check_forward_stuck(self, z):
        now = self.get_clock().now().nanoseconds / 1e9
        if self._fwd_stuck_z is None:
            self._fwd_stuck_z = z; self._fwd_stuck_t = now; return
        if now - self._fwd_stuck_t > FORWARD_STUCK_SEC:
            delta = self._fwd_stuck_z - z
            if delta < FORWARD_STUCK_DELTA:
                self._stuck_n += 1
                self.get_logger().warn(
                    f"[FWD STUCK #{self._stuck_n}]"
                    f" z_delta={delta:.3f}m < {FORWARD_STUCK_DELTA}m → REVERSE")
                self._fwd_stuck_z = None; self._fwd_stuck_t = None
                self._go(STATE_REVERSE, force=True)
                return
            self._fwd_stuck_z = z; self._fwd_stuck_t = now

    # ── Control laws ─────────────────────────────────────────────
    def _control(self, x, z, yaw, bearing, cmd):
        s = self.state

        if s == STATE_SEARCH:
            cmd.linear.x  = 0.0
            cmd.angular.z = SEARCH_W
            self._last_wdir = 1.0

        elif s == STATE_APPROACH:
            if abs(bearing) < APPROACH_BEAR_DEAD:
                self.pid_b.reset()
                cmd.angular.z = 0.0
            else:
                raw_w = self.pid_b.update(bearing)
                cmd.angular.z = -raw_w
            if abs(cmd.angular.z) > 0.01:
                self._last_wdir = math.copysign(1.0, cmd.angular.z)
            align_factor = max(0.2, math.cos(bearing))
            cmd.linear.x = APPROACH_V_BASE * align_factor
            self._check_stuck(z)

        elif s == STATE_ALIGN:
            yaw_error = yaw - self._align_target_yaw
            now_sec   = self.get_clock().now().nanoseconds / 1e9

            if abs(yaw_error) < ALIGN_YAW_DEAD:
                cmd.angular.z = 0.0
                cmd.linear.x  = 0.0
                self.pid_y.reset()
                self._align_stable_frames += 1
                if self._align_yaw_ok_t is None:
                    self._align_yaw_ok_t = now_sec
                yaw_ok_dur = now_sec - self._align_yaw_ok_t
                self.get_logger().info(
                    f"[ALIGN] ✓ stable {self._align_stable_frames}"
                    f"  yaw={math.degrees(yaw):.1f}°"
                    f"  err={math.degrees(yaw_error):.1f}°"
                    f"  ok_dur={yaw_ok_dur:.1f}s")
                if self._align_stable_frames >= ALIGN_STABLE_REQUIRED \
                        or yaw_ok_dur >= ALIGN_YAW_TIMEOUT:
                    self.get_logger().info(
                        f"[ALIGN] → FORWARD"
                        f"  (stable={self._align_stable_frames}"
                        f"  ok_dur={yaw_ok_dur:.1f}s)")
                    self._go(STATE_FORWARD, force=True)
            else:
                if abs(yaw_error) > ALIGN_YAW_DEAD * 2:
                    self._align_stable_frames = 0
                    self._align_yaw_ok_t = None

                raw_w = self.pid_y.update(yaw_error)

                if abs(yaw_error) < ALIGN_BRAKE_ZONE:
                    brake_scale = abs(yaw_error) / ALIGN_BRAKE_ZONE
                    raw_w *= brake_scale

                cmd.angular.z = raw_w * self._align_yaw_sign

                if abs(yaw_error) < ALIGN_BRAKE_ZONE:
                    cmd.linear.x = 0.0
                else:
                    cmd.linear.x = ALIGN_V * max(0.0, math.cos(yaw_error))

            if abs(cmd.angular.z) > 0.01:
                self._last_wdir = math.copysign(1.0, cmd.angular.z)
            self._check_stuck(z)

        elif s == STATE_FORWARD:
            cmd.linear.x  = FORWARD_V
            cmd.angular.z = 0.0
            self._check_forward_stuck(z)

        elif s == STATE_SCAN_BACK:
            cmd.linear.x = 0.0
            now  = self.get_clock().now().nanoseconds / 1e9
            elap = now - (self._sb_t0 or now)
            dt   = now - (self._sb_last_t or now); self._sb_last_t = now
            if elap > SCAN_BACK_TIMEOUT or \
               self._sb_turned >= math.radians(SCAN_BACK_MAX_DEG):
                cmd.angular.z = 0.0
                self._go(STATE_SEARCH, force=True); return
            cmd.angular.z    = self._sb_dir * SCAN_BACK_W
            self._sb_turned += abs(cmd.angular.z) * max(dt, 0.0)

        elif s == STATE_REVERSE:
            now  = self.get_clock().now().nanoseconds / 1e9
            elap = now - (self._rv_t0 or now)
            if elap < REVERSE_T:
                cmd.linear.x = REVERSE_V; cmd.angular.z = 0.0
            else:
                cmd.linear.x = cmd.angular.z = 0.0
                self.tag.reset(); self.n_stable = 0; self.last_t = None
                self._go(STATE_SEARCH, force=True)

        else:  # DONE
            cmd.linear.x = cmd.angular.z = 0.0

    # ── Main callback ────────────────────────────────────────────
    def _image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None: return
        except Exception as e:
            self.get_logger().warn(f"decode: {e}", throttle_duration_sec=1.0)
            return

        self.frame_cnt += 1
        cmd = Twist()
        x = z = yaw = bearing = None

        if self.frame_cnt % DETECT_EVERY == 0:
            result = self._detect(frame)
            if result is not None:
                rx, rz, ry = result
                self.tag.push(rx, rz, ry)
                self.n_stable += 1; self.n_miss = 0
                if self.state == STATE_SCAN_BACK and self.n_stable >= STABLE_REQUIRED:
                    self._go(STATE_APPROACH, force=True)
            else:
                self.n_miss += 1
                if self.n_miss > MISS_GRACE:
                    self.n_stable = 0

        if self.state == STATE_APPROACH:
            if self.last_t is not None:
                lost = (self.get_clock().now() - self.last_t).nanoseconds / 1e9
                if lost > LOST_TIMEOUT:
                    self.get_logger().warn(f"[LOST] {lost:.1f}s → SCAN_BACK")
                    self._go(STATE_SCAN_BACK, force=True); self.n_stable = 0

        if self.state == STATE_DONE:
            cmd.linear.x = cmd.angular.z = 0.0
        elif self.state in (STATE_REVERSE, STATE_SCAN_BACK):
            self._control(None, None, None, None, cmd)
        elif self.state == STATE_SEARCH:
            if self.tag.valid and self.n_stable >= STABLE_REQUIRED:
                self._go(STATE_APPROACH, force=True)
            else:
                self._control(None, None, None, None, cmd)
        elif self.tag.valid and self.n_stable >= STABLE_REQUIRED:
            x, z, yaw = self.tag.x, self.tag.z, self.tag.yaw
            bearing   = self.tag.bearing
            next_s    = self._next_nav_state(z)
            self._go(next_s, force=(next_s in (STATE_DONE, STATE_ALIGN)))  # ← force ALIGN ด้วย
            self._control(x, z, yaw, bearing, cmd)

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
            float(self._align_stable_frames),
            float(self._align_yaw_sign),
            float(self.tag.x) if self.tag.x is not None else -999.0,
            float(1.0 if self.spatial.active else 0.0),
            float(self.spatial.cx) if self.spatial.cx else -999.0,
        ]
        self.dbg_pub.publish(dbg)


# ════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = AprilTagServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()