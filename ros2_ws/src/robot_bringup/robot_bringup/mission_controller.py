#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32, String

# ════════════════════════════════════════════════════════════════
STOP_THRESH  = 0.003   # หยุดเมื่อเหลือ 3 mm
SLOW_1_DIST  = 0.08    # 8 cm  (reference เท่านั้น, PID จัดการเอง)
SLOW_1_FRAC  = 0.30
SLOW_2_DIST  = 0.03    # 3 cm
SLOW_2_FRAC  = 0.15

FINAL_MOVE_M = 1.0     # ระยะเดินหน้าหลัง FINISH (เมตร)

# ════════════════════════════════════════════════════════════════
MS_IDLE           = 0
MS_WAIT_0         = 1
MS_APPROACH       = 2
MS_WAIT_APPROACH  = 3
MS_REVERSE        = 4
MS_WAIT_REVERSE   = 5
MS_PLANT_1        = 6
MS_WAIT_1         = 7
MS_PLANT_2        = 8
MS_WAIT_2         = 9
MS_GAP            = 10
MS_WAIT_3         = 11
MS_INTERVAL_1     = 12
MS_WAIT_4A        = 13
MS_INTERVAL_2     = 14
MS_WAIT_4B        = 15
MS_FINISH         = 16   # หยุด + publish FINISH แล้วรอ wait_sec
MS_FINAL_MOVE     = 17   # เดินหน้า 1.0 m
MS_DONE           = 18   # จบสมบูรณ์

MS_NAME = {
    MS_IDLE:          "IDLE",        MS_WAIT_0:        "WAIT_0",
    MS_APPROACH:      "APPROACH",    MS_WAIT_APPROACH: "WAIT_APPROACH",
    MS_REVERSE:       "REVERSE",     MS_WAIT_REVERSE:  "WAIT_REVERSE",
    MS_PLANT_1:       "PLANT_1",     MS_WAIT_1:        "WAIT_1",
    MS_PLANT_2:       "PLANT_2",     MS_WAIT_2:        "WAIT_2",
    MS_GAP:           "GAP",         MS_WAIT_3:        "WAIT_3",
    MS_INTERVAL_1:    "INTERVAL_1",  MS_WAIT_4A:       "WAIT_4A",
    MS_INTERVAL_2:    "INTERVAL_2",  MS_WAIT_4B:       "WAIT_4B",
    MS_FINISH:        "FINISH",
    MS_FINAL_MOVE:    "FINAL_MOVE",
    MS_DONE:          "DONE",
}

APRILTAG_STATE_DONE = 4


# ════════════════════════════════════════════════════════════════
class PIDController:
    """
    Velocity PID
      input  : error (m)   = remaining distance
      output : speed (m/s) clamped [out_min, out_max]

    Anti-windup: integral clamped ±integral_limit
    Velocity ramp: output ถูก ramp ขึ้นจาก 0 ใน ramp_time วินาที
                   ป้องกัน wheel slip ตอนออกตัว
    """

    def __init__(self,
                 kp: float, ki: float, kd: float,
                 out_min: float, out_max: float,
                 integral_limit: float = 2.0,
                 ramp_time: float = 0.20):
        self.kp             = kp
        self.ki             = ki
        self.kd             = kd
        self.out_min        = out_min
        self.out_max        = out_max
        self.integral_limit = integral_limit
        self.ramp_time      = ramp_time        # วินาที ramp up

        self._integral  = 0.0
        self._prev_err  = 0.0
        self._prev_t    = None
        self._start_t   = None   # เวลา reset() (สำหรับ ramp)
        self._prev_out  = 0.0

    def reset(self):
        """เรียกทุก segment ใหม่"""
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_t   = None
        self._start_t  = None
        self._prev_out = 0.0

    def compute(self, error: float, now: float) -> float:
        # init tick
        if self._prev_t is None:
            self._prev_t  = now
            self._start_t = now
            self._prev_err = error
            return self.out_min

        dt = now - self._prev_t
        if dt <= 1e-6:
            return self._prev_out

        # ── PID ───────────────────────────────────────────
        p_term = self.kp * error

        self._integral += error * dt
        self._integral  = max(-self.integral_limit,
                              min(self.integral_limit, self._integral))
        i_term = self.ki * self._integral

        d_term = self.kd * (error - self._prev_err) / dt

        self._prev_err = error
        self._prev_t   = now

        raw = p_term + i_term + d_term
        out = max(self.out_min, min(self.out_max, raw))

        # ── velocity ramp ─────────────────────────────────
        elapsed  = now - self._start_t
        ramp_fac = min(1.0, elapsed / max(self.ramp_time, 1e-6))
        out      = out * ramp_fac
        out      = max(self.out_min * ramp_fac, out)  # ไม่ต่ำกว่า 0 ช่วง ramp

        self._prev_out = out
        return out

    def update_gains(self, kp: float, ki: float, kd: float):
        self.kp, self.ki, self.kd = kp, ki, kd


# ════════════════════════════════════════════════════════════════
class SlipDetector:
    """
    เปรียบเทียบ cmd_vel กับ actual velocity จาก encoder
    เงื่อนไข (robust): actual < cmd_speed * slip_ratio
    ป้องกัน false-positive ใน slow zone และช่วง ramp

    ตัวอย่าง:
      cmd=0.06, actual=0.02  → 0.02 < 0.06*0.3=0.018?  ไม่ใช่ → ปกติ
      cmd=0.06, actual=0.01  → 0.01 < 0.018?            ใช่     → นับ
    """

    def __init__(self,
                 vel_thresh: float = 0.05,
                 slip_ratio: float = 0.30,
                 slip_max: int     = 10):
        self.vel_thresh = vel_thresh   # cmd ขั้นต่ำที่เริ่ม check
        self.slip_ratio = slip_ratio   # actual ต้องมากกว่า cmd * ratio
        self.slip_max   = slip_max
        self._count     = 0

    def reset(self):
        self._count = 0

    def update(self, cmd_speed: float, actual_speed: float) -> bool:
        """คืน True เมื่อตรวจพบ slip สะสมเกิน slip_max"""
        if cmd_speed > self.vel_thresh \
                and actual_speed < cmd_speed * self.slip_ratio:
            self._count += 1
        else:
            self._count = max(0, self._count - 1)   # decay เมื่อปกติ
        return self._count >= self.slip_max

    @property
    def count(self):
        return self._count


# ════════════════════════════════════════════════════════════════
class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')
        self._declare_params()
        self._load_params()
        self._build_ros()
        self._init_state()
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"MissionController (PID+AntiOvershoot+SlipDetect) ready"
            f" | cmd={self._cmd_topic}"
            f" | camera_target_z={self._camera_target_z}m"
            f" | reverse={self._reverse_after_m}m"
            f" | wait={self._wait_sec}s"
            f" | ticks_unit={self._ticks_unit}"
            f" | stop={STOP_THRESH*1000:.0f}mm"
            f" | PID kp={self._kp} ki={self._ki} kd={self._kd}"
            f" | decel={self._decel_mps2}m/s²"
            f" | ramp={self._ramp_time}s"
            f" | slip_ratio={self._slip_ratio}"
            f" | spike_filter={self._encoder_spike_cm}cm"
            f" | final_move={FINAL_MOVE_M}m")

    # ── Params ────────────────────────────────────────────────────
    def _declare_params(self):
        d = self.declare_parameter
        d('wheel_diameter',       0.127)
        d('ticks_per_rev',        5940)
        d('forward_vel',          0.20)
        d('reverse_vel',          0.20)
        d('camera_target_z',      0.50)
        d('reverse_after_m',      0.40)
        d('wait_sec',             3.0)
        d('cmd_topic',            '/cmd_vel_mission')
        d('ticks_unit',           'cm')
        d('encoder_noise_cm',     0.5)
        # PID
        d('kp',                   1.5)
        d('ki',                   0.0)
        d('kd',                   0.0)
        d('pid_out_min',          0.05)
        # Anti-overshoot
        d('decel_mps2',           0.30)   # m/s²  ความสามารถเบรกของหุ่น
        # Velocity ramp
        d('ramp_time',            0.20)   # s
        # Slip detection
        d('slip_vel_thresh',      0.05)   # m/s  cmd ขั้นต่ำที่เริ่ม check
        d('slip_ratio',           0.30)   # actual ต้องมากกว่า cmd*ratio
        d('slip_max',             10)     # ticks ก่อน warn
        # Encoder spike filter
        d('encoder_spike_cm',     10.0)   # cm  delta เกินนี้ = spike ละเว้น

    def _load_params(self):
        g = self.get_parameter
        self._wheel_circ        = math.pi * g('wheel_diameter').value
        self._ticks_per_rev     = int(g('ticks_per_rev').value)
        self._forward_vel       = g('forward_vel').value
        self._reverse_vel       = g('reverse_vel').value
        self._camera_target_z   = g('camera_target_z').value
        self._reverse_after_m   = g('reverse_after_m').value
        self._wait_sec          = g('wait_sec').value
        self._cmd_topic         = g('cmd_topic').value
        self._ticks_unit        = g('ticks_unit').value
        self._encoder_noise_cm  = g('encoder_noise_cm').value
        self._kp                = g('kp').value
        self._ki                = g('ki').value
        self._kd                = g('kd').value
        self._pid_out_min       = g('pid_out_min').value
        self._decel_mps2        = g('decel_mps2').value
        self._ramp_time         = g('ramp_time').value
        self._slip_vel_thresh   = g('slip_vel_thresh').value
        self._slip_ratio        = g('slip_ratio').value
        self._slip_max          = int(g('slip_max').value)
        self._encoder_spike_cm  = g('encoder_spike_cm').value

    # ── ROS ───────────────────────────────────────────────────────
    def _build_ros(self):
        self.create_subscription(Float32MultiArray, '/vision_debug',
                                 self._cb_vision_debug, 10)
        self.create_subscription(Float32MultiArray, '/apriltag/pose',
                                 self._cb_pose, 10)
        self.create_subscription(Int32,  '/apriltag/planting_distance',
                                 self._cb_plant,    10)
        self.create_subscription(Int32,  '/apriltag/gap_type',
                                 self._cb_gap,      10)
        self.create_subscription(Int32,  '/apriltag/cabbage_interval',
                                 self._cb_interval, 10)
        self.create_subscription(Float32MultiArray, '/wheel_ticks',
                                 self._cb_ticks, 10)
        self.create_subscription(String, '/plant_feedback',
                                 self._cb_plant_feedback,   10)
        self.create_subscription(String, '/capture_feedback',
                                 self._cb_capture_feedback, 10)

        self._cmd_pub    = self.create_publisher(Twist,  self._cmd_topic,  1)
        self._msg_pub    = self.create_publisher(String, '/msg',           10)
        self._ms_dbg_pub = self.create_publisher(Int32,  '/mission_debug', 5)

    # ── State init ────────────────────────────────────────────────
    def _init_state(self):
        self._ms                  = MS_IDLE
        self._wait_start          = None
        self._apriltag_state      = -1
        self._pose_z              = None
        self._plant_dist_cm       = None
        self._gap_cm              = None
        self._interval_cm         = None

        # odometry
        self._ticks_last_raw      = None
        self._ticks_last_t        = None
        self._odometry_m          = 0.0
        self._move_dist_m         = 0.0
        self._target_m            = 0.0
        self._approach_m          = 0.0

        # actual velocity estimate (m/s) from encoder diff
        self._actual_vel          = 0.0
        self._last_cmd_speed      = 0.0   # magnitude ที่สั่งครั้งล่าสุด

        self._done_triggered      = False
        self._plant_feedback_ok   = False
        self._capture_feedback_ok = False

        # PID (สอง instance แยก forward / reverse state)
        self._pid_fwd = PIDController(
            kp=self._kp, ki=self._ki, kd=self._kd,
            out_min=self._pid_out_min,
            out_max=self._forward_vel,
            ramp_time=self._ramp_time)
        self._pid_rev = PIDController(
            kp=self._kp, ki=self._ki, kd=self._kd,
            out_min=self._pid_out_min,
            out_max=self._reverse_vel,
            ramp_time=self._ramp_time)

        # slip detector
        self._slip = SlipDetector(
            vel_thresh=self._slip_vel_thresh,
            slip_ratio=self._slip_ratio,
            slip_max=self._slip_max)

    # ── Callbacks ─────────────────────────────────────────────────
    def _cb_vision_debug(self, msg):
        if len(msg.data) > 0:
            self._apriltag_state = int(msg.data[0])

    def _cb_pose(self, msg):
        if len(msg.data) >= 3:
            self._pose_z = float(msg.data[2])

    def _cb_plant(self, msg):
        self._plant_dist_cm = int(msg.data)
        self.get_logger().info(
            f"[PARAM] planting_distance={self._plant_dist_cm}cm")

    def _cb_gap(self, msg):
        gap_code  = int(msg.data)
        gap_table = {1: 5, 2: 10, 3: 15, 4: 20, 5: 25}
        self._gap_cm = gap_table.get(gap_code, 10)
        if gap_code not in gap_table:
            self.get_logger().warn(
                f"[GAP] unknown code {gap_code}, default 10cm")
        self.get_logger().info(
            f"[PARAM] gap_type={gap_code} → {self._gap_cm}cm")

    def _cb_interval(self, msg):
        self._interval_cm = int(msg.data)
        self.get_logger().info(
            f"[PARAM] cabbage_interval={self._interval_cm}cm")

    def _cb_ticks(self, msg):
        if len(msg.data) == 0:
            return
        now      = self._now()
        n_wheels = min(4, len(msg.data))
        avg_raw  = sum(abs(float(v)) for v in msg.data[:n_wheels]) / n_wheels

        if self._ticks_unit == 'cm':
            if self._ticks_last_raw is None:
                self._ticks_last_raw = avg_raw
                self._ticks_last_t   = now
                return
            delta_cm   = avg_raw - self._ticks_last_raw
            dt         = now - self._ticks_last_t if self._ticks_last_t else 0.05
            abs_dcm    = abs(delta_cm)

            self._ticks_last_raw = avg_raw
            self._ticks_last_t   = now

            if abs_dcm < 0.05:           # < 0.5mm = noise
                self._actual_vel = 0.0
                return

            # ★ encoder spike filter — delta ผิดปกติ (หลุด/กระโดด)
            if abs_dcm > self._encoder_spike_cm:
                self.get_logger().warn(
                    f"[ENCODER] spike ignored: delta={abs_dcm:.1f}cm")
                return

            # ★ actual velocity low-pass (α=0.7) ลด noise
            raw_vel = (abs_dcm / 100.0) / dt if dt > 1e-6 else 0.0
            self._actual_vel = 0.7 * self._actual_vel + 0.3 * raw_vel

            self._move_dist_m += abs_dcm / 100.0
            if delta_cm > 0:
                self._odometry_m += delta_cm / 100.0

        elif self._ticks_unit == 'cm_inc':
            dt = (now - self._ticks_last_t) if self._ticks_last_t else 0.05
            self._ticks_last_t = now
            dm = avg_raw / 100.0
            self._move_dist_m += dm
            self._odometry_m  += dm
            if dt > 1e-6:
                raw_vel = dm / dt
                self._actual_vel = 0.7 * self._actual_vel + 0.3 * raw_vel

        elif self._ticks_unit == 'ticks':
            dt = (now - self._ticks_last_t) if self._ticks_last_t else 0.05
            self._ticks_last_t = now
            m  = avg_raw * (self._wheel_circ / self._ticks_per_rev)
            self._move_dist_m += m
            self._odometry_m  += m
            if dt > 1e-6:
                raw_vel = m / dt
                self._actual_vel = 0.7 * self._actual_vel + 0.3 * raw_vel

    def _cb_plant_feedback(self, msg):
        if msg.data.strip().upper() == "SUCCESS":
            self.get_logger().info(
                f"[FEEDBACK] plant SUCCESS ({MS_NAME[self._ms]})")
            self._plant_feedback_ok = True

    def _cb_capture_feedback(self, msg):
        if msg.data.strip().upper() == "SUCCESS":
            self.get_logger().info(
                f"[FEEDBACK] capture SUCCESS ({MS_NAME[self._ms]})")
            self._capture_feedback_ok = True

    # ── Core helpers ──────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _remaining_m(self):
        return max(0.0, self._target_m - self._move_dist_m)

    def _predict_stop_dist(self, speed: float) -> float:
        """
        Anti-overshoot: ระยะที่หุ่นยังเคลื่อนที่ได้หลังออกคำสั่ง stop
        ใช้ v² / (2·a)  โดย a = decel_mps2 (ค่าเบรกจริงของหุ่น)
        """
        if self._decel_mps2 <= 0:
            return 0.0
        return (speed ** 2) / (2.0 * self._decel_mps2)

    def _start_move(self, dist_m: float):
        self._move_dist_m = 0.0
        self._target_m    = dist_m
        self._actual_vel  = 0.0
        self._pid_fwd.reset()
        self._pid_rev.reset()
        self._slip.reset()
        self.get_logger().info(
            f"[MOVE] target={dist_m*100:.1f}cm"
            f"  odom_total={self._odometry_m:.4f}m")

    def _stop(self):
        self._cmd_pub.publish(Twist())
        self._last_cmd_speed = 0.0

    def _calc_speed(self, remaining, full_speed):
        """Open-loop fallback (reference เท่านั้น, ไม่ได้ใช้)"""
        if remaining <= STOP_THRESH:
            return 0.0
        elif remaining < SLOW_2_DIST:
            return full_speed * SLOW_2_FRAC
        elif remaining < SLOW_1_DIST:
            return full_speed * SLOW_1_FRAC
        return full_speed

    def _forward(self):
        """เดินหน้าด้วย PID + deadzone guard + physical speed cap + anti-overshoot"""
        remaining = self._remaining_m()

        # ★ deadzone guard — หยุดทันทีก่อน PID จะคำนวณ
        #   ป้องกัน oscillation ใกล้ stop (PID ยังสั่ง out_min แม้ error เล็กมาก)
        if remaining <= STOP_THRESH:
            self._stop()
            return

        spd = self._pid_fwd.compute(remaining, self._now())

        # ★ physical speed cap — clamp ไม่ให้เร็วเกินที่เบรกได้ทัน
        #   spd_max = sqrt(2 * decel * remaining)
        if self._decel_mps2 > 0:
            spd_cap = math.sqrt(2.0 * self._decel_mps2 * remaining)
            spd = min(spd, spd_cap)

        # anti-overshoot: ถ้าระยะเหลือ ≤ predict stop dist → หยุด
        stop_dist = self._predict_stop_dist(spd)
        if remaining <= stop_dist + STOP_THRESH:
            self._stop()
            self.get_logger().debug(
                f"[ANTI-OVERSHOOT fwd] remaining={remaining*100:.1f}cm"
                f"  stop_dist={stop_dist*100:.1f}cm → STOP")
            return

        cmd = Twist()
        cmd.linear.x     = spd
        self._last_cmd_speed = spd
        self._cmd_pub.publish(cmd)

        # slip detection
        if self._slip.update(spd, self._actual_vel):
            self.get_logger().warn(
                f"[SLIP] forward  cmd={spd:.3f}m/s"
                f"  actual={self._actual_vel:.3f}m/s"
                f"  count={self._slip.count}")

    def _reverse_cmd(self):
        """ถอยหลังด้วย PID + deadzone guard + physical speed cap + anti-overshoot"""
        remaining = self._remaining_m()

        # ★ deadzone guard
        if remaining <= STOP_THRESH:
            self._stop()
            return

        spd = self._pid_rev.compute(remaining, self._now())

        # ★ physical speed cap
        if self._decel_mps2 > 0:
            spd_cap = math.sqrt(2.0 * self._decel_mps2 * remaining)
            spd = min(spd, spd_cap)

        stop_dist = self._predict_stop_dist(spd)
        if remaining <= stop_dist + STOP_THRESH:
            self._stop()
            self.get_logger().debug(
                f"[ANTI-OVERSHOOT rev] remaining={remaining*100:.1f}cm"
                f"  stop_dist={stop_dist*100:.1f}cm → STOP")
            return

        cmd = Twist()
        cmd.linear.x     = -spd   # ทิศลบ = ถอย
        self._last_cmd_speed = spd
        self._cmd_pub.publish(cmd)

        if self._slip.update(spd, self._actual_vel):
            self.get_logger().warn(
                f"[SLIP] reverse  cmd={spd:.3f}m/s"
                f"  actual={self._actual_vel:.3f}m/s"
                f"  count={self._slip.count}")

    def _pub_msg(self, text: str):
        m = String()
        m.data = text
        self._msg_pub.publish(m)
        self.get_logger().info(f"[PUB] /msg → '{text}'")

    def _go(self, new_state: int):
        self.get_logger().info(
            f"[MS] {MS_NAME[self._ms]} → {MS_NAME[new_state]}"
            f"  (odom_total={self._odometry_m:.4f}m)")
        self._ms = new_state
        self._ms_dbg_pub.publish(Int32(data=new_state))

    def _start_wait(self):
        self._stop()
        self._wait_start = self._now()

    def _wait_done(self):
        return (self._now() - self._wait_start) >= self._wait_sec

    def _params_ready(self):
        return (self._plant_dist_cm is not None and
                self._gap_cm        is not None and
                self._interval_cm   is not None)

    def _reached(self):
        return self._remaining_m() <= STOP_THRESH

    def _log_stop(self, label: str):
        err = self._move_dist_m - self._target_m
        self.get_logger().info(
            f"[STOP] {label}"
            f"  target={self._target_m*100:.1f}cm"
            f"  actual={self._move_dist_m*100:.1f}cm"
            f"  err={err*100:+.1f}cm"
            f"  slip_count={self._slip.count}")

    # ── Main tick ─────────────────────────────────────────────────
    def _tick(self):

        # ── IDLE ─────────────────────────────────────────────────
        if self._ms == MS_IDLE:
            now = self._now()
            if not hasattr(self, '_last_idle_log') \
                    or (now - self._last_idle_log) > 2.0:
                self._last_idle_log = now
                self.get_logger().info(
                    f"[IDLE CHECK] apriltag={self._apriltag_state}"
                    f" pose_z={self._pose_z}"
                    f" plant={self._plant_dist_cm}"
                    f" gap={self._gap_cm}"
                    f" interval={self._interval_cm}")
            if (self._apriltag_state == APRILTAG_STATE_DONE
                    and not self._done_triggered
                    and self._params_ready()
                    and self._pose_z is not None):
                self._done_triggered = True
                self.get_logger().info(
                    f"[TRIGGER] z={self._pose_z:.3f}m"
                    f"  plant={self._plant_dist_cm}cm"
                    f"  gap={self._gap_cm}cm"
                    f"  interval={self._interval_cm}cm")
                self._go(MS_WAIT_0)
                self._start_wait()
            return

        # ── WAIT_0 ───────────────────────────────────────────────
        elif self._ms == MS_WAIT_0:
            if self._wait_done():
                if self._pose_z is None:
                    self.get_logger().warn(
                        "[WAIT_0] pose_z still None, waiting...")
                    self._wait_start = self._now()
                    return
                approach_m = max(0.0, self._camera_target_z - self._pose_z)
                if approach_m < 0.005:
                    self._approach_m = 0.0
                    self._go(MS_REVERSE)
                    self._start_move(self._reverse_after_m)
                else:
                    self._approach_m = approach_m
                    self._go(MS_APPROACH)
                    self._start_move(approach_m)

        # ── APPROACH ─────────────────────────────────────────────
        elif self._ms == MS_APPROACH:
            if self._reached():
                self._log_stop("APPROACH")
                self._go(MS_WAIT_APPROACH)
                self._start_wait()
                self._pub_msg("UP")
            else:
                self._forward()

        # ── WAIT_APPROACH ────────────────────────────────────────
        elif self._ms == MS_WAIT_APPROACH:
            if self._wait_done():
                reverse_m = max(0.0,
                                self._reverse_after_m - self._approach_m)
                self._go(MS_REVERSE)
                self._start_move(reverse_m)

        # ── REVERSE ──────────────────────────────────────────────
        elif self._ms == MS_REVERSE:
            if self._reached():
                self._log_stop("REVERSE")
                self._go(MS_WAIT_REVERSE)
                self._start_wait()
            else:
                self._reverse_cmd()

        # ── WAIT_REVERSE ─────────────────────────────────────────
        elif self._ms == MS_WAIT_REVERSE:
            if self._wait_done():
                self._go(MS_PLANT_1)
                self._start_move(self._plant_dist_cm / 100.0)

        # ── PLANT_1 ──────────────────────────────────────────────
        elif self._ms == MS_PLANT_1:
            if self._reached():
                self._log_stop("PLANT_1")
                self._stop()
                self._plant_feedback_ok = False
                self._go(MS_WAIT_1)
                self._start_wait()
                if self._pose_z is not None and self._pose_z > 16.0:
                    self._pub_msg("DONE:planting1A")
                else:
                    self._pub_msg("DONE:planting1B")
            else:
                self._forward()

        # ── WAIT_1 ───────────────────────────────────────────────
        elif self._ms == MS_WAIT_1:
            if self._wait_done() and self._plant_feedback_ok:
                self._plant_feedback_ok = False
                self._go(MS_PLANT_2)
                self._start_move(self._plant_dist_cm / 100.0)
            elif self._wait_done():
                self.get_logger().warn(
                    "[WAIT_1] waiting for /plant_feedback...",
                    throttle_duration_sec=2.0)

        # ── PLANT_2 ──────────────────────────────────────────────
        elif self._ms == MS_PLANT_2:
            if self._reached():
                self._log_stop("PLANT_2")
                self._stop()
                self._plant_feedback_ok = False
                self._go(MS_WAIT_2)
                self._start_wait()
                self._pub_msg("DONE:planting2")
            else:
                self._forward()

        # ── WAIT_2 ───────────────────────────────────────────────
        elif self._ms == MS_WAIT_2:
            if self._wait_done() and self._plant_feedback_ok:
                self._plant_feedback_ok = False
                self._go(MS_GAP)
                self._start_move(self._gap_cm / 100.0)
            elif self._wait_done():
                self.get_logger().warn(
                    "[WAIT_2] waiting for /plant_feedback...",
                    throttle_duration_sec=2.0)

        # ── GAP ──────────────────────────────────────────────────
        elif self._ms == MS_GAP:
            if self._reached():
                self._log_stop("GAP")
                self._stop()
                self._capture_feedback_ok = False
                self._go(MS_WAIT_3)
                self._start_wait()
                self._pub_msg("DONE:cabbage")
            else:
                self._forward()

        # ── WAIT_3 ───────────────────────────────────────────────
        elif self._ms == MS_WAIT_3:
            if self._wait_done() and self._capture_feedback_ok:
                self._capture_feedback_ok = False
                self._go(MS_INTERVAL_1)
                self._start_move(self._interval_cm / 100.0)
            elif self._wait_done():
                self.get_logger().warn(
                    "[WAIT_3] waiting for /capture_feedback...",
                    throttle_duration_sec=2.0)

        # ── INTERVAL_1 ───────────────────────────────────────────
        elif self._ms == MS_INTERVAL_1:
            if self._reached():
                self._log_stop("INTERVAL_1")
                self._stop()
                self._capture_feedback_ok = False
                self._go(MS_WAIT_4A)
                self._start_wait()
                self._pub_msg("DONE:cabbage")
            else:
                self._forward()

        # ── WAIT_4A ──────────────────────────────────────────────
        elif self._ms == MS_WAIT_4A:
            if self._wait_done() and self._capture_feedback_ok:
                self._capture_feedback_ok = False
                self._go(MS_INTERVAL_2)
                self._start_move(self._interval_cm / 100.0)
            elif self._wait_done():
                self.get_logger().warn(
                    "[WAIT_4A] waiting for /capture_feedback...",
                    throttle_duration_sec=2.0)

        # ── INTERVAL_2 ───────────────────────────────────────────
        elif self._ms == MS_INTERVAL_2:
            if self._reached():
                self._log_stop("INTERVAL_2")
                self._stop()
                self._capture_feedback_ok = False
                self._go(MS_WAIT_4B)
                self._start_wait()
                self._pub_msg("DONE:cabbage")
            else:
                self._forward()

        # ── WAIT_4B ──────────────────────────────────────────────
        elif self._ms == MS_WAIT_4B:
            if self._wait_done() and self._capture_feedback_ok:
                self._capture_feedback_ok = False
                self._go(MS_FINISH)
                self._start_wait()   # หยุดรอ wait_sec ก่อนเดินต่อ
            elif self._wait_done():
                self.get_logger().warn(
                    "[WAIT_4B] waiting for /capture_feedback...",
                    throttle_duration_sec=2.0)

        # ── FINISH ───────────────────────────────────────────────
        # หยุดนิ่ง + publish FINISH + รอ wait_sec แล้วจึงออกตัว
        elif self._ms == MS_FINISH:
            if self._wait_done():
                self.get_logger().info(
                    f"[MISSION] FINISH stop complete"
                    f"  odom_total={self._odometry_m:.3f}m"
                    f"  → FINAL_MOVE {FINAL_MOVE_M}m")
                self._go(MS_FINAL_MOVE)
                self._start_move(FINAL_MOVE_M)
                self._pub_msg("FINISH")   # publish หลังหยุดนิ่งแล้ว

        # ── FINAL_MOVE ───────────────────────────────────────────
        # เดินหน้า 1.0 m ด้วย PID เหมือน segment ปกติ
        elif self._ms == MS_FINAL_MOVE:
            if self._reached():
                self._log_stop("FINAL_MOVE")
                self._stop()
                self.get_logger().info(
                    f"[MISSION] DONE  odom_total={self._odometry_m:.3f}m")
                self._go(MS_DONE)
                self.timer.cancel()
            else:
                self._forward()

        # ── DONE ─────────────────────────────────────────────────
        elif self._ms == MS_DONE:
            self._stop()   # safety stop (ไม่ควรถึงจุดนี้หลัง cancel)


# ════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()