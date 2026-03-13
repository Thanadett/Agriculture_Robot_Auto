#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32, String

# ════════════════════════════════════════════════════════════════
STOP_THRESH  = 0.003   # หยุดเมื่อเหลือ 3 mm
SLOW_1_DIST  = 0.08    # เริ่ม slow zone 1 ที่ 8 cm  (reference เท่านั้น)
SLOW_1_FRAC  = 0.30    # 30% of full speed
SLOW_2_DIST  = 0.03    # เริ่ม slow zone 2 ที่ 3 cm
SLOW_2_FRAC  = 0.15    # 15% of full speed

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
MS_FINISH         = 16

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
}

APRILTAG_STATE_DONE = 4


# ════════════════════════════════════════════════════════════════
class PIDController:

    def __init__(self,
                 kp: float, ki: float, kd: float,
                 out_min: float, out_max: float,
                 integral_limit: float = 2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral_limit = integral_limit

        self._integral  = 0.0
        self._prev_err  = 0.0
        self._prev_t    = None   # timestamp (sec)

    def reset(self):
        """เรียกทุก segment ใหม่ เพื่อล้าง integral/derivative state"""
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_t   = None

    def compute(self, error: float, now: float) -> float:
        """
        คำนวณ output speed จาก error (remaining distance)
        คืนค่า 0.0 เมื่อยังไม่มี dt (tick แรก)
        """
        if self._prev_t is None:
            self._prev_t   = now
            self._prev_err = error
            return 0.0

        dt = now - self._prev_t
        if dt <= 1e-6:
            return 0.0

        # Proportional
        p_term = self.kp * error

        # Integral  + anti-windup clamp
        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))
        i_term = self.ki * self._integral

        # Derivative
        d_term = self.kd * (error - self._prev_err) / dt

        self._prev_err = error
        self._prev_t   = now

        raw = p_term + i_term + d_term
        return max(self.out_min, min(self.out_max, raw))

    def update_gains(self, kp: float, ki: float, kd: float):
        self.kp, self.ki, self.kd = kp, ki, kd


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
            f"MissionController (PID) ready"
            f" | cmd={self._cmd_topic}"
            f" | camera_target_z={self._camera_target_z}m"
            f" | reverse={self._reverse_after_m}m"
            f" | wait={self._wait_sec}s"
            f" | ticks_unit={self._ticks_unit}"
            f" | stop={STOP_THRESH*1000:.0f}mm"
            f" | PID kp={self._kp} ki={self._ki} kd={self._kd}"
            f" | out_min={self._pid_out_min:.3f}m/s")

    # ── Parameter declarations ────────────────────────────────────
    def _declare_params(self):
        d = self.declare_parameter
        d('wheel_diameter',    0.127)
        d('ticks_per_rev',     5940)
        d('forward_vel',       0.10)
        d('reverse_vel',       0.10)
        d('camera_target_z',   0.50)
        d('reverse_after_m',   0.40)
        d('wait_sec',          3.0)
        d('cmd_topic',         '/cmd_vel_mission')
        d('ticks_unit',        'cm')
        d('encoder_noise_cm',  0.5)
        # ── PID gains ──────────────────────────────────────
        d('kp',           0.8)
        d('ki',           0.0)
        d('kd',           0.0)
        d('pid_out_min',  0.05)   # ความเร็วต่ำสุด (m/s)

    def _load_params(self):
        g = self.get_parameter
        self._wheel_circ       = math.pi * g('wheel_diameter').value
        self._ticks_per_rev    = int(g('ticks_per_rev').value)
        self._forward_vel      = g('forward_vel').value
        self._reverse_vel      = g('reverse_vel').value
        self._camera_target_z  = g('camera_target_z').value
        self._reverse_after_m  = g('reverse_after_m').value
        self._wait_sec         = g('wait_sec').value
        self._cmd_topic        = g('cmd_topic').value
        self._ticks_unit       = g('ticks_unit').value
        self._encoder_noise_cm = g('encoder_noise_cm').value
        # PID
        self._kp          = g('kp').value
        self._ki          = g('ki').value
        self._kd          = g('kd').value
        self._pid_out_min = g('pid_out_min').value

    # ── ROS interfaces ────────────────────────────────────────────
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
        # ── odometry ──────────────────────────────────────
        self._ticks_last_raw      = None
        self._odometry_m          = 0.0   # สะสม forward เท่านั้น (log รวม)
        self._move_dist_m         = 0.0   # reset ทุก _start_move(), นับ |delta|
        self._target_m            = 0.0
        self._approach_m          = 0.0
        self._done_triggered      = False
        self._plant_feedback_ok   = False
        self._capture_feedback_ok = False
        # ── PID ───────────────────────────────────────────
        self._pid_fwd = PIDController(
            kp=self._kp, ki=self._ki, kd=self._kd,
            out_min=self._pid_out_min,
            out_max=self._forward_vel)
        self._pid_rev = PIDController(
            kp=self._kp, ki=self._ki, kd=self._kd,
            out_min=self._pid_out_min,
            out_max=self._reverse_vel)

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
        n_wheels = min(4, len(msg.data))
        avg_raw  = sum(abs(float(v)) for v in msg.data[:n_wheels]) / n_wheels

        if self._ticks_unit == 'cm':
            if self._ticks_last_raw is None:
                self._ticks_last_raw = avg_raw
                return
            delta_cm = avg_raw - self._ticks_last_raw
            self._ticks_last_raw = avg_raw
            abs_delta_cm = abs(delta_cm)

            if abs_delta_cm < 0.05:       # < 0.5mm = noise ละเว้น
                return

            # _move_dist_m นับ |delta| ทั้ง forward / reverse
            self._move_dist_m += abs_delta_cm / 100.0

            # _odometry_m นับเฉพาะ forward
            if delta_cm > 0:
                self._odometry_m += delta_cm / 100.0

        elif self._ticks_unit == 'cm_inc':
            self._move_dist_m += avg_raw / 100.0
            self._odometry_m  += avg_raw / 100.0

        elif self._ticks_unit == 'ticks':
            m = avg_raw * (self._wheel_circ / self._ticks_per_rev)
            self._move_dist_m += m
            self._odometry_m  += m

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

    # ── Helpers ───────────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _remaining_m(self):
        return max(0.0, self._target_m - self._move_dist_m)

    def _start_move(self, dist_m):
        """เริ่ม segment ใหม่ — reset encoder และ PID ทั้งคู่"""
        self._move_dist_m = 0.0
        self._target_m    = dist_m
        self._pid_fwd.reset()
        self._pid_rev.reset()
        self.get_logger().info(
            f"[MOVE] target={dist_m*100:.1f}cm"
            f"  odom_total={self._odometry_m:.4f}m")

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _calc_speed(self, remaining, full_speed):
        """
        Open-loop speed schedule (reference เท่านั้น ไม่ได้ใช้ใน PID mode)
        """
        if remaining <= STOP_THRESH:
            return 0.0
        elif remaining < SLOW_2_DIST:
            return full_speed * SLOW_2_FRAC
        elif remaining < SLOW_1_DIST:
            return full_speed * SLOW_1_FRAC
        else:
            return full_speed

    def _forward(self):
        """เดินหน้าด้วย PID — speed ขึ้นกับ remaining distance จริง"""
        err = self._remaining_m()
        spd = self._pid_fwd.compute(err, self._now())
        cmd = Twist()
        cmd.linear.x = spd
        self._cmd_pub.publish(cmd)

    def _reverse_cmd(self):
        """ถอยหลังด้วย PID — speed ขึ้นกับ remaining distance จริง"""
        err = self._remaining_m()
        spd = self._pid_rev.compute(err, self._now())
        cmd = Twist()
        cmd.linear.x = -spd      # ทิศลบ = ถอยหลัง
        self._cmd_pub.publish(cmd)

    def _pub_msg(self, text):
        m = String()
        m.data = text
        self._msg_pub.publish(m)
        self.get_logger().info(f"[PUB] /msg → '{text}'")

    def _go(self, new_state):
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

    def _log_stop(self, label):
        err = self._move_dist_m - self._target_m
        self.get_logger().info(
            f"[STOP] {label}"
            f"  target={self._target_m*100:.1f}cm"
            f"  actual={self._move_dist_m*100:.1f}cm"
            f"  err={err*100:+.1f}cm")

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
                    self._pub_msg("UP")
                else:
                    self._approach_m = approach_m
                    self._go(MS_APPROACH)
                    self._start_move(approach_m)
                    self._pub_msg("UP")

        # ── APPROACH ─────────────────────────────────────────────
        elif self._ms == MS_APPROACH:
            if self._reached():
                self._log_stop("APPROACH")
                self._go(MS_WAIT_APPROACH)
                self._start_wait()
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
                if self._pose_z is not None and self._pose_z > 18:
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
                    "[WAIT_1] waiting for /plant_feedback...")

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
                    "[WAIT_2] waiting for /plant_feedback...")

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
                    "[WAIT_3] waiting for /capture_feedback...")

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
                    "[WAIT_4A] waiting for /capture_feedback...")

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
            elif self._wait_done():
                self.get_logger().warn(
                    "[WAIT_4B] waiting for /capture_feedback...")

        # ── FINISH ───────────────────────────────────────────────
        elif self._ms == MS_FINISH:
            self._stop()
            self._pub_msg("FINISH")
            self.get_logger().info(
                f"[MISSION] FINISH  odom_total={self._odometry_m:.3f}m")
            self.timer.cancel()


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