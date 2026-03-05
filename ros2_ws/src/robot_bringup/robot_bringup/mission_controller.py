#!/usr/bin/env python3
"""
Mission Controller Node
============================================================
รับ trigger จาก apriltag_servo (STATE_DONE) แล้วดำเนิน mission sequence
ต่อด้วย encoder-based movement

Topics subscribed:
  /vision_debug               (Float32MultiArray)  — [state, x, z, yaw, v, w, n_stable, stuck, bearing]
  /apriltag/pose              (Float32MultiArray)  — [x, 0, z, bearing_deg]
  /apriltag/planting_distance (Int32)              — AB (cm)
  /apriltag/gap_type          (Int32)              — C  (cm)
  /apriltag/cabbage_interval  (Int32)              — DE (cm)
  /wheel_ticks                (Int32)              — total accumulated encoder ticks

Topics published:
  /cmd_vel_mission            (Twist)              — velocity command
  /msg                        (String)             — mission events: DONE:1 … FINISH

Parameters:
  wheel_diameter   (float, default 0.195)  m
  ticks_per_rev    (int,   default 1440)   encoder counts per revolution
  forward_vel      (float, default 0.10)   m/s ขณะเดินหน้า
  reverse_vel      (float, default 0.10)   m/s ขณะถอยหลัง  (ค่าบวก ระบบจะใส่ลบให้)
  camera_target_z  (float, default 0.50)   m  ระยะที่ต้องการให้กล้องอยู่ห่าง tag
  reverse_after_m  (float, default 0.40)   m  ระยะถอยหลังหลัง final approach
  wait_sec         (float, default 2.0)    s  หน่วงเวลาระหว่าง step

Sequence หลัง DONE:
  1. หยุด wait_sec
  2. เดินหน้า (camera_target_z − z_pose)  ← encoder
  3. ถอยหลัง reverse_after_m             ← encoder
  4. หยุด wait_sec
  5. เดินหน้า planting_distance cm        ← encoder  → pub DONE:1
  6. หยุด wait_sec
  7. เดินหน้า planting_distance cm        ← encoder  → pub DONE:2
  8. หยุด wait_sec
  9. เดินหน้า gap_type cm                 ← encoder  → pub DONE:3
 10. หยุด wait_sec
 11. เดินหน้า cabbage_interval cm         ← encoder  → pub DONE:4
 12. หยุด wait_sec
 13. เดินหน้า cabbage_interval cm         ← encoder  → pub DONE:4
 14. หยุด wait_sec
 15. pub FINISH
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32, String

# ════════════════════════════════════════════════════════════════
# State machine
# ════════════════════════════════════════════════════════════════
MS_IDLE          = 0   # รอ DONE จาก apriltag_servo
MS_WAIT_0        = 1   # หยุดรอหลัง DONE
MS_APPROACH      = 2   # เดินหน้าจนกล้องห่าง tag = camera_target_z
MS_REVERSE       = 3   # ถอยหลัง reverse_after_m
MS_WAIT_REVERSE  = 4   # หยุดรอหลังถอย
MS_PLANT_1       = 5   # เดินหน้า planting_distance
MS_WAIT_1        = 6   # หยุด → pub DONE:1
MS_PLANT_2       = 7   # เดินหน้า planting_distance
MS_WAIT_2        = 8   # หยุด → pub DONE:2
MS_GAP           = 9   # เดินหน้า gap_type
MS_WAIT_3        = 10  # หยุด → pub DONE:3
MS_INTERVAL_1    = 11  # เดินหน้า cabbage_interval
MS_WAIT_4A       = 12  # หยุด → pub DONE:4
MS_INTERVAL_2    = 13  # เดินหน้า cabbage_interval
MS_WAIT_4B       = 14  # หยุด → pub DONE:4
MS_FINISH        = 15  # pub FINISH แล้วจบ

MS_NAME = {
    MS_IDLE:         "IDLE",
    MS_WAIT_0:       "WAIT_0",
    MS_APPROACH:     "APPROACH",
    MS_REVERSE:      "REVERSE",
    MS_WAIT_REVERSE: "WAIT_REVERSE",
    MS_PLANT_1:      "PLANT_1",
    MS_WAIT_1:       "WAIT_1",
    MS_PLANT_2:      "PLANT_2",
    MS_WAIT_2:       "WAIT_2",
    MS_GAP:          "GAP",
    MS_WAIT_3:       "WAIT_3",
    MS_INTERVAL_1:   "INTERVAL_1",
    MS_WAIT_4A:      "WAIT_4A",
    MS_INTERVAL_2:   "INTERVAL_2",
    MS_WAIT_4B:      "WAIT_4B",
    MS_FINISH:       "FINISH",
}

# STATE_DONE ใน apriltag_servo = 4
APRILTAG_STATE_DONE = 4


# ════════════════════════════════════════════════════════════════
# Helper: ticks ↔ metres
# ════════════════════════════════════════════════════════════════
def ticks_to_m(ticks: int, wheel_circ: float, ticks_per_rev: int) -> float:
    return ticks * (wheel_circ / ticks_per_rev)


# ════════════════════════════════════════════════════════════════
# Node
# ════════════════════════════════════════════════════════════════
class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')
        self._declare_params()
        self._load_params()
        self._build_ros()
        self._init_state()
        self.timer = self.create_timer(0.05, self._tick)   # 20 Hz
        self.get_logger().info(
            f"MissionController ready"
            f" | cmd={self._cmd_topic}"
            f" | camera_target_z={self._camera_target_z}m"
            f" | reverse={self._reverse_after_m}m"
            f" | wait={self._wait_sec}s")

    # ── Parameters ──────────────────────────────────────────────
    def _declare_params(self):
        import math
        d = self.declare_parameter
        d('wheel_diameter',  0.195)
        d('ticks_per_rev',   1440)
        d('forward_vel',     0.10)
        d('reverse_vel',     0.10)
        d('camera_target_z', 0.50)
        d('reverse_after_m', 0.40)
        d('wait_sec',        2.0)
        d('cmd_topic',       '/cmd_vel_pid')  # เปลี่ยนเป็น /cmd_vel หรือ /cmd_vel_pid ถ้าจำเป็น

    def _load_params(self):
        import math
        g = self.get_parameter
        self._wheel_circ       = math.pi * g('wheel_diameter').value
        self._ticks_per_rev    = int(g('ticks_per_rev').value)
        self._forward_vel      = g('forward_vel').value
        self._reverse_vel      = g('reverse_vel').value
        self._camera_target_z  = g('camera_target_z').value
        self._reverse_after_m  = g('reverse_after_m').value
        self._wait_sec         = g('wait_sec').value
        self._cmd_topic        = g('cmd_topic').value

    # ── ROS interfaces ───────────────────────────────────────────
    def _build_ros(self):
        # Subscribers
        self.create_subscription(
            Float32MultiArray, '/vision_debug',
            self._cb_vision_debug, 10)
        self.create_subscription(
            Float32MultiArray, '/apriltag/pose',
            self._cb_pose, 10)
        self.create_subscription(
            Int32, '/apriltag/planting_distance',
            self._cb_plant, 10)
        self.create_subscription(
            Int32, '/apriltag/gap_type',
            self._cb_gap, 10)
        self.create_subscription(
            Int32, '/apriltag/cabbage_interval',
            self._cb_interval, 10)
        self.create_subscription(
            Float32MultiArray, '/wheel_ticks',
            self._cb_ticks, 10)

        # Publishers
        self._cmd_pub = self.create_publisher(Twist,  self._cmd_topic, 1)
        self._msg_pub = self.create_publisher(String, '/msg',             10)

    # ── State init ───────────────────────────────────────────────
    def _init_state(self):
        self._ms           = MS_IDLE
        self._wait_start   = None   # time.monotonic() ณ เริ่ม wait

        # ข้อมูลจาก apriltag_servo
        self._apriltag_state  = -1
        self._pose_z          = None   # z จาก /apriltag/pose (m)
        self._plant_dist_cm   = None   # planting_distance (cm)
        self._gap_cm          = None   # gap_type (cm)
        self._interval_cm     = None   # cabbage_interval (cm)

        # encoder tracking
        self._ticks_now       = 0.0    # ค่า ticks เฉลี่ย 4 ล้อ ล่าสุด
        self._ticks_ref       = 0.0    # ค่า ticks ณ เริ่ม movement
        self._target_m        = 0.0    # ระยะที่ต้องเดิน (m)
        self._approach_m      = 0.0    # ระยะที่เดินเพิ่มใน APPROACH (เซฟไว้คำนวณ reverse)

        self._done_triggered  = False  # กันกด trigger ซ้ำ

    # ── Callbacks ────────────────────────────────────────────────
    def _cb_vision_debug(self, msg: Float32MultiArray):
        if len(msg.data) > 0:
            self._apriltag_state = int(msg.data[0])

    def _cb_pose(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._pose_z = float(msg.data[2])

    def _cb_plant(self, msg: Int32):
        self._plant_dist_cm = int(msg.data)
        self.get_logger().info(f"[PARAM] planting_distance={self._plant_dist_cm}cm")

    def _cb_gap(self, msg: Int32):
        self._gap_cm = int(msg.data)
        self.get_logger().info(f"[PARAM] gap_type={self._gap_cm}cm")

    def _cb_interval(self, msg: Int32):
        self._interval_cm = int(msg.data)
        self.get_logger().info(f"[PARAM] cabbage_interval={self._interval_cm}cm")

    def _cb_ticks(self, msg: Float32MultiArray):
        # /wheel_ticks = [FL, FR, RL, RR]  หน่วย cm
        # เฉลี่ย 4 ล้อ แล้วแปลง → เมตร
        if len(msg.data) >= 4:
            self._ticks_now = float(sum(msg.data[:4]) / 4.0) / 100.0
        elif len(msg.data) > 0:
            self._ticks_now = float(msg.data[0]) / 100.0

    # ── Helpers ──────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _travelled_m(self) -> float:
        """ระยะที่เดินไปจาก ref ถึงปัจจุบัน (เมตร)
        _ticks_now เป็นเมตรแล้ว (แปลงจาก cm ใน _cb_ticks)
        """
        return abs(self._ticks_now - self._ticks_ref)

    def _start_move(self, target_m: float):
        """เริ่ม movement ใหม่ — บันทึก reference position (m)"""
        self._ticks_ref = self._ticks_now
        self._target_m  = target_m
        self.get_logger().info(
            f"[MOVE] target={target_m:.3f}m  ref={self._ticks_ref:.4f}m")

    def _stop(self):
        cmd = Twist()
        self._cmd_pub.publish(cmd)

    def _forward(self):
        cmd = Twist()
        cmd.linear.x = self._forward_vel
        self._cmd_pub.publish(cmd)

    def _reverse(self):
        cmd = Twist()
        cmd.linear.x = -self._reverse_vel
        self._cmd_pub.publish(cmd)

    def _pub_msg(self, text: str):
        msg = String()
        msg.data = text
        self._msg_pub.publish(msg)
        self.get_logger().info(f"[PUB] /msg → '{text}'")

    def _go(self, new_state: int):
        self.get_logger().info(
            f"[MS] {MS_NAME[self._ms]} → {MS_NAME[new_state]}")
        self._ms = new_state

    def _start_wait(self):
        self._stop()
        self._wait_start = self._now()

    def _wait_done(self) -> bool:
        return (self._now() - self._wait_start) >= self._wait_sec

    def _params_ready(self) -> bool:
        return (self._plant_dist_cm is not None and
                self._gap_cm        is not None and
                self._interval_cm   is not None)

    # ── Main tick (20 Hz) ────────────────────────────────────────
    def _tick(self):
        # ── IDLE: รอ STATE_DONE จาก apriltag_servo ──────────────
        if self._ms == MS_IDLE:
            if (self._apriltag_state == APRILTAG_STATE_DONE
                    and not self._done_triggered
                    and self._params_ready()
                    and self._pose_z is not None):
                self._done_triggered = True
                self.get_logger().info(
                    f"[TRIGGER] DONE detected"
                    f"  pose_z={self._pose_z:.3f}m"
                    f"  plant={self._plant_dist_cm}cm"
                    f"  gap={self._gap_cm}cm"
                    f"  interval={self._interval_cm}cm")
                self._go(MS_WAIT_0)
                self._start_wait()
            return

        # ── WAIT_0: หยุด 2 วิหลัง DONE ─────────────────────────
        elif self._ms == MS_WAIT_0:
            if self._wait_done():
                # คำนวณระยะ final approach
                approach_m = max(0.0, self._camera_target_z - self._pose_z)
                if approach_m < 0.005:
                    # ใกล้พอแล้ว ข้าม approach
                    self._approach_m = 0.0
                    self.get_logger().info(
                        f"[APPROACH] skip (approach_m={approach_m:.3f}m < 5mm)")
                    reverse_m = max(0.0, self._reverse_after_m - self._approach_m)
                    self._go(MS_REVERSE)
                    self._start_move(reverse_m)
                else:
                    self._approach_m = approach_m
                    self.get_logger().info(
                        f"[APPROACH] target={self._camera_target_z}m"
                        f"  pose_z={self._pose_z:.3f}m"
                        f"  move={approach_m:.3f}m")
                    self._go(MS_APPROACH)
                    self._start_move(approach_m)

        # ── APPROACH: เดินหน้า (camera_target_z - pose_z) ───────
        elif self._ms == MS_APPROACH:
            travelled = self._travelled_m()
            if travelled >= self._target_m:
                self.get_logger().info(
                    f"[APPROACH] done  {travelled:.3f}m / {self._target_m:.3f}m")
                reverse_m = max(0.0, self._reverse_after_m - self._approach_m)
                self.get_logger().info(
                    f"[REVERSE] plan={self._reverse_after_m}m"
                    f" - approach={self._approach_m:.3f}m"
                    f" = {reverse_m:.3f}m")
                self._go(MS_REVERSE)
                self._start_move(reverse_m)
            else:
                self._forward()
                self.get_logger().info(
                    f"[APPROACH] {travelled:.3f}/{self._target_m:.3f}m"
                    f"  pos={self._ticks_now:.4f}m ref={self._ticks_ref:.4f}m",
                    throttle_duration_sec=0.5)

        # ── REVERSE: ถอยหลัง reverse_after_m ───────────────────
        elif self._ms == MS_REVERSE:
            if self._travelled_m() >= self._target_m:
                self.get_logger().info(
                    f"[REVERSE] done  {self._travelled_m():.3f}m"
                    f" / {self._target_m:.3f}m")
                self._go(MS_WAIT_REVERSE)
                self._start_wait()
            else:
                self._reverse()

        # ── WAIT_REVERSE: หยุด 2 วิหลังถอย ─────────────────────
        elif self._ms == MS_WAIT_REVERSE:
            if self._wait_done():
                dist_m = self._plant_dist_cm / 100.0
                self.get_logger().info(
                    f"[PLANT_1] move {self._plant_dist_cm}cm")
                self._go(MS_PLANT_1)
                self._start_move(dist_m)

        # ── PLANT_1: เดิน planting_distance ─────────────────────
        elif self._ms == MS_PLANT_1:
            if self._travelled_m() >= self._target_m:
                self._go(MS_WAIT_1)
                self._start_wait()
                self._pub_msg("DONE:1")
            else:
                self._forward()

        # ── WAIT_1: หยุด 2 วิ ────────────────────────────────────
        elif self._ms == MS_WAIT_1:
            if self._wait_done():
                dist_m = self._plant_dist_cm / 100.0
                self.get_logger().info(
                    f"[PLANT_2] move {self._plant_dist_cm}cm")
                self._go(MS_PLANT_2)
                self._start_move(dist_m)

        # ── PLANT_2: เดิน planting_distance อีกครั้ง ─────────────
        elif self._ms == MS_PLANT_2:
            if self._travelled_m() >= self._target_m:
                self._go(MS_WAIT_2)
                self._start_wait()
                self._pub_msg("DONE:2")
            else:
                self._forward()

        # ── WAIT_2: หยุด 2 วิ ────────────────────────────────────
        elif self._ms == MS_WAIT_2:
            if self._wait_done():
                dist_m = self._gap_cm / 100.0
                self.get_logger().info(
                    f"[GAP] move {self._gap_cm}cm")
                self._go(MS_GAP)
                self._start_move(dist_m)

        # ── GAP: เดิน gap_type ───────────────────────────────────
        elif self._ms == MS_GAP:
            if self._travelled_m() >= self._target_m:
                self._go(MS_WAIT_3)
                self._start_wait()
                self._pub_msg("DONE:3")
            else:
                self._forward()

        # ── WAIT_3: หยุด 2 วิ ────────────────────────────────────
        elif self._ms == MS_WAIT_3:
            if self._wait_done():
                dist_m = self._interval_cm / 100.0
                self.get_logger().info(
                    f"[INTERVAL_1] move {self._interval_cm}cm")
                self._go(MS_INTERVAL_1)
                self._start_move(dist_m)

        # ── INTERVAL_1: เดิน cabbage_interval ───────────────────
        elif self._ms == MS_INTERVAL_1:
            if self._travelled_m() >= self._target_m:
                self._go(MS_WAIT_4A)
                self._start_wait()
                self._pub_msg("DONE:4")
            else:
                self._forward()

        # ── WAIT_4A: หยุด 2 วิ ───────────────────────────────────
        elif self._ms == MS_WAIT_4A:
            if self._wait_done():
                dist_m = self._interval_cm / 100.0
                self.get_logger().info(
                    f"[INTERVAL_2] move {self._interval_cm}cm")
                self._go(MS_INTERVAL_2)
                self._start_move(dist_m)

        # ── INTERVAL_2: เดิน cabbage_interval อีกครั้ง ───────────
        elif self._ms == MS_INTERVAL_2:
            if self._travelled_m() >= self._target_m:
                self._go(MS_WAIT_4B)
                self._start_wait()
                self._pub_msg("DONE:4")
            else:
                self._forward()

        # ── WAIT_4B: หยุด 2 วิ ───────────────────────────────────
        elif self._ms == MS_WAIT_4B:
            if self._wait_done():
                self._go(MS_FINISH)

        # ── FINISH ───────────────────────────────────────────────
        elif self._ms == MS_FINISH:
            self._stop()
            self._pub_msg("FINISH")
            self.get_logger().info("[MISSION] complete — FINISH")
            # หยุด timer ไม่รัน tick อีก
            self.timer.cancel()


# ════════════════════════════════════════════════════════════════
# Entry point
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