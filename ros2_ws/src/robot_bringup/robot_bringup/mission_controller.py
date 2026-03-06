#!/usr/bin/env python3
"""
Mission Controller Node
============================================================
(Same as previous version — added /mission_debug Int32 publisher
 so debug_x11 can display current mission state.)

Topics subscribed:
  /vision_debug               (Float32MultiArray)
  /apriltag/pose              (Float32MultiArray)
  /apriltag/planting_distance (Int32)
  /apriltag/gap_type          (Int32)
  /apriltag/cabbage_interval  (Int32)
  /wheel_ticks                (Float32MultiArray)  [FL,FR,RL,RR] cm
  /plant_feedback             (String)  → "SUCCESS"
  /capture_feedback           (String)  → "SUCCESS"

Topics published:
  /cmd_vel_mission   (Twist)
  /msg               (String)   DONE:planting / DONE:cabbage / FINISH
  /mission_debug     (Int32)    current MS_* state (for debug_x11)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32, String

# ════════════════════════════════════════════════════════════════
# States
# ════════════════════════════════════════════════════════════════
MS_IDLE          = 0
MS_WAIT_0        = 1
MS_APPROACH      = 2
MS_REVERSE       = 3
MS_WAIT_REVERSE  = 4
MS_PLANT_1       = 5
MS_WAIT_1        = 6
MS_PLANT_2       = 7
MS_WAIT_2        = 8
MS_GAP           = 9
MS_WAIT_3        = 10
MS_INTERVAL_1    = 11
MS_WAIT_4A       = 12
MS_INTERVAL_2    = 13
MS_WAIT_4B       = 14
MS_FINISH        = 15

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

APRILTAG_STATE_DONE = 4


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')
        self._declare_params()
        self._load_params()
        self._build_ros()
        self._init_state()
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"MissionController ready"
            f" | cmd={self._cmd_topic}"
            f" | camera_target_z={self._camera_target_z}m"
            f" | reverse={self._reverse_after_m}m"
            f" | wait={self._wait_sec}s")

    def _declare_params(self):
        import math
        d = self.declare_parameter
        d('wheel_diameter',  0.127)
        d('ticks_per_rev',   1440)
        d('forward_vel',     0.10)
        d('reverse_vel',     0.10)
        d('camera_target_z', 0.50)
        d('reverse_after_m', 0.40)
        d('wait_sec',        3.0)
        d('cmd_topic',       '/cmd_vel_mission')

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

    def _build_ros(self):
        self.create_subscription(Float32MultiArray, '/vision_debug',
                                 self._cb_vision_debug, 10)
        self.create_subscription(Float32MultiArray, '/apriltag/pose',
                                 self._cb_pose, 10)
        self.create_subscription(Int32, '/apriltag/planting_distance',
                                 self._cb_plant, 10)
        self.create_subscription(Int32, '/apriltag/gap_type',
                                 self._cb_gap, 10)
        self.create_subscription(Int32, '/apriltag/cabbage_interval',
                                 self._cb_interval, 10)
        self.create_subscription(Float32MultiArray, '/wheel_ticks',
                                 self._cb_ticks, 10)
        self.create_subscription(String, '/plant_feedback',
                                 self._cb_plant_feedback, 10)
        self.create_subscription(String, '/capture_feedback',
                                 self._cb_capture_feedback, 10)

        self._cmd_pub     = self.create_publisher(Twist,  self._cmd_topic, 1)
        self._msg_pub     = self.create_publisher(String, '/msg',          10)
        self._ms_dbg_pub  = self.create_publisher(Int32,  '/mission_debug', 5)

    def _init_state(self):
        self._ms           = MS_IDLE
        self._wait_start   = None
        self._apriltag_state  = -1
        self._pose_z          = None
        self._plant_dist_cm   = None
        self._gap_cm          = None
        self._interval_cm     = None
        self._ticks_now       = 0.0
        self._ticks_ref       = 0.0
        self._target_m        = 0.0
        self._approach_m      = 0.0
        self._done_triggered  = False
        self._plant_feedback_ok   = False
        self._capture_feedback_ok = False

    # ── Callbacks ────────────────────────────────────────────────
    def _cb_vision_debug(self, msg):
        if len(msg.data) > 0:
            self._apriltag_state = int(msg.data[0])

    def _cb_pose(self, msg):
        if len(msg.data) >= 3:
            self._pose_z = float(msg.data[2])

    def _cb_plant(self, msg):
        self._plant_dist_cm = int(msg.data)
        self.get_logger().info(f"[PARAM] planting_distance={self._plant_dist_cm}cm")

    def _cb_gap(self, msg):
        self._gap_cm = int(msg.data)
        self.get_logger().info(f"[PARAM] gap_type={self._gap_cm}cm")

    def _cb_interval(self, msg):
        self._interval_cm = int(msg.data)
        self.get_logger().info(f"[PARAM] cabbage_interval={self._interval_cm}cm")

    def _cb_ticks(self, msg):
        if len(msg.data) >= 4:
            self._ticks_now = float(sum(msg.data[:4]) / 4.0) / 100.0
        elif len(msg.data) > 0:
            self._ticks_now = float(msg.data[0]) / 100.0

    def _cb_plant_feedback(self, msg):
        if msg.data.strip().upper() == "SUCCESS":
            self.get_logger().info(
                f"[FEEDBACK] /plant_feedback SUCCESS  ({MS_NAME[self._ms]})")
            self._plant_feedback_ok = True

    def _cb_capture_feedback(self, msg):
        if msg.data.strip().upper() == "SUCCESS":
            self.get_logger().info(
                f"[FEEDBACK] /capture_feedback SUCCESS  ({MS_NAME[self._ms]})")
            self._capture_feedback_ok = True

    # ── Helpers ──────────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _travelled_m(self):
        return abs(self._ticks_now - self._ticks_ref)

    def _start_move(self, target_m):
        self._ticks_ref = self._ticks_now
        self._target_m  = target_m
        self.get_logger().info(f"[MOVE] target={target_m:.3f}m")

    def _stop(self):    self._cmd_pub.publish(Twist())
    def _forward(self):
        cmd = Twist(); cmd.linear.x = self._forward_vel; self._cmd_pub.publish(cmd)
    def _reverse_cmd(self):
        cmd = Twist(); cmd.linear.x = -self._reverse_vel; self._cmd_pub.publish(cmd)

    def _pub_msg(self, text):
        msg = String(); msg.data = text
        self._msg_pub.publish(msg)
        self.get_logger().info(f"[PUB] /msg → '{text}'")

    def _go(self, new_state):
        self.get_logger().info(
            f"[MS] {MS_NAME[self._ms]} → {MS_NAME[new_state]}")
        self._ms = new_state
        self._ms_dbg_pub.publish(Int32(data=new_state))  # ← debug_x11 用

    def _start_wait(self):
        self._stop(); self._wait_start = self._now()

    def _wait_done(self):
        return (self._now() - self._wait_start) >= self._wait_sec

    def _params_ready(self):
        return (self._plant_dist_cm is not None and
                self._gap_cm        is not None and
                self._interval_cm   is not None)

    # ── Main tick ────────────────────────────────────────────────
    def _tick(self):

        if self._ms == MS_IDLE:
            if (self._apriltag_state == APRILTAG_STATE_DONE
                    and not self._done_triggered
                    and self._params_ready()
                    and self._pose_z is not None):
                self._done_triggered = True
                self.get_logger().info(
                    f"[TRIGGER] DONE  z={self._pose_z:.3f}m"
                    f"  plant={self._plant_dist_cm}cm"
                    f"  gap={self._gap_cm}cm  interval={self._interval_cm}cm")
                self._go(MS_WAIT_0); self._start_wait()
            return

        elif self._ms == MS_WAIT_0:
            if self._wait_done():
                approach_m = max(0.0, self._camera_target_z - self._pose_z)
                if approach_m < 0.005:
                    self._approach_m = 0.0
                    self._go(MS_REVERSE)
                    self._start_move(self._reverse_after_m)
                else:
                    self._approach_m = approach_m
                    self._go(MS_APPROACH)
                    self._start_move(approach_m)

        elif self._ms == MS_APPROACH:
            if self._travelled_m() >= self._target_m:
                reverse_m = max(0.0, self._reverse_after_m - self._approach_m)
                self._go(MS_REVERSE); self._start_move(reverse_m)
            else:
                self._forward()

        elif self._ms == MS_REVERSE:
            if self._travelled_m() >= self._target_m:
                self._go(MS_WAIT_REVERSE); self._start_wait()
            else:
                self._reverse_cmd()

        elif self._ms == MS_WAIT_REVERSE:
            if self._wait_done():
                self._go(MS_PLANT_1)
                self._start_move(self._plant_dist_cm / 100.0)

        elif self._ms == MS_PLANT_1:
            if self._travelled_m() >= self._target_m:
                self._plant_feedback_ok = False
                self._go(MS_WAIT_1); self._start_wait()
                self._pub_msg("DONE:planting")
            else:
                self._forward()

        elif self._ms == MS_WAIT_1:
            if self._wait_done() and self._plant_feedback_ok:
                self._plant_feedback_ok = False
                self._go(MS_PLANT_2)
                self._start_move(self._plant_dist_cm / 100.0)
            elif self._wait_done():
                self.get_logger().warn("[WAIT_1] waiting for /plant_feedback SUCCESS...")

        elif self._ms == MS_PLANT_2:
            if self._travelled_m() >= self._target_m:
                self._plant_feedback_ok = False
                self._go(MS_WAIT_2); self._start_wait()
                self._pub_msg("DONE:planting")
            else:
                self._forward()

        elif self._ms == MS_WAIT_2:
            if self._wait_done() and self._plant_feedback_ok:
                self._plant_feedback_ok = False
                self._go(MS_GAP)
                self._start_move(self._gap_cm / 100.0)
            elif self._wait_done():
                self.get_logger().warn("[WAIT_2] waiting for /plant_feedback SUCCESS...")

        elif self._ms == MS_GAP:
            if self._travelled_m() >= self._target_m:
                self._capture_feedback_ok = False
                self._go(MS_WAIT_3); self._start_wait()
                self._pub_msg("DONE:cabbage")
            else:
                self._forward()

        elif self._ms == MS_WAIT_3:
            if self._wait_done() and self._capture_feedback_ok:
                self._capture_feedback_ok = False
                self._go(MS_INTERVAL_1)
                self._start_move(self._interval_cm / 100.0)
            elif self._wait_done():
                self.get_logger().warn("[WAIT_3] waiting for /capture_feedback SUCCESS...")

        elif self._ms == MS_INTERVAL_1:
            if self._travelled_m() >= self._target_m:
                self._capture_feedback_ok = False
                self._go(MS_WAIT_4A); self._start_wait()
                self._pub_msg("DONE:cabbage")
            else:
                self._forward()

        elif self._ms == MS_WAIT_4A:
            if self._wait_done() and self._capture_feedback_ok:
                self._capture_feedback_ok = False
                self._go(MS_INTERVAL_2)
                self._start_move(self._interval_cm / 100.0)
            elif self._wait_done():
                self.get_logger().warn("[WAIT_4A] waiting for /capture_feedback SUCCESS...")

        elif self._ms == MS_INTERVAL_2:
            if self._travelled_m() >= self._target_m:
                self._capture_feedback_ok = False
                self._go(MS_WAIT_4B); self._start_wait()
                self._pub_msg("DONE:cabbage")
            else:
                self._forward()

        elif self._ms == MS_WAIT_4B:
            if self._wait_done() and self._capture_feedback_ok:
                self._capture_feedback_ok = False
                self._go(MS_FINISH)
            elif self._wait_done():
                self.get_logger().warn("[WAIT_4B] waiting for /capture_feedback SUCCESS...")

        elif self._ms == MS_FINISH:
            self._stop()
            self._pub_msg("FINISH")
            self.get_logger().info("[MISSION] FINISH")
            self.timer.cancel()


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