#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


STATE_WAIT       = 0
STATE_APPROACH   = 7 
STATE_RESET      = 1
STATE_WAIT_TICKS = 2
STATE_MOVE       = 3
STATE_STOP       = 4
STATE_DELAY      = 5
STATE_DONE       = 6

TICKS_ZERO_THRESHOLD = 2.0   # cm
TICKS_WAIT_TIMEOUT   = 3.0   # s
SPEED_FULL           = 0.3   # m/s
SPEED_SLOW           = 0.08  # m/s
DECEL_ZONE           = 10.0  # cm
RESET_RETRY_DELAY    = 1.0   # s
APPROACH_DISTANCE    = 72.5  # cm — เดินหน้าก่อนใช้ค่าจาก topic


class PlantingController(Node):

    def __init__(self):
        super().__init__('planting_controller')

        self.cb_group = ReentrantCallbackGroup()

        self.state            = STATE_WAIT
        self.target_distance  = None
        self.new_command      = False
        self.repeat_count     = 0
        self.max_repeats      = 2
        self.wheel_ticks      = [0.0, 0.0, 0.0, 0.0]
        self.start_distance   = 0.0
        self.delay_start_time = None
        self.delay_duration   = 5.0
        self.wait_ticks_start = None

        # approach
        self.approach_start_distance = 0.0

        self._lock          = threading.Lock()
        self._reset_pending = False
        self._reset_done    = False
        self._reset_success = False
        self._waiting_retry = False
        self._retry_start   = None

        # ── Service client ────────────────────────────────────────────────
        self.reset_client = self.create_client(
            Empty, '/wheel_ticks/reset',
            callback_group=self.cb_group)

        self.create_subscription(
            Float32MultiArray, '/wheel_ticks',
            self.wheel_callback, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            Int32, '/apriltag/planting_distance',
            self.planting_callback, 10,
            callback_group=self.cb_group)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_pid', 10)

        self.timer = self.create_timer(
            0.05, self.update,
            callback_group=self.cb_group)

        self.get_logger().info("Planting Controller Started")

    # =====================================================
    # Callbacks
    # =====================================================
    def wheel_callback(self, msg):
        self.wheel_ticks = list(msg.data)

    def planting_callback(self, msg):
        if self.state in (STATE_WAIT, STATE_DONE):
            self.target_distance = float(msg.data)
            self.new_command = True

    def get_average_distance(self):
        if len(self.wheel_ticks) >= 4:
            return sum(self.wheel_ticks) / 4.0
        return 0.0

    # =====================================================
    # Reset — ใช้ thread แยก + spin_until_future_complete
    # =====================================================
    def _reset_thread_fn(self):
        # รอ service พร้อม
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Service /wheel_ticks/reset not available after 5s")
            with self._lock:
                self._reset_success = False
                self._reset_done    = True
                self._reset_pending = False
            return

        future = self.reset_client.call_async(Empty.Request())

        # Block ใน thread นี้จนกว่า future จะ done
        # โดยให้ executor (MultiThreadedExecutor) จัดการ spin ปกติ
        timeout_sec = 10.0
        import time
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().warn("Reset call timed out")
                with self._lock:
                    self._reset_success = False
                    self._reset_done    = True
                    self._reset_pending = False
                return
            time.sleep(0.05)

        try:
            result  = future.result()
            success = result is not None
        except Exception as e:
            self.get_logger().error(f"Reset future exception: {e}")
            success = False

        if success:
            self.get_logger().info(
                "Encoder reset success — waiting for ticks to reach 0...")
        else:
            self.get_logger().error("Reset failed")

        with self._lock:
            self._reset_success = success
            self._reset_done    = True
            self._reset_pending = False

    def _spawn_reset(self):
        with self._lock:
            self._reset_pending = True
            self._reset_done    = False
            self._reset_success = False
        self._waiting_retry = False
        threading.Thread(target=self._reset_thread_fn, daemon=True).start()

    # =====================================================
    # State Machine
    # =====================================================
    def update(self):

        cmd = Twist()
        cmd.linear.x = 0.0

        # -------------------------
        # WAIT
        # -------------------------
        if self.state == STATE_WAIT:
            if self.new_command and self.target_distance is not None:
                self.get_logger().info(
                    f"New planting distance: {self.target_distance} cm — "
                    f"approaching {APPROACH_DISTANCE} cm first")
                self.repeat_count            = 0
                self.new_command             = False
                self._waiting_retry          = False
                self.approach_start_distance = self.get_average_distance()
                self.state                   = STATE_APPROACH

        # -------------------------
        # APPROACH — เดินหน้าก่อน
        # -------------------------
        elif self.state == STATE_APPROACH:
            current  = self.get_average_distance()
            traveled = current - self.approach_start_distance
            remain   = APPROACH_DISTANCE - traveled

            if remain > DECEL_ZONE:
                cmd.linear.x = SPEED_FULL
            elif remain > 0:
                ratio        = remain / DECEL_ZONE
                cmd.linear.x = max(
                    SPEED_SLOW + (SPEED_FULL - SPEED_SLOW) * ratio,
                    SPEED_SLOW)
            else:
                cmd.linear.x = 0.0
                self.get_logger().info(
                    f"Approach done (traveled={traveled:.1f} cm) — resetting encoder")
                self.state = STATE_RESET

        # -------------------------
        # RESET
        # -------------------------
        elif self.state == STATE_RESET:

            with self._lock:
                pending = self._reset_pending
                done    = self._reset_done
                success = self._reset_success

            if self._waiting_retry:
                now     = self.get_clock().now()
                elapsed = (now - self._retry_start).nanoseconds * 1e-9
                if elapsed >= RESET_RETRY_DELAY:
                    self.get_logger().info("Retrying encoder reset...")
                    self._spawn_reset()

            elif not pending and not done:
                self.get_logger().info("Resetting encoder...")
                self._spawn_reset()

            elif done:
                with self._lock:
                    self._reset_done    = False
                    self._reset_pending = False

                if success:
                    self._waiting_retry   = False
                    self.wait_ticks_start = self.get_clock().now()
                    self.state            = STATE_WAIT_TICKS
                else:
                    self._waiting_retry = True
                    self._retry_start   = self.get_clock().now()

        # -------------------------
        # WAIT_TICKS
        # -------------------------
        elif self.state == STATE_WAIT_TICKS:
            avg     = self.get_average_distance()
            now     = self.get_clock().now()
            elapsed = (now - self.wait_ticks_start).nanoseconds * 1e-9

            if abs(avg) < TICKS_ZERO_THRESHOLD:
                self.start_distance = avg
                self.get_logger().info(
                    f"Ticks zeroed ({avg:.2f} cm) — starting move")
                self.state = STATE_MOVE
            elif elapsed > TICKS_WAIT_TIMEOUT:
                self.start_distance = avg
                self.get_logger().warn(
                    f"Ticks wait timeout — baseline = {avg:.2f} cm")
                self.state = STATE_MOVE

        # -------------------------
        # MOVE — deceleration zone
        # -------------------------
        elif self.state == STATE_MOVE:
            current  = self.get_average_distance()
            traveled = current - self.start_distance
            remain   = self.target_distance - traveled

            if remain > DECEL_ZONE:
                cmd.linear.x = SPEED_FULL
            elif remain > 0:
                ratio        = remain / DECEL_ZONE
                cmd.linear.x = max(
                    SPEED_SLOW + (SPEED_FULL - SPEED_SLOW) * ratio,
                    SPEED_SLOW)
            else:
                cmd.linear.x = 0.0
                self.get_logger().info(
                    f"Reached {self.target_distance} cm "
                    f"(traveled={traveled:.1f} cm)")
                self.state = STATE_STOP

        # -------------------------
        # STOP
        # -------------------------
        elif self.state == STATE_STOP:
            cmd.linear.x          = 0.0
            self.delay_start_time = self.get_clock().now()
            self.state            = STATE_DELAY
            self.get_logger().info("Stopping 5 seconds...")

        # -------------------------
        # DELAY
        # -------------------------
        elif self.state == STATE_DELAY:
            now     = self.get_clock().now()
            elapsed = (now - self.delay_start_time).nanoseconds * 1e-9

            if elapsed >= self.delay_duration:
                self.repeat_count += 1
                if self.repeat_count < self.max_repeats:
                    self.get_logger().info("Continue to next move")
                    self._waiting_retry = False
                    self.state          = STATE_RESET
                else:
                    self.get_logger().info("Planting cycle complete")
                    self.state = STATE_DONE

        # -------------------------
        # DONE
        # -------------------------
        elif self.state == STATE_DONE:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)


# =====================================================
# MAIN
# =====================================================
def main(args=None):
    rclpy.init(args=args)

    node     = PlantingController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()