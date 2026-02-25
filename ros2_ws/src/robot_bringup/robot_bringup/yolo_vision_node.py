#!/home/prukubt/yolo_env/bin/python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from cv_bridge import CvBridge

import cv2
import math
import numpy as np
from ultralytics import YOLO
from pupil_apriltags import Detector


# ════════════════════════════════════════════════════════════════
# STATES
# ════════════════════════════════════════════════════════════════
STATE_SEARCH         = 0
STATE_CONFIRM        = 1
STATE_TRACK          = 2
STATE_LOST           = 3
STATE_FINAL_APPROACH = 4

STATE_NAME = {
    STATE_SEARCH:         "SEARCH",
    STATE_CONFIRM:        "CONFIRM",
    STATE_TRACK:          "TRACK",
    STATE_LOST:           "LOST",
    STATE_FINAL_APPROACH: "FINAL_APPROACH",
}

FA_APPROACH = "approach"
FA_DONE     = "done"

TARGET_CLASS_ID     = 0
YOLO_INTERVAL       = 3
LOCK_CONFIRM_FRAMES = 3
MAX_LOST_FRAMES     = 10
MIN_TRACK_FRAMES    = 15
EMA_ALPHA           = 0.5

# ── ขอบล่างของภาพที่ใช้เป็น setpoint ───────────────────────────
# top_y >= H - BOTTOM_MARGIN_PX → ถือว่า object ชนขอบล่างแล้ว
BOTTOM_MARGIN_PX = 20   # pixel buffer จากขอบจริง (ปรับได้)

TAG_INTERVAL      = 5
TAG_READ_REQUIRED = 8


# ════════════════════════════════════════════════════════════════
# EMA BBox Smoother  (ใช้ cx, top_y, area)
# ════════════════════════════════════════════════════════════════
class EMABoxSmoother:
    def __init__(self, alpha: float = 0.5):
        self.alpha = alpha
        self.cx = self.top_y = self.area = None
        self.valid = False

    def update(self, cx, top_y, area):
        if not self.valid:
            self.cx = cx; self.top_y = top_y; self.area = area
            self.valid = True
        else:
            a = self.alpha
            self.cx    = a * cx    + (1 - a) * self.cx
            self.top_y = a * top_y + (1 - a) * self.top_y
            self.area  = a * area  + (1 - a) * self.area

    def reset(self):
        self.cx = self.top_y = self.area = None
        self.valid = False

    @property
    def values(self):
        return self.cx, self.top_y, self.area


# ════════════════════════════════════════════════════════════════
# MAIN NODE
# ════════════════════════════════════════════════════════════════
class RobotFollower(Node):

    def __init__(self):
        super().__init__('robot_follower')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('camera_id',    0)
        self.declare_parameter('image_width',  640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('model_path',   '/home/prukubt/392_Agri/ros2_ws/best.pt')
        self.declare_parameter('conf',         0.5)
        self.declare_parameter('max_linear',   0.2)
        self.declare_parameter('max_angular',  0.4)
        self.declare_parameter('camera_vertical_fov_deg',         45.0)
        self.declare_parameter('center_deadband_px',               12)
        self.declare_parameter('angle_deadband_rad',               0.03)
        self.declare_parameter('angular_deadband_when_stopped_px', 30)
        self.declare_parameter('max_bbox_change_ratio',            1.8)
        self.declare_parameter('ema_alpha',                        EMA_ALPHA)
        self.declare_parameter('bottom_margin_px',    BOTTOM_MARGIN_PX)  # pixel buffer ขอบล่าง
        self.declare_parameter('min_stopped_frames',  3)

        # Final approach (AprilTag)
        self.declare_parameter('final_stop_distance_m', 0.20)  # หยุดเมื่อ z < 20 cm
        self.declare_parameter('final_forward_speed',   0.10)  # ความเร็ว final approach
        self.declare_parameter('final_tag_timeout_sec', 3.0)   # safety stop ถ้าไม่เห็น tag

        # Camera intrinsics
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
        self.declare_parameter('publish_image', True)

        # ── Load params ─────────────────────────────────────────────
        cam_id     = self.get_parameter('camera_id').value
        model_path = self.get_parameter('model_path').value
        self.conf  = self.get_parameter('conf').value

        self.max_linear  = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.W = self.get_parameter('image_width').value
        self.H = self.get_parameter('image_height').value

        self.vfov_rad = math.radians(
            self.get_parameter('camera_vertical_fov_deg').value)
        self.center_deadband          = self.get_parameter('center_deadband_px').value
        self.angle_deadband           = self.get_parameter('angle_deadband_rad').value
        self.angular_deadband_stopped = self.get_parameter('angular_deadband_when_stopped_px').value
        self.max_bbox_change          = self.get_parameter('max_bbox_change_ratio').value
        self.ema_alpha                = self.get_parameter('ema_alpha').value
        self.bottom_margin            = self.get_parameter('bottom_margin_px').value
        self.min_stopped_frames       = self.get_parameter('min_stopped_frames').value
        self.final_stop_dist          = self.get_parameter('final_stop_distance_m').value
        self.final_speed              = self.get_parameter('final_forward_speed').value
        self.final_tag_timeout        = self.get_parameter('final_tag_timeout_sec').value

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

        # ── setpoint pixel y (ขอบล่างของภาพ - margin) ───────────────
        self.setpoint_y = self.H - self.bottom_margin  # เส้นที่ top_y ต้องชน

        # ── YOLO ────────────────────────────────────────────────────
        self.model = YOLO(model_path)
        _dummy = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        self.model(_dummy, conf=self.conf, verbose=False)
        self.get_logger().info("YOLO warm-up done")

        # ── AprilTag ─────────────────────────────────────────────────
        self.at_detector = Detector(
            families="tagStandard52h13",
            nthreads=2,
            quad_decimate=4.5,
            refine_edges=1,
        )

        # ── ROS I/O ──────────────────────────────────────────────────
        self.cmd_pub      = self.create_publisher(Twist,             '/cmd_vel_pid',  1)
        self.debug_pub    = self.create_publisher(Float32MultiArray, '/vision_debug', 5)
        self.image_pub    = self.create_publisher(Image,             '/vision/image', 1)
        self.plant_pub    = self.create_publisher(Int32,             '/apriltag/planting_distance', 5)
        self.gap_pub      = self.create_publisher(Int32,             '/apriltag/gap_type',           5)
        self.interval_pub = self.create_publisher(Int32,             '/apriltag/cabbage_interval',   5)
        self.pose_pub     = self.create_publisher(Float32MultiArray, '/apriltag/pose',               1)
        self.wheel_sub    = self.create_subscription(
            Float32MultiArray, '/wheel_ticks', self.wheel_ticks_callback, 5)
        self.reset_client = self.create_client(Empty, '/wheel_ticks/reset')
        self.bridge = CvBridge()

        # ── Camera ───────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # ── State ────────────────────────────────────────────────────
        self.state               = STATE_SEARCH
        self.frame_count         = 0
        self.confirm_count       = 0
        self.smoother            = EMABoxSmoother(alpha=self.ema_alpha)
        self.lost_count          = 0
        self.track_frame_count   = 0
        self.stopped_frame_count = 0
        self.current_ref_y       = None
        self.wheel_ticks         = [0.0, 0.0, 0.0, 0.0]

        # AprilTag
        self.tag_read_count      = 0
        self.tag_published       = False
        self.last_tag_data       = None   # (ab, c, de, x, y, z, yaw)
        self.last_tag_time       = None
        self.final_approach_done = False
        self.fa_sub_state        = FA_APPROACH

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info(
            f"RobotFollower v5 started | "
            f"setpoint_y={self.setpoint_y}px | "
            f"final_stop={self.final_stop_dist}m"
        )

    # ════════════════════════════════════════════════════════════════
    # WHEEL TICKS
    # ════════════════════════════════════════════════════════════════
    def wheel_ticks_callback(self, msg):
        self.wheel_ticks = list(msg.data)

    def get_average_wheel_distance(self):
        return sum(self.wheel_ticks) / 4.0 if len(self.wheel_ticks) >= 4 else 0.0

    def reset_wheel_ticks(self):
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Reset service not available')
            return
        future = self.reset_client.call_async(Empty.Request())
        def cb(f):
            try:
                f.result()
            except Exception as e:
                self.get_logger().error(f'Reset failed: {e}')
        future.add_done_callback(cb)

    # ════════════════════════════════════════════════════════════════
    # HELPERS
    # ════════════════════════════════════════════════════════════════
    def reset_tracking_state(self):
        self.smoother.reset()
        self.lost_count = self.track_frame_count = self.stopped_frame_count = 0
        self.current_ref_y = None

    def _enter_final_approach(self):
        self.tag_read_count      = 0
        self.tag_published       = False
        self.last_tag_data       = None
        self.last_tag_time       = None
        self.final_approach_done = False
        self.fa_sub_state        = FA_APPROACH
        self.state               = STATE_FINAL_APPROACH
        self.stopped_frame_count = 0
        self.get_logger().info("[STATE] → FINAL_APPROACH")

    def detect_best(self, frame):
        results  = self.model(frame, conf=self.conf, verbose=False)
        best_box = None; best_area = -1
        for box in results[0].boxes:
            if int(box.cls[0]) != TARGET_CLASS_ID:
                continue
            x1, y1, x2, y2 = box.xyxy[0]
            area = float((x2 - x1) * (y2 - y1))
            if area > best_area:
                best_area = area
                best_box  = (float(x1), float(y1), float(x2), float(y2))
        return best_box

    # ════════════════════════════════════════════════════════════════
    # CONTROL  — PID เดินหน้า/หมุน ตาม top_y เทียบกับ setpoint_y
    #
    # setpoint_y = H - bottom_margin  (ใกล้ขอบล่าง)
    # ไกล  → top_y น้อย (สูงในภาพ) → error บวก → เดินหน้า
    # ใกล้  → top_y มาก (ต่ำในภาพ) → error ≈ 0   → หยุด
    # ════════════════════════════════════════════════════════════════
    def compute_cmd(self, cx, top_y, cmd):
        """
        คืนค่า True ถ้าหยุดแล้ว (top_y ≥ setpoint_y)
        """
        error_y = self.setpoint_y - top_y   # บวก = ยังต้องเดินหน้า

        is_stopped = error_y <= 0           # top_y ชนหรือผ่าน setpoint แล้ว

        # ── angular (PID หมุน ตาม cx) ───────────────────────────────
        pixel_error_x = cx - (self.W / 2.0)
        deadband = self.angular_deadband_stopped if is_stopped else self.center_deadband
        if abs(pixel_error_x) < deadband:
            pixel_error_x = 0.0
        heading       = -(pixel_error_x / (self.W / 2.0)) * (self.vfov_rad / 2.0)
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, heading))

        # ── linear (proportional เดินหน้า) ──────────────────────────
        if not is_stopped:
            # proportional gain เบาๆ เมื่อใกล้ setpoint
            ratio        = min(error_y / max(self.setpoint_y, 1.0), 1.0)
            speed        = self.max_linear * max(ratio, 0.4)   # อย่างน้อย 40% speed
            cmd.linear.x = speed
        else:
            cmd.linear.x = 0.0

        return is_stopped

    # ════════════════════════════════════════════════════════════════
    # APRILTAG
    # ════════════════════════════════════════════════════════════════
    def apriltag_read_and_pub(self, frame):
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
            self.get_logger().warn(f"AprilTag detect error: {e}")
            return
        if not detections:
            return

        best  = max(detections, key=lambda d: cv2.contourArea(d.corners.astype(int)))
        tx, ty, tz = best.pose_t.flatten()
        R_mat = best.pose_R
        x_m   = float(tx); y_m = float(ty); z_m = float(tz)
        yaw_d = math.degrees(math.atan2(float(R_mat[0, 2]), float(R_mat[2, 2])))
        tag_id = best.tag_id
        ab = tag_id // 1000
        c  = (tag_id // 100) % 10
        de = tag_id % 100

        self.tag_read_count += 1
        self.last_tag_data   = (ab, c, de, x_m, y_m, z_m, yaw_d)
        self.last_tag_time   = self.get_clock().now()

        # วาด overlay
        corners = best.corners.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1) % 4]),
                     (255, 0, 255), 2)
        cv2.putText(frame,
            f"TAG:{tag_id} z={z_m:.3f}m x={x_m:.3f}m "
            f"[{self.tag_read_count}/{TAG_READ_REQUIRED}]",
            (10, self.H - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

        if self.tag_read_count >= TAG_READ_REQUIRED and not self.tag_published:
            self.plant_pub.publish(Int32(data=ab))
            self.gap_pub.publish(Int32(data=c))
            self.interval_pub.publish(Int32(data=de))
            pose_msg = Float32MultiArray()
            pose_msg.data = [x_m, y_m, z_m, yaw_d]
            self.pose_pub.publish(pose_msg)
            self.tag_published = True
            self.get_logger().info(
                f"[AprilTag] AB={ab} C={c} DE={de} | z={z_m:.3f}m yaw={yaw_d:.1f}deg")

    # ════════════════════════════════════════════════════════════════
    # DRAWING
    # ════════════════════════════════════════════════════════════════
    def draw_bbox(self, frame, cx, bot_y, w, h):
        x = int(cx - w / 2); y = int(bot_y - h)   # y_top = bot_y - h
        cv2.rectangle(frame, (x, y), (x + int(w), y + int(h)), (0, 255, 0), 2)
        # จุดแดงที่ขอบล่าง
        cv2.circle(frame, (int(cx), int(bot_y)), 6, (0, 0, 255), -1)
        cv2.putText(frame,
            f"{STATE_NAME[self.state]} | bot_y={int(bot_y)} | lost:{self.lost_count}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    def draw_overlay(self, frame, cmd):
        # เส้นกลางภาพ (แนวตั้ง)
        cv2.line(frame, (self.W // 2, 0), (self.W // 2, self.H), (255, 255, 0), 1)

        # ── เส้น setpoint (ขอบล่าง) ─────────────────────────────────
        sp = int(self.setpoint_y)
        # เส้นทึบสีแดงสด หนา 2px
        cv2.line(frame, (0, sp), (self.W, sp), (0, 0, 255), 2)
        # ป้ายกำกับ
        cv2.putText(frame, f"STOP LINE y={sp}px",
            (5, sp - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # ── สถานะ TRACK ─────────────────────────────────────────────
        if self.state == STATE_TRACK and self.current_ref_y is not None:
            error_y = self.setpoint_y - self.current_ref_y
            if error_y <= 0:
                txt   = f"STOPPED ({self.stopped_frame_count}/{self.min_stopped_frames})"
                color = (0, 255, 0)
            else:
                txt   = f"MOVING err={error_y:.0f}px | track={self.track_frame_count}"
                color = (0, 165, 255)
            cv2.putText(frame, txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # ── สถานะ FINAL_APPROACH ────────────────────────────────────
        if self.state == STATE_FINAL_APPROACH and self.last_tag_data:
            z_m = self.last_tag_data[5]
            cv2.putText(frame, f"FA z={z_m:.3f}m / stop={self.final_stop_dist}m",
                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

    # ════════════════════════════════════════════════════════════════
    # FINAL APPROACH
    # ════════════════════════════════════════════════════════════════
    def handle_final_approach(self, frame, cmd):
        tag_z = self.last_tag_data[5] if self.last_tag_data else None

        if self.fa_sub_state == FA_APPROACH:
            # ตรวจ timeout (safety: ไม่เห็น tag นานเกิน)
            timed_out = False
            if self.last_tag_time is not None:
                elapsed = (self.get_clock().now() - self.last_tag_time).nanoseconds / 1e9
                timed_out = elapsed > self.final_tag_timeout

            if timed_out:
                cmd.linear.x = cmd.angular.z = 0.0
                cv2.putText(frame, "FINAL: TAG TIMEOUT - STOPPED",
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                self.get_logger().warn("[FINAL] Tag timeout → safety stop")

            elif tag_z is None:
                # ยังไม่เห็น tag → เดินหน้าช้าๆ
                cmd.linear.x  = self.final_speed
                cmd.angular.z = 0.0
                cv2.putText(frame, "FINAL: searching tag...",
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 150, 255), 2)

            elif tag_z > self.final_stop_dist:
                # ยังไกล → เดินหน้า (proportional)
                ratio        = min((tag_z - self.final_stop_dist) / 0.5, 1.0)
                speed        = self.final_speed * max(ratio, 0.5)
                cmd.linear.x  = speed
                cmd.angular.z = 0.0
                cv2.putText(frame,
                    f"FINAL: z={tag_z:.3f}m → {self.final_stop_dist:.2f}m spd={speed:.2f}",
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 100, 0), 2)

            else:
                # ถึงระยะแล้ว → หยุดรอ
                cmd.linear.x = cmd.angular.z = 0.0
                self.fa_sub_state        = FA_DONE
                self.final_approach_done = True
                self.get_logger().info(f"[FINAL] Done! z={tag_z:.3f}m ≤ {self.final_stop_dist}m")

        elif self.fa_sub_state == FA_DONE:
            cmd.linear.x = cmd.angular.z = 0.0
            cv2.putText(frame, f"FINAL: DONE (z={tag_z:.3f}m)" if tag_z else "FINAL: DONE",
                (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # ════════════════════════════════════════════════════════════════
    # MAIN LOOP
    # ════════════════════════════════════════════════════════════════
    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1
        cmd = Twist()

        # อ่าน AprilTag ทุก TAG_INTERVAL เฟรม (ทุก state)
        if self.frame_count % TAG_INTERVAL == 0:
            self.apriltag_read_and_pub(frame)

        # ════════════════════════════════════════════════════════════
        # FINAL_APPROACH
        # ════════════════════════════════════════════════════════════
        if self.state == STATE_FINAL_APPROACH:
            self.handle_final_approach(frame, cmd)

        # ════════════════════════════════════════════════════════════
        # TRACK
        # ════════════════════════════════════════════════════════════
        elif self.state == STATE_TRACK:
            self.track_frame_count += 1

            # YOLO re-detect ทุก YOLO_INTERVAL เฟรม
            if self.frame_count % YOLO_INTERVAL == 0:
                box = self.detect_best(frame)
                if box is not None:
                    x1, y1, x2, y2 = box
                    new_cx   = (x1 + x2) / 2.0
                    new_bot  = float(y2)           # ขอบล่าง
                    new_area = (x2 - x1) * (y2 - y1)
                    if self.smoother.valid:
                        ratio = new_area / max(self.smoother.area, 1.0)
                        if (1.0 / self.max_bbox_change) < ratio < self.max_bbox_change:
                            self.smoother.update(new_cx, new_bot, new_area)
                            self.lost_count = 0
                        # else: bbox กระโดด → ignore เฟรมนี้
                    else:
                        self.smoother.update(new_cx, new_bot, new_area)
                        self.lost_count = 0
                else:
                    self.lost_count += 1
                    # หยุดทันทีถ้ามองไม่เห็น > 2 เฟรม
                    if self.lost_count > 2:
                        cmd.linear.x = cmd.angular.z = 0.0

            if self.smoother.valid:
                s_cx, s_top_y, s_area = self.smoother.values
                s_h = math.sqrt(max(s_area, 1.0))   # ประมาณ h
                s_w = s_h
                self.current_ref_y = s_top_y

                # compute cmd และตรวจว่าหยุดแล้วหรือยัง
                is_stopped = self.compute_cmd(s_cx, s_top_y, cmd)
                self.draw_bbox(frame, s_cx, s_top_y, s_w, s_h)

                if is_stopped:
                    self.stopped_frame_count += 1
                else:
                    self.stopped_frame_count = 0

                # Trigger FINAL_APPROACH เมื่อ track ครบ MIN_TRACK_FRAMES
                # และ stopped ครบ min_stopped_frames
                if (self.track_frame_count >= MIN_TRACK_FRAMES and
                        self.stopped_frame_count >= self.min_stopped_frames):
                    self._enter_final_approach()

            # Lost เกิน MAX_LOST_FRAMES → LOST
            if self.lost_count > MAX_LOST_FRAMES:
                self.get_logger().warn("Lost target → LOST")
                cmd.linear.x = cmd.angular.z = 0.0
                self.state = STATE_LOST

        # ════════════════════════════════════════════════════════════
        # LOST → SEARCH
        # ════════════════════════════════════════════════════════════
        if self.state == STATE_LOST:
            self.reset_tracking_state()
            self.confirm_count = 0
            self.state = STATE_SEARCH

        # ════════════════════════════════════════════════════════════
        # SEARCH / CONFIRM
        # ════════════════════════════════════════════════════════════
        if (self.state in [STATE_SEARCH, STATE_CONFIRM] and
                self.frame_count % YOLO_INTERVAL == 0):
            box = self.detect_best(frame)
            if box is not None:
                self.confirm_count += 1
                if self.confirm_count >= LOCK_CONFIRM_FRAMES:
                    x1, y1, x2, y2 = box
                    self.smoother.reset()
                    self.smoother.update((x1 + x2) / 2.0, float(y2),  # ขอบล่าง
                                         (x2 - x1) * (y2 - y1))
                    self.tag_read_count      = 0
                    self.tag_published       = False
                    self.state               = STATE_TRACK
                    self.confirm_count       = 0
                    self.track_frame_count   = 0
                    self.stopped_frame_count = 0
                    self.lost_count          = 0
                    self.get_logger().info("Target locked → TRACK")
            else:
                self.confirm_count = 0
                if self.state == STATE_CONFIRM:
                    self.state = STATE_SEARCH

        # ════════════════════════════════════════════════════════════
        # Draw & Publish
        # ════════════════════════════════════════════════════════════
        self.draw_overlay(frame, cmd)

        # ป้องกัน cmd หลุดออกนอก TRACK / FINAL_APPROACH
        if self.state not in [STATE_TRACK, STATE_FINAL_APPROACH]:
            cmd.linear.x = cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        debug = Float32MultiArray()
        debug.data = [
            float(self.state),
            float(self.current_ref_y) if self.current_ref_y else 0.0,
            float(cmd.angular.z),
            float(cmd.linear.x),
            float(self.stopped_frame_count),
            float(self.track_frame_count),
            float(self.last_tag_data[5]) if self.last_tag_data else -1.0,
            float(self.lost_count),
            float(self.setpoint_y),   # debug: ดู setpoint ด้วย
        ]
        self.debug_pub.publish(debug)

        if self.get_parameter('publish_image').value:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = RobotFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()