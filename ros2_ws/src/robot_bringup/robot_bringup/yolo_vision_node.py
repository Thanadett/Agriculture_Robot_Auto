#!/home/t/yolo_env/bin/python

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


# ─────────────────────────────────────────────
# STATES
# ─────────────────────────────────────────────
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

# ─────────────────────────────────────────────
# FINAL_APPROACH sub-states
# ─────────────────────────────────────────────
FA_APPROACH   = "approach"    # เดินหน้าจนถึง 30cm
FA_READ_TAG   = "read_tag"    # หยุดอ่าน AprilTag
FA_ADVANCE    = "advance"     # เดินหน้าต่อตามระยะ z ที่อ่านได้
FA_DONE       = "done"        # เสร็จ

TARGET_CLASS_ID     = 0
YOLO_INTERVAL       = 5
REDETECT_INTERVAL   = 15
LOCK_CONFIRM_FRAMES = 3
MAX_LOST_FRAMES     = 8
MAX_CENTER_JUMP     = 120
MIN_TRACK_FRAMES    = 30

# AprilTag
TAG_INTERVAL      = 3
TAG_READ_REQUIRED = 8


class RobotFollower(Node):

    def __init__(self):
        super().__init__('robot_follower')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('camera_id',   5)
        self.declare_parameter('image_width',  640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('model_path',  '/home/t/392_project/ros2_ws/best.pt')
        self.declare_parameter('conf',        0.5)

        self.declare_parameter('max_linear',  0.2)
        self.declare_parameter('max_angular', 0.4)

        self.declare_parameter('camera_vertical_fov_deg', 45.0)
        self.declare_parameter('center_deadband_px',               12)
        self.declare_parameter('target_vertical_angle',            -0.25)
        self.declare_parameter('angle_deadband_rad',               0.03)
        self.declare_parameter('angular_deadband_when_stopped_px', 30)

        self.declare_parameter('max_bbox_change_ratio', 1.5)
        self.declare_parameter('tracker_type',          'KCF')

        self.declare_parameter('final_stop_distance_m', 0.30)   # หยุดที่ z < ค่านี้
        self.declare_parameter('final_forward_speed',   0.2)
        self.declare_parameter('final_tag_timeout_sec', 1.5)
        self.declare_parameter('min_stopped_frames',    3)

        # ── [ใหม่] ระยะ advance หลังอ่าน tag ─────────────────────────
        # advance_speed: ความเร็วเดินหน้าหลังอ่าน tag ได้แล้ว (m/s)
        self.declare_parameter('advance_speed',        0.15)
        # read_tag_timeout_sec: timeout อ่าน tag ถ้าอ่านไม่ครบให้ใช้ค่าล่าสุด
        self.declare_parameter('read_tag_timeout_sec', 5.0)
        # advance_margin_m: offset ลบออกจากระยะ z (เผื่อหยุดก่อนชน)
        self.declare_parameter('advance_margin_m',     0.05)

        # ── Camera Intrinsics ────────────────────────────────────────
        self.declare_parameter('fx', 651.50491737)
        self.declare_parameter('fy', 650.39077601)
        self.declare_parameter('cx', 320.62707882)
        self.declare_parameter('cy', 236.91812436)
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
            self.get_parameter('camera_vertical_fov_deg').value
        )

        self.center_deadband          = self.get_parameter('center_deadband_px').value
        self.target_v_angle           = self.get_parameter('target_vertical_angle').value
        self.angle_deadband           = self.get_parameter('angle_deadband_rad').value
        self.angular_deadband_stopped = self.get_parameter('angular_deadband_when_stopped_px').value

        self.max_bbox_change     = self.get_parameter('max_bbox_change_ratio').value
        self.tracker_type        = self.get_parameter('tracker_type').value
        self.final_stop_dist     = self.get_parameter('final_stop_distance_m').value
        self.final_speed         = self.get_parameter('final_forward_speed').value
        self.final_tag_timeout   = self.get_parameter('final_tag_timeout_sec').value
        self.min_stopped_frames  = self.get_parameter('min_stopped_frames').value

        self.advance_speed       = self.get_parameter('advance_speed').value
        self.read_tag_timeout    = self.get_parameter('read_tag_timeout_sec').value
        self.advance_margin      = self.get_parameter('advance_margin_m').value

        # ── Camera Intrinsics ────────────────────────────────────────
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value

        self.camera_params = (fx, fy, cx, cy)
        self.camera_matrix = np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1]
        ], dtype=np.float64)
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
            (self.W, self.H), 1, (self.W, self.H)
        )

        # ── Detectors ────────────────────────────────────────────────
        self.model = YOLO(model_path)

        self.at_detector = Detector(
            families="tagStandard52h13",
            nthreads=4,
            quad_decimate=1.5,
            refine_edges=True,
        )

        # ── ROS Publishers & Subscribers ─────────────────────────────
        self.cmd_pub   = self.create_publisher(Twist,             '/cmd_vel_pid',      10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/vision_debug', 10)
        self.image_pub = self.create_publisher(Image,             '/vision/image', 10)

        self.plant_pub    = self.create_publisher(Int32,             '/apriltag/planting_distance', 10)
        self.gap_pub      = self.create_publisher(Int32,             '/apriltag/gap_type',           10)
        self.interval_pub = self.create_publisher(Int32,             '/apriltag/cabbage_interval',   10)
        self.pose_pub     = self.create_publisher(Float32MultiArray, '/apriltag/pose',               10)

        self.wheel_sub = self.create_subscription(
            Float32MultiArray, '/wheel_ticks', self.wheel_ticks_callback, 10
        )
        self.reset_client = self.create_client(Empty, '/wheel_ticks/reset')

        self.bridge = CvBridge()

        # ── Camera ───────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)

        # ── YOLO State ───────────────────────────────────────────────
        self.state         = STATE_SEARCH
        self.tracker       = None
        self.frame_count   = 0
        self.confirm_count = 0

        self.prev_cx           = None
        self.prev_bbox_area    = None
        self.current_ref_y     = None
        self.lost_count        = 0
        self.track_frame_count   = 0
        self.stopped_frame_count = 0

        # ── Wheel encoder ────────────────────────────────────────────
        self.wheel_ticks = [0.0, 0.0, 0.0, 0.0]

        # ── AprilTag state ───────────────────────────────────────────
        self.tag_read_count    = 0
        self.tag_published     = False
        self.last_tag_data     = None
        self.last_tag_time     = None
        self.final_approach_done = False

        # ── Final approach sub-state ─────────────────────────────────
        self.fa_sub_state       = FA_APPROACH
        self.fa_read_tag_start  = None   # เวลาเริ่มอ่าน tag (สำหรับ timeout)
        self.fa_advance_target  = None   # ระยะที่ต้องเดิน (เมตร, แปลงจาก z → cm ผ่าน wheel)
        self.fa_advance_start   = None   # wheel distance ตอนเริ่ม advance (cm)

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info(
            f"RobotFollower started | tracker={self.tracker_type} | "
            f"tag_size={self.tag_size}m | stop_dist={self.final_stop_dist}m"
        )

    # ════════════════════════════════════════════════════════════════
    # WHEEL TICKS
    # ════════════════════════════════════════════════════════════════
    def wheel_ticks_callback(self, msg):
        self.wheel_ticks = list(msg.data)

    def get_average_wheel_distance(self):
        if len(self.wheel_ticks) >= 4:
            return sum(self.wheel_ticks) / 4.0
        return 0.0

    def reset_wheel_ticks(self):
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Reset service not available')
            return
        req    = Empty.Request()
        future = self.reset_client.call_async(req)
        def cb(f):
            try:
                f.result()
                self.get_logger().info('Wheel ticks reset OK')
            except Exception as e:
                self.get_logger().error(f'Reset failed: {e}')
        future.add_done_callback(cb)

    # ════════════════════════════════════════════════════════════════
    # TRACKER HELPERS
    # ════════════════════════════════════════════════════════════════
    def reset_tracking_state(self):
        self.prev_cx             = None
        self.prev_bbox_area      = None
        self.current_ref_y       = None
        self.stopped_frame_count = 0
        self.lost_count          = 0
        self.track_frame_count   = 0

    def reset_tracker_only(self):
        self.prev_cx        = None
        self.prev_bbox_area = None
        self.lost_count     = 0

    def init_tracker(self, frame, box, full_reset=True):
        x1, y1, x2, y2 = box
        bbox = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))
        if self.tracker_type.upper() == 'KCF':
            self.tracker = (cv2.TrackerKCF_create()
                            if hasattr(cv2, 'TrackerKCF_create')
                            else cv2.legacy.TrackerKCF_create())
        else:
            self.tracker = (cv2.TrackerCSRT_create()
                            if hasattr(cv2, 'TrackerCSRT_create')
                            else cv2.legacy.TrackerCSRT_create())
        self.tracker.init(frame, bbox)
        if full_reset:
            self.reset_tracking_state()
        else:
            self.reset_tracker_only()

    # ════════════════════════════════════════════════════════════════
    # CONTROL
    # ════════════════════════════════════════════════════════════════
    def compute_cmd(self, cx, top_y, cmd):
        pixel_error_y  = top_y - (self.H / 2.0)
        vertical_angle = -(pixel_error_y / (self.H / 2.0)) * (self.vfov_rad / 2.0)
        angle_error    = self.target_v_angle - vertical_angle
        is_stopped     = abs(angle_error) <= self.angle_deadband

        pixel_error_x = cx - (self.W / 2.0)
        if is_stopped:
            if abs(pixel_error_x) < self.angular_deadband_stopped:
                pixel_error_x = 0.0
        else:
            if abs(pixel_error_x) < self.center_deadband:
                pixel_error_x = 0.0

        heading       = -(pixel_error_x / (self.W / 2.0)) * (self.vfov_rad / 2.0)
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, heading))

        if not is_stopped:
            cmd.linear.x = self.max_linear if angle_error < 0 else 0.0
        else:
            cmd.linear.x = 0.0

    # ════════════════════════════════════════════════════════════════
    # YOLO DETECTION
    # ════════════════════════════════════════════════════════════════
    def detect_best(self, frame):
        results   = self.model(frame, conf=self.conf, verbose=False)
        best_box  = None
        best_area = -1
        for box in results[0].boxes:
            if int(box.cls[0]) != TARGET_CLASS_ID:
                continue
            x1, y1, x2, y2 = box.xyxy[0]
            area = float((x2 - x1) * (y2 - y1))
            if area > best_area:
                best_area = area
                best_box  = (x1, y1, x2, y2)
        return best_box

    # ════════════════════════════════════════════════════════════════
    # APRILTAG DETECTION
    # ════════════════════════════════════════════════════════════════
    def apriltag_read_and_pub(self, frame):
        undist = cv2.undistort(
            frame, self.camera_matrix,
            self.dist_coeffs, None, self.new_camera_matrix
        )
        gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)

        results = self.at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )
        if not results:
            return

        best  = min(results, key=lambda r: r.pose_t[2][0])
        t     = best.pose_t
        R     = best.pose_R
        x_m   = float(t[0][0])
        y_m   = float(t[1][0])
        z_m   = float(t[2][0])
        yaw_d = math.degrees(math.atan2(float(R[0][2]), float(R[2][2])))

        tag_id = best.tag_id
        ab     = tag_id // 1000
        c      = (tag_id // 100) % 10
        de     = tag_id % 100

        self.tag_read_count += 1
        self.last_tag_data  = (ab, c, de, x_m, y_m, z_m, yaw_d)
        self.last_tag_time  = self.get_clock().now()

        corners = best.corners.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), (255, 0, 255), 2)
        cv2.putText(frame,
            f"TAG ID:{tag_id} z={z_m:.3f}m x={x_m:.3f}m yaw={yaw_d:.1f}deg "
            f"[{self.tag_read_count}/{TAG_READ_REQUIRED}]",
            (10, self.H - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

        if self.tag_read_count >= TAG_READ_REQUIRED and not self.tag_published:
            msg_ab = Int32(); msg_ab.data = ab
            msg_c  = Int32(); msg_c.data  = c
            msg_de = Int32(); msg_de.data = de
            self.plant_pub.publish(msg_ab)
            self.gap_pub.publish(msg_c)
            self.interval_pub.publish(msg_de)

            pose_msg = Float32MultiArray()
            pose_msg.data = [float(x_m), float(y_m), float(z_m), float(yaw_d)]
            self.pose_pub.publish(pose_msg)

            self.tag_published = True
            self.get_logger().info(
                f"[AprilTag] Published → AB={ab}cm C={c} DE={de}cm | "
                f"x={x_m:.3f}m z={z_m:.3f}m yaw={yaw_d:.1f}deg"
            )

    # ════════════════════════════════════════════════════════════════
    # DRAWING
    # ════════════════════════════════════════════════════════════════
    def draw_bbox(self, frame, x, y, w, h, cx, top_y):
        cv2.rectangle(frame, (int(x), int(y)), (int(x+w), int(y+h)), (0, 255, 0), 2)
        cv2.circle(frame, (int(cx), int(top_y)), 5, (255, 128, 0), -1)
        cv2.line(frame, (int(x), int(top_y)), (int(x+w), int(top_y)), (255, 128, 0), 2)
        cv2.putText(frame,
            f"State: {STATE_NAME[self.state]} | Area: {int(w*h)}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def draw_overlay(self, frame, cmd):
        cv2.line(frame, (self.W//2, 0), (self.W//2, self.H), (255, 255, 0), 1)

        target_pixel_y = int(
            (self.H / 2.0) - self.target_v_angle * (self.H / 2.0) / (self.vfov_rad / 2.0)
        )
        dash_len = 20
        for x_start in range(0, self.W, dash_len * 2):
            x_end = min(x_start + dash_len, self.W)
            cv2.line(frame, (x_start, target_pixel_y), (x_end, target_pixel_y), (0, 220, 255), 2)
        cv2.putText(frame, f"TARGET y={target_pixel_y}px",
            (5, target_pixel_y - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 2)

        if self.state == STATE_TRACK and self.current_ref_y is not None:
            pixel_error_y  = self.current_ref_y - (self.H / 2.0)
            vertical_angle = -(pixel_error_y / (self.H / 2.0)) * (self.vfov_rad / 2.0)
            angle_error    = self.target_v_angle - vertical_angle

            if abs(angle_error) <= self.angle_deadband:
                txt   = (f"STOPPED ({self.stopped_frame_count}/{self.min_stopped_frames}) "
                         f"| track {self.track_frame_count}/{MIN_TRACK_FRAMES}")
                color = (0, 255, 0)
            else:
                txt   = f"MOVING err={angle_error:.3f} | track={self.track_frame_count}"
                color = (0, 165, 255)
            cv2.putText(frame, txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # ════════════════════════════════════════════════════════════════
    # FINAL APPROACH (sub-state machine)
    #
    #  FA_APPROACH  → เดินหน้าจนกว่า AprilTag z < final_stop_distance_m
    #  FA_READ_TAG  → หยุด, รออ่าน AprilTag ให้ครบ TAG_READ_REQUIRED
    #                 (หรือ timeout → ใช้ค่าล่าสุดที่มี)
    #  FA_ADVANCE   → reset wheel, เดินหน้าต่อด้วยระยะ = z - advance_margin (เมตร → cm)
    #  FA_DONE      → หยุดถาวร
    # ════════════════════════════════════════════════════════════════
    def handle_final_approach(self, frame, cmd):

        # อ่าน tag ทุก TAG_INTERVAL เฟรม (ทำใน update() แล้ว — ที่นี่แค่ดึงค่า)
        tag_z = self.last_tag_data[5] if self.last_tag_data is not None else None

        # ── FA_APPROACH ────────────────────────────────────────────
        if self.fa_sub_state == FA_APPROACH:

            tag_timed_out = False
            if self.last_tag_time is not None:
                elapsed = (self.get_clock().now() - self.last_tag_time).nanoseconds / 1e9
                if elapsed > self.final_tag_timeout:
                    tag_timed_out = True

            if tag_timed_out:
                # ไม่เห็น tag นานเกินไประหว่างเข้าหา → safety stop
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                cv2.putText(frame, "FINAL APPROACH: TAG TIMEOUT - STOPPED",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

            elif tag_z is None or tag_z > self.final_stop_dist:
                # ยังไกล → เดินหน้า
                cmd.linear.x  = self.final_speed
                cmd.angular.z = 0.0
                z_txt = f"{tag_z:.3f}m" if tag_z is not None else "N/A"
                cv2.putText(frame, f"FINAL: approaching z={z_txt}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)

            else:
                # ถึง 30cm แล้ว → หยุด เริ่มอ่าน tag
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self.fa_sub_state      = FA_READ_TAG
                self.fa_read_tag_start = self.get_clock().now()
                # reset counter เพื่ออ่านใหม่ตั้งแต่ต้น (ให้ได้ค่าที่ระยะนี้จริงๆ)
                self.tag_read_count = 0
                self.tag_published  = False
                self.last_tag_data  = None
                self.get_logger().info(
                    f"[FINAL] Reached {self.final_stop_dist}m → reading AprilTag..."
                )

        # ── FA_READ_TAG ────────────────────────────────────────────
        elif self.fa_sub_state == FA_READ_TAG:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0

            elapsed = (self.get_clock().now() - self.fa_read_tag_start).nanoseconds / 1e9
            reads_ok = self.tag_read_count >= TAG_READ_REQUIRED
            timed_out = elapsed > self.read_tag_timeout

            if reads_ok or (timed_out and self.last_tag_data is not None):
                # ได้ข้อมูล tag แล้ว → คำนวณระยะ advance
                z_m = self.last_tag_data[5]
                # เดินหน้าต่อ = ระยะ z (เมตร) - margin, แปลงเป็น cm
                advance_m  = max(0.0, z_m - self.advance_margin)
                advance_cm = advance_m * 100.0

                self.get_logger().info(
                    f"[FINAL] Tag z={z_m:.3f}m → advance {advance_cm:.1f}cm "
                    f"(reads={self.tag_read_count}, elapsed={elapsed:.1f}s)"
                )

                # reset wheel ticks แล้วเริ่ม advance
                self.reset_wheel_ticks()
                self.fa_advance_target = advance_cm
                self.fa_advance_start  = self.get_average_wheel_distance()
                self.fa_sub_state      = FA_ADVANCE

            elif timed_out and self.last_tag_data is None:
                # timeout และไม่มีข้อมูล tag เลย → abort
                self.get_logger().warn(
                    f"[FINAL] Tag read timeout ({elapsed:.1f}s) - no data → DONE"
                )
                self.fa_sub_state        = FA_DONE
                self.final_approach_done = True
            else:
                # กำลังรออ่าน tag
                cv2.putText(frame,
                    f"FINAL: reading tag {self.tag_read_count}/{TAG_READ_REQUIRED} "
                    f"({elapsed:.1f}s/{self.read_tag_timeout:.0f}s)",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 2)

        # ── FA_ADVANCE ─────────────────────────────────────────────
        elif self.fa_sub_state == FA_ADVANCE:
            current_dist = self.get_average_wheel_distance()
            traveled     = current_dist - self.fa_advance_start
            remain       = self.fa_advance_target - traveled

            DECEL_ZONE = 10.0  # cm
            SPEED_SLOW = 0.06  # m/s

            if remain > DECEL_ZONE:
                cmd.linear.x = self.advance_speed
            elif remain > 0:
                ratio        = remain / DECEL_ZONE
                cmd.linear.x = max(SPEED_SLOW + (self.advance_speed - SPEED_SLOW) * ratio,
                                   SPEED_SLOW)
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f"[FINAL] Advance complete: traveled={traveled:.1f}cm / "
                    f"target={self.fa_advance_target:.1f}cm"
                )
                self.fa_sub_state        = FA_DONE
                self.final_approach_done = True

            cmd.angular.z = 0.0
            cv2.putText(frame,
                f"FINAL ADVANCE: {traveled:.1f}/{self.fa_advance_target:.1f}cm "
                f"remain={remain:.1f}cm",
                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

        # ── FA_DONE ─────────────────────────────────────────────────
        elif self.fa_sub_state == FA_DONE:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            cv2.putText(frame, "FINAL: DONE",
                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # ════════════════════════════════════════════════════════════════
    # MAIN LOOP
    # ════════════════════════════════════════════════════════════════
    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1
        cmd = Twist()

        # อ่าน AprilTag ทุก TAG_INTERVAL เฟรม
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

            if self.track_frame_count % REDETECT_INTERVAL == 0:
                box = self.detect_best(frame)
                if box is not None:
                    self.init_tracker(frame, box, full_reset=False)
                    self.get_logger().info(
                        f"Re-init tracker (track={self.track_frame_count})"
                    )

            ok, bbox = self.tracker.update(frame)

            if ok:
                x, y, w, h   = bbox
                current_area = w * h

                if self.prev_bbox_area is not None:
                    ratio = current_area / self.prev_bbox_area
                    if ratio > self.max_bbox_change or ratio < (1.0 / self.max_bbox_change):
                        self.get_logger().warn(f"Box jump {ratio:.2f}x → lost")
                        self.state   = STATE_LOST
                        self.tracker = None
                        self.stopped_frame_count = 0
                    else:
                        self.prev_bbox_area = current_area
                        cx    = x + w / 2.0
                        top_y = float(y)
                        self.current_ref_y = top_y

                        if self.prev_cx is not None and \
                           abs(cx - self.prev_cx) > MAX_CENTER_JUMP:
                            self.get_logger().warn("CX jump → lost")
                            self.state   = STATE_LOST
                            self.tracker = None
                            self.stopped_frame_count = 0
                        else:
                            self.prev_cx    = cx
                            self.lost_count = 0
                            self.compute_cmd(cx, top_y, cmd)
                            self.draw_bbox(frame, x, y, w, h, cx, top_y)

                            if abs(cmd.linear.x) < 0.01:
                                self.stopped_frame_count += 1
                            else:
                                self.stopped_frame_count = 0

                            # Trigger FINAL_APPROACH
                            if self.track_frame_count >= MIN_TRACK_FRAMES and \
                               self.stopped_frame_count >= self.min_stopped_frames:
                                self.get_logger().info(
                                    f'[TRIGGER] track={self.track_frame_count} '
                                    f'stopped={self.stopped_frame_count} → FINAL_APPROACH'
                                )
                                self.tag_read_count      = 0
                                self.tag_published       = False
                                self.last_tag_time       = None
                                self.final_approach_done = False
                                self.fa_sub_state        = FA_APPROACH   # reset sub-state
                                self.state               = STATE_FINAL_APPROACH
                                self.stopped_frame_count = 0
                else:
                    self.prev_bbox_area = current_area
                    cx    = x + w / 2.0
                    top_y = float(y)
                    self.current_ref_y = top_y
                    self.prev_cx       = cx
                    self.draw_bbox(frame, x, y, w, h, cx, top_y)
            else:
                self.lost_count += 1
                self.stopped_frame_count = 0
                if self.lost_count > MAX_LOST_FRAMES:
                    self.get_logger().warn("Lost target")
                    self.state   = STATE_LOST
                    self.tracker = None

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
        if self.state in [STATE_SEARCH, STATE_CONFIRM] and \
           self.frame_count % YOLO_INTERVAL == 0:
            box = self.detect_best(frame)
            if box is not None:
                self.confirm_count += 1
                if self.confirm_count >= LOCK_CONFIRM_FRAMES:
                    self.init_tracker(frame, box, full_reset=True)
                    self.tag_read_count = 0
                    self.tag_published  = False
                    self.state         = STATE_TRACK
                    self.confirm_count = 0
                    self.get_logger().info("Target locked → TRACK")
            else:
                self.confirm_count = 0
                if self.state == STATE_CONFIRM:
                    self.state = STATE_SEARCH

        # ════════════════════════════════════════════════════════════
        # Publish
        # ════════════════════════════════════════════════════════════
        self.draw_overlay(frame, cmd)

        if self.state not in [STATE_TRACK, STATE_FINAL_APPROACH]:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0

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
        ]
        self.debug_pub.publish(debug)

        if self.get_parameter('publish_image').value:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            )


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