#!/home/t/yolo_env/bin/python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import math
import time
import numpy as np
from ultralytics import YOLO
from pupil_apriltags import Detector


# ─────────────────────────────────────────────
# STATES
# ─────────────────────────────────────────────
STATE_SEARCH       = 0
STATE_FAR_APPROACH = 1
STATE_NEAR_SERVO   = 2
STATE_FINAL_ALIGN  = 3
STATE_READ_TAG     = 4
STATE_DONE         = 5

STATE_NAME = {
    STATE_SEARCH:       "SEARCH",
    STATE_FAR_APPROACH: "FAR_APPROACH",
    STATE_NEAR_SERVO:   "NEAR_SERVO",
    STATE_FINAL_ALIGN:  "FINAL_ALIGN",
    STATE_READ_TAG:     "READ_TAG",
    STATE_DONE:         "DONE",
}

TARGET_CLASS_ID     = 0
YOLO_INTERVAL       = 3
TAG_INTERVAL        = 2
LOCK_CONFIRM_FRAMES = 5
MAX_LOST_FRAMES     = 10
TAG_READ_REQUIRED   = 12

FAR_TO_NEAR_Z       = 0.8
FINAL_ALIGN_YAW_DEG = 2.0
STOP_EZ_THRESH      = 0.01
TAG_LOST_GRACE      = 3


class RobotFollower(Node):

    def __init__(self):
        super().__init__('robot_follower')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('camera_id',    4)
        self.declare_parameter('image_width',  640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('model_path',   '/home/t/392_project/ros2_ws/best.pt')
        self.declare_parameter('conf',         0.5)

        self.declare_parameter('max_linear',   0.4)
        self.declare_parameter('max_angular',  0.5)

        # FAR_APPROACH PID
        self.declare_parameter('far_kp',              0.00075)
        self.declare_parameter('far_ki',              0.0001)
        self.declare_parameter('far_kd',              0.001)
        self.declare_parameter('far_v',               0.25)
        self.declare_parameter('lateral_deadband_px', 30)

        # NEAR_SERVO
        self.declare_parameter('near_kz',          0.2)
        self.declare_parameter('near_kx',          0.7)
        self.declare_parameter('near_kpsi',        0.4)
        self.declare_parameter('near_v_max',       0.3)
        self.declare_parameter('near_v_min',       0.05)
        self.declare_parameter('slow_zone_dist',   0.25)
        self.declare_parameter('slow_zone_factor', 0.5)

        # FINAL_ALIGN
        self.declare_parameter('align_kpsi', 0.8)

        # Distance targets
        self.declare_parameter('z_target',            0.15)
        self.declare_parameter('z_cam_offset',        0.0)
        self.declare_parameter('tag_to_pot_offset_x', 0.04)

        # ── Camera Intrinsics จาก calibration จริง ──────────────────
        # fx, fy, cx, cy จาก camera matrix
        self.declare_parameter('fx', 651.50491737)
        self.declare_parameter('fy', 650.39077601)
        self.declare_parameter('cx', 320.62707882)
        self.declare_parameter('cy', 236.91812436)
        # Distortion coefficients: k1, k2, p1, p2, k3
        self.declare_parameter('dist_k1',  0.21581633)
        self.declare_parameter('dist_k2', -1.09508649)
        self.declare_parameter('dist_p1', -0.00213472)
        self.declare_parameter('dist_p2',  0.00169510)
        self.declare_parameter('dist_k3',  1.64003200)

        self.declare_parameter('tag_size',   0.042)
        self.declare_parameter('ema_alpha',  0.7)
        self.declare_parameter('yaw_sign_test', False)
        self.declare_parameter('publish_image', True)

        # ── Load params ─────────────────────────────────────────────
        cam_id = self.get_parameter('camera_id').value
        self.W = self.get_parameter('image_width').value
        self.H = self.get_parameter('image_height').value
        self.cx_frame = self.W / 2.0

        self.max_linear  = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.far_v       = self.get_parameter('far_v').value

        self.z_target_robot      = self.get_parameter('z_target').value
        self.z_cam_offset        = self.get_parameter('z_cam_offset').value
        self.z_target            = self.z_target_robot + self.z_cam_offset
        self.tag_to_pot_offset_x = self.get_parameter('tag_to_pot_offset_x').value

        self.far_kp           = self.get_parameter('far_kp').value
        self.far_ki           = self.get_parameter('far_ki').value
        self.far_kd           = self.get_parameter('far_kd').value
        self.lateral_deadband = self.get_parameter('lateral_deadband_px').value

        self.near_kz          = self.get_parameter('near_kz').value
        self.near_kx          = self.get_parameter('near_kx').value
        self.near_kpsi        = self.get_parameter('near_kpsi').value
        self.near_v_max       = self.get_parameter('near_v_max').value
        self.near_v_min       = self.get_parameter('near_v_min').value
        self.slow_zone_dist   = self.get_parameter('slow_zone_dist').value
        self.slow_zone_factor = self.get_parameter('slow_zone_factor').value
        self.align_kpsi       = self.get_parameter('align_kpsi').value

        # ── Camera matrix & distortion ───────────────────────────────
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value

        # camera_params สำหรับ pupil_apriltags: (fx, fy, cx, cy)
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

        self.tag_size      = self.get_parameter('tag_size').value
        self.yaw_sign_test = self.get_parameter('yaw_sign_test').value

        # คำนวณ optimal camera matrix สำหรับ undistort
        self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs,
            (self.W, self.H), 1, (self.W, self.H)
        )

        self.get_logger().info(
            f"Camera calibration loaded:\n"
            f"  fx={fx:.5f} fy={fy:.5f}\n"
            f"  cx={cx:.5f} cy={cy:.5f}\n"
            f"  dist=[{self.dist_coeffs[0,0]:.5f}, {self.dist_coeffs[0,1]:.5f}, "
            f"{self.dist_coeffs[0,2]:.5f}, {self.dist_coeffs[0,3]:.5f}, {self.dist_coeffs[0,4]:.5f}]\n"
            f"  z_target={self.z_target:.3f}m  tag_size={self.tag_size}m"
        )

        # ── Detectors ───────────────────────────────────────────────
        self.yolo = YOLO(self.get_parameter('model_path').value)
        self.conf = self.get_parameter('conf').value

        self.at_detector = Detector(
            families="tagStandard52h13",
            nthreads=4,
            quad_decimate=1.5,
            refine_edges=True,
        )

        # ── ROS Publishers ──────────────────────────────────────────
        self.cmd_pub   = self.create_publisher(Twist,             '/cmd_vel_pid',  10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/vision_debug', 10)
        self.image_pub = self.create_publisher(Image,             '/vision/image', 10)

        self.plant_pub    = self.create_publisher(Int32,             '/apriltag/planting_distance', 10)
        self.gap_pub      = self.create_publisher(Int32,             '/apriltag/gap_type',           10)
        self.interval_pub = self.create_publisher(Int32,             '/apriltag/cabbage_interval',   10)
        self.pose_pub     = self.create_publisher(Float32MultiArray, '/apriltag/pose',               10)

        self.bridge = CvBridge()

        # ── Camera ──────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)

        # ── State variables ─────────────────────────────────────────
        self.state       = STATE_SEARCH
        self.frame_count = 0

        self.confirm_count = 0
        self.lost_count    = 0
        self.last_bbox     = None

        self.pid_integral  = 0.0
        self.pid_prev_err  = 0.0
        self.pid_last_time = time.time()

        self.filt_x   = None
        self.filt_y   = None
        self.filt_z   = None
        self.filt_yaw = None

        self.tag_last_pose  = None
        self.tag_miss_count = 0

        self.tag_read_count = 0
        self.tag_ab = self.tag_c = self.tag_de = None
        self.tag_pose = None

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info("RobotFollower started")

    # ════════════════════════════════════════════════════════════════
    # HELPERS
    # ════════════════════════════════════════════════════════════════

    def clamp(self, v, lim):
        return max(-abs(lim), min(abs(lim), v))

    def pid_far(self, cx_obj):
        error = cx_obj - self.cx_frame
        if abs(error) <= self.lateral_deadband:
            self.pid_integral = 0.0
            self.pid_prev_err = 0.0
            return 0.0
        now = time.time()
        dt  = max(now - self.pid_last_time, 0.001)
        self.pid_last_time = now
        self.pid_integral += error * dt
        max_i = 150.0 / (self.far_ki + 1e-9)
        self.pid_integral = self.clamp(self.pid_integral, max_i)
        derivative = (error - self.pid_prev_err) / dt
        self.pid_prev_err = error
        output = self.far_kp * error + self.far_ki * self.pid_integral + self.far_kd * derivative
        return self.clamp(-output, self.max_angular)

    def reset_pid(self):
        self.pid_integral  = 0.0
        self.pid_prev_err  = 0.0
        self.pid_last_time = time.time()

    def curvature_control(self, e_x_comp, e_z, e_yaw_rad):
        v = self.near_kz * e_z
        v = self.clamp(v, self.near_v_max)
        if 0 < e_z < self.slow_zone_dist:
            v *= self.slow_zone_factor
        if e_z > STOP_EZ_THRESH:
            v = max(v, self.near_v_min)
        else:
            v = 0.0
        kappa = self.near_kx * e_x_comp + self.near_kpsi * e_yaw_rad
        omega = v * kappa
        omega = self.clamp(omega, self.max_angular)
        return v, omega

    def yolo_detect(self, frame):
        results   = self.yolo(frame, conf=self.conf, verbose=False)
        best_box  = None
        best_area = -1
        for box in results[0].boxes:
            if int(box.cls[0]) != TARGET_CLASS_ID:
                continue
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            area = (x2 - x1) * (y2 - y1)
            if area > best_area:
                best_area = area
                best_box  = (x1, y1, x2, y2)
        if best_box is None:
            return None
        x1, y1, x2, y2 = best_box
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0, x1, y1, x2, y2

    def apriltag_detect(self, frame):
        """
        Undistort ก่อน detect เพื่อความแม่นยำของ pose
        คืนค่า EMA-filtered (x, y, z, yaw_deg, tag_id) หรือ None
        """
        # Undistort ด้วย calibration จริง
        undistorted = cv2.undistort(
            frame, self.camera_matrix,
            self.dist_coeffs, None, self.new_camera_matrix
        )
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

        results = self.at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )
        if not results:
            return None

        best = min(results, key=lambda r: r.pose_t[2][0])
        t = best.pose_t
        R = best.pose_R

        raw_x   = float(t[0][0])
        raw_y   = float(t[1][0])
        raw_z   = float(t[2][0])
        raw_yaw = math.degrees(math.atan2(float(R[0][2]), float(R[2][2])))

        alpha = self.get_parameter('ema_alpha').value
        if self.filt_z is None:
            self.filt_x, self.filt_y = raw_x, raw_y
            self.filt_z, self.filt_yaw = raw_z, raw_yaw
        else:
            self.filt_x   = alpha * self.filt_x   + (1 - alpha) * raw_x
            self.filt_y   = alpha * self.filt_y   + (1 - alpha) * raw_y
            self.filt_z   = alpha * self.filt_z   + (1 - alpha) * raw_z
            self.filt_yaw = alpha * self.filt_yaw + (1 - alpha) * raw_yaw

        if self.yaw_sign_test:
            e_yaw_rad = math.radians(self.filt_yaw)
            self.get_logger().info(
                f"[SIGN TEST] raw_yaw={raw_yaw:.2f}deg filt_yaw={self.filt_yaw:.2f}deg "
                f"expected cmd.az={-self.near_kpsi * e_yaw_rad:.4f} rad/s"
            )

        # วาด debug บน frame
        corners = best.corners.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
        cv2.circle(frame, (int(best.center[0]), int(best.center[1])), 5, (0, 0, 255), -1)
        cv2.putText(frame,
            f"ID:{best.tag_id} z={self.filt_z:.3f}m "
            f"x={self.filt_x:.3f}m yaw={self.filt_yaw:.1f}deg",
            (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        return self.filt_x, self.filt_y, self.filt_z, self.filt_yaw, best.tag_id

    def reset_ema(self):
        self.filt_x = self.filt_y = self.filt_z = self.filt_yaw = None
        self.tag_last_pose  = None
        self.tag_miss_count = 0

    def decode_tag_id(self, tag_id):
        AB = tag_id // 1000
        C  = (tag_id // 100) % 10
        DE = tag_id % 100
        return AB, C, DE

    def apriltag_with_grace(self, frame):
        if self.frame_count % TAG_INTERVAL == 0:
            at = self.apriltag_detect(frame)
        else:
            at = self.tag_last_pose
        if at is not None:
            self.tag_last_pose  = at
            self.tag_miss_count = 0
            return at
        else:
            self.tag_miss_count += 1
            if self.tag_miss_count <= TAG_LOST_GRACE:
                return self.tag_last_pose
            return None

    def draw_crosshair(self, frame):
        cv2.line(frame, (int(self.cx_frame), 0), (int(self.cx_frame), self.H), (255, 255, 0), 1)

    def draw_yolo_box(self, frame, x1, y1, x2, y2, cx_obj):
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 0), 2)
        cy_obj = int((y1 + y2) / 2)
        cv2.circle(frame, (int(cx_obj), cy_obj), 5, (0, 0, 255), -1)
        cv2.line(frame, (int(self.cx_frame), cy_obj), (int(cx_obj), cy_obj), (255, 80, 0), 2)
        err_px = cx_obj - self.cx_frame
        cv2.putText(frame, f"lateral err={err_px:+.0f}px",
            (int(x1), int(y1) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)

    def draw_errors(self, frame, e_x, e_x_comp, e_z, e_yaw, v, omega, label):
        cv2.putText(frame,
            f"{label}  ex={e_x:.3f}m(comp={e_x_comp:.3f}) ez={e_z:.3f}m eyaw={e_yaw:.1f}deg",
            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 255), 2)
        cv2.putText(frame, f"v={v:.3f} m/s  omega={omega:.3f} rad/s",
            (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 140, 0), 2)

    # ════════════════════════════════════════════════════════════════
    # MAIN LOOP
    # ════════════════════════════════════════════════════════════════
    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1
        cmd = Twist()
        self.draw_crosshair(frame)

        # ── DONE ─────────────────────────────────────────────────────
        if self.state == STATE_DONE:
            cmd.linear.x = cmd.angular.z = 0.0
            cv2.putText(frame, "DONE", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            if self.tag_pose is not None:
                x, y, z, yaw = self.tag_pose
                cv2.putText(frame,
                    f"AB={self.tag_ab}cm C={self.tag_c} DE={self.tag_de}cm",
                    (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                cv2.putText(frame,
                    f"pose x={x:.3f}m z={z:.3f}m yaw={yaw:.1f}deg",
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)

        # ── READ_TAG ──────────────────────────────────────────────────
        elif self.state == STATE_READ_TAG:
            cmd.linear.x = cmd.angular.z = 0.0
            at = self.apriltag_with_grace(frame)
            if at is not None:
                x, y, z, yaw, tag_id = at
                ab, c, de = self.decode_tag_id(tag_id)
                self.tag_ab, self.tag_c, self.tag_de = ab, c, de
                self.tag_pose = (x, y, z, yaw)
                self.tag_read_count += 1
                cv2.putText(frame,
                    f"READ_TAG [{self.tag_read_count}/{TAG_READ_REQUIRED}] "
                    f"ID:{tag_id} AB:{ab} C:{c} DE:{de}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
                if self.tag_read_count >= TAG_READ_REQUIRED:
                    msg_ab = Int32(); msg_ab.data = ab
                    msg_c  = Int32(); msg_c.data  = c
                    msg_de = Int32(); msg_de.data = de
                    self.plant_pub.publish(msg_ab)
                    self.gap_pub.publish(msg_c)
                    self.interval_pub.publish(msg_de)
                    pose_msg = Float32MultiArray()
                    pose_msg.data = [float(x), float(y), float(z), float(yaw)]
                    self.pose_pub.publish(pose_msg)
                    self.get_logger().info(
                        f"Published → AB={ab}cm C={c} DE={de}cm | "
                        f"x={x:.3f}m z={z:.3f}m yaw={yaw:.1f}deg"
                    )
                    self.state = STATE_DONE
            else:
                self.tag_read_count = 0
                cv2.putText(frame, "READ_TAG — waiting for tag...",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 255), 2)

        # ── FINAL_ALIGN ───────────────────────────────────────────────
        elif self.state == STATE_FINAL_ALIGN:
            cmd.linear.x = 0.0
            at = self.apriltag_with_grace(frame)
            if at is not None:
                x, y, z, yaw, tag_id = at
                omega = self.clamp(
                    self.align_kpsi * math.radians(yaw),
                    self.max_angular
                )
                cmd.angular.z = -omega
                self.draw_errors(frame, x, x, z - self.z_target, yaw, 0.0, omega, "FINAL_ALIGN")
                if abs(yaw) < FINAL_ALIGN_YAW_DEG:
                    cmd.angular.z = 0.0
                    self.tag_read_count = 0
                    self.reset_ema()
                    self.get_logger().info(f"Aligned yaw={yaw:.2f}deg → READ_TAG")
                    self.state = STATE_READ_TAG
            else:
                cmd.angular.z = 0.0
                cv2.putText(frame, "FINAL_ALIGN — tag lost, holding",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2)

        # ── NEAR_SERVO ────────────────────────────────────────────────
        elif self.state == STATE_NEAR_SERVO:
            at = self.apriltag_with_grace(frame)
            if at is not None:
                x, y, z, yaw, tag_id = at
                e_x      = x
                e_z      = z - self.z_target
                e_yaw    = yaw
                e_yaw_rad = math.radians(e_yaw)
                e_x_comp = e_x - self.tag_to_pot_offset_x

                v, omega = self.curvature_control(e_x_comp, e_z, e_yaw_rad)
                cmd.linear.x  = v
                cmd.angular.z = -omega

                if self.tag_miss_count > 0:
                    cv2.putText(frame,
                        f"NEAR_SERVO [COASTING miss={self.tag_miss_count}/{TAG_LOST_GRACE}]",
                        (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 100, 255), 1)

                self.draw_errors(frame, e_x, e_x_comp, e_z, e_yaw, v, omega, "NEAR_SERVO")

                if e_z <= STOP_EZ_THRESH:
                    cmd.linear.x  = 0.0
                    cmd.angular.z = 0.0
                    self.get_logger().info(
                        f"Distance reached z={z:.3f}m → FINAL_ALIGN"
                    )
                    self.state = STATE_FINAL_ALIGN
            else:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                cv2.putText(frame, "NEAR_SERVO — tag lost, stopping",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2)

        # ── FAR_APPROACH ──────────────────────────────────────────────
        elif self.state == STATE_FAR_APPROACH:
            if self.frame_count % TAG_INTERVAL == 0:
                at = self.apriltag_detect(frame)
                if at is not None:
                    _, _, z, _, _ = at
                    if z < FAR_TO_NEAR_Z:
                        self.get_logger().info(
                            f"AprilTag z={z:.2f}m < {FAR_TO_NEAR_Z}m → NEAR_SERVO"
                        )
                        self.reset_pid()
                        self.state = STATE_NEAR_SERVO
                        cmd.linear.x = cmd.angular.z = 0.0
                        self.cmd_pub.publish(cmd)
                        return

            if self.frame_count % YOLO_INTERVAL == 0:
                result = self.yolo_detect(frame)
                if result is None:
                    self.lost_count += 1
                    if self.lost_count > MAX_LOST_FRAMES:
                        self.get_logger().warn("Object lost → SEARCH")
                        self.state = STATE_SEARCH
                        self.reset_pid()
                        self.reset_ema()
                        self.confirm_count = self.lost_count = 0
                        self.last_bbox = None
                else:
                    self.lost_count = 0
                    self.last_bbox  = result[2:]

            if self.last_bbox is not None:
                x1, y1, x2, y2 = self.last_bbox
                cx_obj = (x1 + x2) / 2.0
                cmd.angular.z = self.pid_far(cx_obj)
                cmd.linear.x  = self.far_v
                self.draw_yolo_box(frame, x1, y1, x2, y2, cx_obj)
                cv2.putText(frame,
                    f"FAR_APPROACH err={cx_obj - self.cx_frame:+.0f}px "
                    f"az={cmd.angular.z:.3f} v={cmd.linear.x:.2f}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
            else:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0

        # ── SEARCH ────────────────────────────────────────────────────
        if self.state == STATE_SEARCH:
            cmd.linear.x  = 0.0
            cmd.angular.z = self.max_angular * 0.4

            if self.frame_count % YOLO_INTERVAL == 0:
                result = self.yolo_detect(frame)
                if result is not None:
                    self.confirm_count += 1
                    cv2.putText(frame,
                        f"SEARCH: found! confirm {self.confirm_count}/{LOCK_CONFIRM_FRAMES}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
                    if self.confirm_count >= LOCK_CONFIRM_FRAMES:
                        self.last_bbox     = result[2:]
                        self.confirm_count = self.lost_count = 0
                        self.reset_pid()
                        self.reset_ema()
                        self.state = STATE_FAR_APPROACH
                        self.get_logger().info("Object locked → FAR_APPROACH")
                else:
                    self.confirm_count = 0
                    cv2.putText(frame, "SEARCH: scanning...",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)

        # ── State label + Publish ─────────────────────────────────────
        cv2.putText(frame,
            f"State: {STATE_NAME[self.state]}  z_tgt={self.z_target:.3f}m",
            (10, self.H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        self.cmd_pub.publish(cmd)

        debug = Float32MultiArray()
        debug.data = [
            float(self.state),
            float(self.filt_z)   if self.filt_z   is not None else -1.0,
            float(self.filt_x)   if self.filt_x   is not None else -1.0,
            float(self.filt_yaw) if self.filt_yaw is not None else  0.0,
            float(cmd.angular.z),
            float(cmd.linear.x),
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