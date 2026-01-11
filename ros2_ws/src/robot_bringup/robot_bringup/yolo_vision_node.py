#!/home/t/yolo_env/bin/python
# rqt_image_view

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import math
from ultralytics import YOLO


# =====================
# CONFIG
# =====================
TARGET_CLASS_ID = 0

YOLO_INTERVAL = 10
REVALIDATE_INTERVAL = 30

LOCK_CONFIRM_FRAMES = 3
MAX_LOST_FRAMES = 8

MAX_CENTER_JUMP = 120
MAX_REVALIDATE_OFFSET = 80

STATE_SEARCH = 0
STATE_CONFIRM = 1
STATE_TRACK = 2
STATE_LOST = 3

STATE_NAME = {
    STATE_SEARCH: "SEARCH",
    STATE_CONFIRM: "CONFIRM",
    STATE_TRACK: "TRACK",
    STATE_LOST: "LOST",
}


class YoloFollower(Node):
    def __init__(self):
        super().__init__('yolo_follower')

        # =====================
        # Parameters
        # =====================
        self.declare_parameter('camera_id', 1)
        self.declare_parameter('model_path', '/home/prukubt/392_Agri/ros2_ws/best.pt')
        self.declare_parameter('conf', 0.5)

        self.declare_parameter('target_area', 50000.0)
        self.declare_parameter('area_tolerance', 3000.0)

        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 0.7)

        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_fov_deg', 60.0)

        # toggle image publisher
        self.declare_parameter('publish_image', True)

        # =====================
        # Load params
        # =====================
        cam_id = self.get_parameter('camera_id').value
        model_path = self.get_parameter('model_path').value
        self.conf = self.get_parameter('conf').value

        self.target_area = self.get_parameter('target_area').value
        self.area_tol = self.get_parameter('area_tolerance').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        self.W = self.get_parameter('image_width').value
        self.H = self.get_parameter('image_height').value
        self.fov_rad = math.radians(
            self.get_parameter('camera_fov_deg').value
        )

        # =====================
        # ROS
        # =====================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/vision_debug', 10)
        self.image_pub = self.create_publisher(Image, '/vision/image', 10)

        self.bridge = CvBridge()

        # =====================
        # YOLO + Camera
        # =====================
        self.model = YOLO(model_path)

        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)

        # =====================
        # State
        # =====================
        self.state = STATE_SEARCH
        self.tracker = None
        self.frame_count = 0

        self.confirm_count = 0
        self.lost_count = 0
        self.prev_cx = None

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info("YOLO follower with image + overlay + toggle started")

    # =====================
    # Main loop
    # =====================
    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1
        cmd = Twist()
        cx = 0.0
        area = 0.0

        # =====================
        # TRACK
        # =====================
        if self.state == STATE_TRACK:
            ok, bbox = self.tracker.update(frame)

            if ok:
                x, y, w, h = bbox
                cx = x + w / 2.0
                area = w * h

                if self.prev_cx is not None:
                    if abs(cx - self.prev_cx) > MAX_CENTER_JUMP:
                        self.state = STATE_LOST
                        self.tracker = None

                self.prev_cx = cx
                self.lost_count = 0
                self.compute_cmd(cx, area, cmd)

                if self.frame_count % REVALIDATE_INTERVAL == 0:
                    if not self.revalidate(frame, cx):
                        self.state = STATE_LOST
                        self.tracker = None

                self.draw_bbox(frame, x, y, w, h, cx)

            else:
                self.lost_count += 1
                if self.lost_count > MAX_LOST_FRAMES:
                    self.state = STATE_LOST
                    self.tracker = None

        # =====================
        # LOST → SEARCH
        # =====================
        if self.state == STATE_LOST:
            self.tracker = None
            self.prev_cx = None
            self.confirm_count = 0
            self.lost_count = 0
            self.state = STATE_SEARCH

        # =====================
        # SEARCH / CONFIRM
        # =====================
        if self.state in [STATE_SEARCH, STATE_CONFIRM] and \
           self.frame_count % YOLO_INTERVAL == 0:

            box = self.detect_best(frame)

            if box is not None:
                self.confirm_count += 1
                if self.confirm_count >= LOCK_CONFIRM_FRAMES:
                    self.init_tracker(frame, box)
                    self.state = STATE_TRACK
                    self.confirm_count = 0
            else:
                self.confirm_count = 0
                self.state = STATE_SEARCH

        # =====================
        # Overlay & Publish
        # =====================
        self.draw_overlay(frame, cx, area)
        self.cmd_pub.publish(cmd)

        debug = Float32MultiArray()
        debug.data = [
            float(self.state),
            float(cx),
            float(area),
            float(cmd.angular.z),
            float(cmd.linear.x)
        ]
        self.debug_pub.publish(debug)

        # ---- image toggle ----
        if self.get_parameter('publish_image').value:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(img_msg)

    # =====================
    # Drawing helpers
    # =====================
    def draw_bbox(self, frame, x, y, w, h, cx):
        cv2.rectangle(frame, (int(x), int(y)),
                      (int(x + w), int(y + h)), (0, 255, 0), 2)
        cv2.circle(frame, (int(cx), int(y + h / 2)),
                   5, (0, 0, 255), -1)

    def draw_overlay(self, frame, cx, area):
        # crosshair
        cv2.line(frame, (self.W // 2, 0), (self.W // 2, self.H), (255, 255, 0), 1)
        cv2.line(frame, (0, self.H // 2), (self.W, self.H // 2), (255, 255, 0), 1)

        text = f"STATE: {STATE_NAME[self.state]}  cx={cx:.1f} area={area:.0f}"
        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # =====================
    # YOLO detect
    # =====================
    def detect_best(self, frame):
        results = self.model(frame, conf=self.conf, verbose=False)
        best_box = None
        best_score = -1.0
        img_center = self.W / 2.0

        for box in results[0].boxes:
            if int(box.cls[0]) != TARGET_CLASS_ID:
                continue

            x1, y1, x2, y2 = box.xyxy[0]
            area = float((x2 - x1) * (y2 - y1))
            cx = float((x1 + x2) / 2.0)

            score = area - 3.0 * abs(cx - img_center)
            if score > best_score:
                best_score = score
                best_box = (x1, y1, x2, y2)

        return best_box

    def revalidate(self, frame, cx):
        results = self.model(frame, conf=self.conf, verbose=False)
        for box in results[0].boxes:
            if int(box.cls[0]) != TARGET_CLASS_ID:
                continue
            x1, _, x2, _ = box.xyxy[0]
            yolo_cx = float((x1 + x2) / 2.0)
            if abs(yolo_cx - cx) < MAX_REVALIDATE_OFFSET:
                return True
        return False

    def init_tracker(self, frame, box):
        x1, y1, x2, y2 = box
        bbox = (int(x1), int(y1),
                int(x2 - x1), int(y2 - y1))

        if hasattr(cv2, 'TrackerCSRT_create'):
            self.tracker = cv2.TrackerCSRT_create()
        else:
            self.tracker = cv2.legacy.TrackerCSRT_create()

        self.tracker.init(frame, bbox)
        self.prev_cx = None
        self.lost_count = 0

    def compute_cmd(self, cx, area, cmd):
        pixel_error = cx - (self.W / 2.0)
        heading_error = -(pixel_error / (self.W / 2.0)) * (self.fov_rad / 2.0)

        cmd.angular.z = max(-self.max_angular,
                            min(self.max_angular, heading_error))

        area_error = self.target_area - area
        if abs(area_error) > self.area_tol:
            cmd.linear.x = max(0.0,
                               min(self.max_linear, 0.00003 * area_error))
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = YoloFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
