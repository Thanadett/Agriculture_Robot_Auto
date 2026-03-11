#!/usr/bin/env python3
"""
Side Camera Publisher — ROS2
============================================================
Node สำหรับอ่านภาพจากกล้อง Side Camera (2K) ผ่าน udev symlink
และส่งข้อมูลผ่าน ROS2 topics ทั้งแบบ Raw และ Compressed

Default Camera: /dev/webcam_EYD_2k
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSidePublisher(Node):

    def __init__(self):
        super().__init__('cam_side_publisher')

        # ── Parameters ──────────────────────────────────────────
        # ปรับให้เป็น string เพื่อรองรับ /dev/webcam_EYD_1080p
        self.declare_parameter('camera_id',     '/dev/webcam_EYD_1080p')
        self.declare_parameter('image_width',   640)
        self.declare_parameter('image_height',  480)
        self.declare_parameter('fps',           30)
        self.declare_parameter('jpeg_quality',  80)
        self.declare_parameter('publish_raw',   True)
        self.declare_parameter('use_x11_debug', False)

        # ดึงค่าจาก Parameter
        self.cam_id_param = self.get_parameter('camera_id').value
        self.W            = self.get_parameter('image_width').value
        self.H            = self.get_parameter('image_height').value
        self.fps          = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.pub_raw      = self.get_parameter('publish_raw').value
        self.use_x11      = self.get_parameter('use_x11_debug').value

        # แปลง cam_id_param เป็น int ถ้าผู้ใช้ใส่มาเป็นตัวเลข
        self.cam_id = self.parse_camera_id(self.cam_id_param)

        # ── Publishers ──────────────────────────────────────────
        self.pub_compressed = self.create_publisher(
            CompressedImage, '/camera_side/image_raw/compressed', 10)

        if self.pub_raw:
            self.pub_image = self.create_publisher(
                Image, '/camera_side/image_raw', 10)

        # ── Camera Setup ────────────────────────────────────────
        self.cap = cv2.VideoCapture(self.cam_id, cv2.CAP_V4L2)

        # บังคับใช้ MJPEG เพื่อลด USB bandwidth
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.bridge = CvBridge()
        self.frame_cnt = 0

        # ── X11 debug window ────────────────────────────────────
        if self.use_x11:
            cv2.namedWindow("Side Camera Debug", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Side Camera Debug", 480, 360)

        # ── Timer ───────────────────────────────────────────────
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"Side Camera Publisher Started\n"
            f" - Device: {self.cam_id_param}\n"
            f" - Resolution: {self.W}x{self.H} @ {self.fps}fps\n"
            f" - Raw Publish: {'ON' if self.pub_raw else 'OFF'}"
        )

    def parse_camera_id(self, value):
        """แปลงค่าจาก parameter ให้เป็น int หรือ string ตามความเหมาะสม"""
        try:
            return int(value)
        except (ValueError, TypeError):
            return value

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read from Side Camera", throttle_duration_sec=2.0)
            return

        self.frame_cnt += 1
        now = self.get_clock().now().to_msg()

        # ── Publish Compressed ──────────────────────────────────
        msg_comp = CompressedImage()
        msg_comp.header.stamp = now
        msg_comp.header.frame_id = "camera_side_link"
        msg_comp.format = "jpeg"
        
        success, encode_image = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if success:
            msg_comp.data = encode_image.tobytes()
            self.pub_compressed.publish(msg_comp)

        # ── Publish Raw (Optional) ──────────────────────────────
        if self.pub_raw:
            msg_raw = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg_raw.header.stamp = now
            msg_raw.header.frame_id = "camera_side_link"
            self.pub_image.publish(msg_raw)

        # ── X11 Preview ─────────────────────────────────────────
        if self.use_x11:
            cv2.imshow("Side Camera Debug", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Closing debug window...")
                self.use_x11 = False
                cv2.destroyAllWindows()

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraSidePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()