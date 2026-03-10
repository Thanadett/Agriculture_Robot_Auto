#!/usr/bin/env python3
"""
Pi Camera Publisher — ROS2
============================================================
รับภาพจากกล้องบน Raspberry Pi แล้ว publish ผ่าน ROS2 topic

Topics published:
  /camera/image_raw/compressed   (sensor_msgs/CompressedImage)
  /camera/image_raw              (sensor_msgs/Image)  [optional, ปิดได้เพื่อลด bandwidth]

Parameter:
  camera_id       : int   = 0
  image_width     : int   = 640
  image_height    : int   = 480
  fps             : int   = 30
  jpeg_quality    : int   = 80   (0-100, ยิ่งสูงคุณภาพดีแต่ bandwidth สูง)
  publish_raw     : bool  = False (publish raw image ด้วยหรือเปล่า, ใช้ bandwidth สูงมาก)
  use_x11_debug   : bool  = True  (แสดงหน้าต่าง preview บน Pi)

Note: แนะนำให้ใช้ compressed เท่านั้น เพื่อประหยัด bandwidth WiFi
      640×480 @ JPEG80 ≈ 15-30 KB/frame ≈ 3-7 MB/s ที่ 30fps
      ถ้า WiFi ไม่แรงพอ ลด fps หรือ jpeg_quality ลง
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from cv_bridge import CvBridge

import cv2
import numpy as np

#v4l2-ctl --list-devices find which one is the pi camera, then set camera_id param accordingly (default=0)
"""
ถ้าใช้กล้อง USB หลายตัวบน Pi อาจเจอปัญหา camera index สลับไปมาเมื่อรีบูตเครื่อง แนะนำให้ตั้ง udev rule ให้กล้องแต่ละตัวมี symlink ชื่อคงที่ 
sudo nano /etc/udev/rules.d/99-usb-cameras.rules
SUBSYSTEM=="video4linux", ATTRS{serial}=="20250603", ATTR{index}=="0", SYMLINK+="webcam_EYD_2k"
SUBSYSTEM=="video4linux", ATTRS{serial}=="241015140801", ATTR{index}=="0", SYMLINK+="webcam_EYD_1080p"
"""

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('pi_camera_publisher')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('camera_id',     '/dev/webcam_EYD_2k')
        self.declare_parameter('image_width',   640)
        self.declare_parameter('image_height',  480)
        self.declare_parameter('fps',           30)
        self.declare_parameter('jpeg_quality',  80)
        self.declare_parameter('publish_raw',   False)
        self.declare_parameter('use_x11_debug', False)

        self.cam_id      = self.get_parameter('camera_id').value
        self.W           = self.get_parameter('image_width').value
        self.H           = self.get_parameter('image_height').value
        self.fps         = self.get_parameter('fps').value
        self.jpeg_quality= self.get_parameter('jpeg_quality').value
        self.pub_raw     = self.get_parameter('publish_raw').value
        self.use_x11     = self.get_parameter('use_x11_debug').value

        # ── Publishers ──────────────────────────────────────────
        self.pub_compressed = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', 5)

        if self.pub_raw:
            self.pub_image = self.create_publisher(
                Image, '/camera/image_raw', 5)

        # ── Camera ──────────────────────────────────────────────
        self.cap = cv2.VideoCapture(self.cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.bridge = CvBridge()
        self.frame_cnt = 0

        # ── X11 debug window ────────────────────────────────────
        if self.use_x11:
            cv2.namedWindow("Pi Camera", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Pi Camera", 480, 360)

        # ── Timer ────────────────────────────────────────────────
        interval = 1.0 / self.fps
        self.timer = self.create_timer(interval, self.loop)

        self.get_logger().info(
            f"Pi Camera Publisher | {self.W}×{self.H} @ {self.fps}fps"
            f" | JPEG quality={self.jpeg_quality}"
            f" | raw={'ON' if self.pub_raw else 'OFF'}")

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera read failed", throttle_duration_sec=1.0)
            return

        self.frame_cnt += 1
        now = self.get_clock().now().to_msg()

        # ── Publish compressed ───────────────────────────────────
        encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        ret2, buf = cv2.imencode('.jpg', frame, encode_param)
        if ret2:
            msg = CompressedImage()
            msg.header.stamp    = now
            msg.header.frame_id = 'camera'
            msg.format          = 'jpeg'
            msg.data            = buf.tobytes()
            self.pub_compressed.publish(msg)

        # ── Publish raw (optional) ───────────────────────────────
        if self.pub_raw:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp    = now
            img_msg.header.frame_id = 'camera'
            self.pub_image.publish(img_msg)

        # ── X11 debug preview ────────────────────────────────────
        if self.use_x11:
            preview = cv2.resize(frame, (480, 360))
            cv2.putText(preview,
                f"frame={self.frame_cnt} JPEG={self.jpeg_quality}",
                (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
            cv2.imshow("Pi Camera", preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                raise KeyboardInterrupt

    def destroy_node(self):
        self.cap.release()
        if self.use_x11:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()