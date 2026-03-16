#!/usr/bin/env python3

import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2
import numpy as np


# ──────────────────────────────────────────────────────────────
# Helper: ตั้งค่า exposure ผ่าน v4l2-ctl
# ──────────────────────────────────────────────────────────────
def set_manual_exposure(device: str, exposure_value: int, logger=None) -> bool:
    # ชื่อ control จาก v4l2-ctl --list-ctrls:
    #   auto_exposure=1           → Manual Mode  (default=3 คือ Aperture Priority)
    #   exposure_time_absolute    → shutter speed (range 2–1250)
    if not isinstance(device, str):
        if logger:
            logger.warn("set_manual_exposure: device ต้องเป็น string path เช่น /dev/video0")
        return False

    cmds = [
        ["v4l2-ctl", "-d", device, "--set-ctrl=auto_exposure=1"],
        ["v4l2-ctl", "-d", device, f"--set-ctrl=exposure_time_absolute={exposure_value}"],
    ]
    for cmd in cmds:
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
            if result.returncode != 0 and logger:
                logger.warn(f"v4l2-ctl: {result.stderr.strip()}")
        except FileNotFoundError:
            if logger:
                logger.warn("v4l2-ctl not found — ใช้ OpenCV fallback")
            return False
        except subprocess.TimeoutExpired:
            if logger:
                logger.warn("v4l2-ctl timeout")
            return False
    if logger:
        logger.info(f"Manual exposure set: exposure_time_absolute={exposure_value}")
    return True


# ──────────────────────────────────────────────────────────────
# CLAHE processor  (LAB color space → แก้แค่ความสว่าง รักษาสี)
# ──────────────────────────────────────────────────────────────
class CLAHEProcessor:
    def __init__(self, clip_limit: float = 2.0, tile_grid: tuple = (8, 8)):
        self.clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid)

    def apply(self, frame_bgr: np.ndarray) -> np.ndarray:
        lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        lab_eq = cv2.merge([self.clahe.apply(l), a, b])
        return cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)


# ──────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────
class CameraSidePublisher(Node):

    def __init__(self):
        super().__init__('cam_side_publisher')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('camera_id',       '/dev/webcam_EYD_1080p')
        self.declare_parameter('image_width',      640)
        self.declare_parameter('image_height',     480)
        self.declare_parameter('fps',              30)
        self.declare_parameter('jpeg_quality',     80)
        self.declare_parameter('publish_raw',      True)
        self.declare_parameter('use_x11_debug',    False)
        self.declare_parameter('manual_exposure',  True)
        self.declare_parameter('exposure_value',   200)    # ลองปรับ 2–1250
        self.declare_parameter('clahe_clip_limit', 2.0)
        self.declare_parameter('clahe_tile_w',     8)
        self.declare_parameter('clahe_tile_h',     8)

        self.cam_id_param = self.get_parameter('camera_id').value
        self.W            = self.get_parameter('image_width').value
        self.H            = self.get_parameter('image_height').value
        self.fps          = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.pub_raw      = self.get_parameter('publish_raw').value
        self.use_x11      = self.get_parameter('use_x11_debug').value
        self.manual_exp   = self.get_parameter('manual_exposure').value
        self.exp_value    = self.get_parameter('exposure_value').value
        clip              = self.get_parameter('clahe_clip_limit').value
        tile_w            = self.get_parameter('clahe_tile_w').value
        tile_h            = self.get_parameter('clahe_tile_h').value

        # แปลง cam_id เป็น int ถ้าเป็นตัวเลข
        self.cam_id = self._parse_camera_id(self.cam_id_param)

        # ── CLAHE ────────────────────────────────────────────────
        self.clahe_proc = CLAHEProcessor(clip_limit=clip, tile_grid=(tile_w, tile_h))

        # ── Publishers ──────────────────────────────────────────
        self.pub_compressed = self.create_publisher(
            CompressedImage, '/camera_side/image_raw/compressed', 10)

        if self.pub_raw:
            self.pub_image = self.create_publisher(
                Image, '/camera_side/image_raw', 10)

        # ── Camera ──────────────────────────────────────────────
        self.cap = cv2.VideoCapture(self.cam_id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        self.cap.set(cv2.CAP_PROP_AUTO_WB,      0)   # ปิด Auto White Balance → ป้องกันภาพกระพริบ

        # ── Manual Exposure ──────────────────────────────────────
        if self.manual_exp:
            ok = set_manual_exposure(
                self.cam_id if isinstance(self.cam_id, str) else self.cam_id_param,
                self.exp_value,
                self.get_logger()
            )
            if not ok:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
                self.cap.set(cv2.CAP_PROP_EXPOSURE, -6)
                self.get_logger().warn("ใช้ OpenCV CAP_PROP_EXPOSURE fallback")

        self.bridge    = CvBridge()
        self.frame_cnt = 0

        # ── X11 debug ────────────────────────────────────────────
        if self.use_x11:
            cv2.namedWindow("Side Camera (CLAHE)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Side Camera (CLAHE)", 480, 360)

        # ── Timer ────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        self.get_logger().info(
            f"Side Camera Publisher ready | {self.W}x{self.H} @ {self.fps}fps"
            f" | JPEG={self.jpeg_quality}"
            f" | exposure={'manual(' + str(self.exp_value) + ')' if self.manual_exp else 'auto'}"
            f" | CLAHE clip={clip} tile=({tile_w},{tile_h})"
            f" | raw={'ON' if self.pub_raw else 'OFF'}"
        )

    # ── Main loop ────────────────────────────────────────────────
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn(
                "Failed to read from Side Camera", throttle_duration_sec=2.0)
            return

        self.frame_cnt += 1
        now = self.get_clock().now().to_msg()

        # CLAHE ก่อน publish
        frame_out = self.clahe_proc.apply(frame)

        # ── Publish Compressed ───────────────────────────────────
        encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        success, buf = cv2.imencode('.jpg', frame_out, encode_param)
        if success:
            msg = CompressedImage()
            msg.header.stamp    = now
            msg.header.frame_id = 'camera_side_link'
            msg.format          = 'jpeg'
            msg.data            = buf.tobytes()
            self.pub_compressed.publish(msg)

        # ── Publish Raw (Optional) ───────────────────────────────
        if self.pub_raw:
            msg_raw = self.bridge.cv2_to_imgmsg(frame_out, encoding='bgr8')
            msg_raw.header.stamp    = now
            msg_raw.header.frame_id = 'camera_side_link'
            self.pub_image.publish(msg_raw)

        # ── X11 debug ────────────────────────────────────────────
        if self.use_x11:
            preview = cv2.resize(frame_out, (480, 360))
            cv2.putText(preview,
                f"frame={self.frame_cnt} exp={self.exp_value} CLAHE",
                (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
            cv2.imshow("Side Camera (CLAHE)", preview)
            if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                self.get_logger().info("Closing debug window...")
                self.use_x11 = False
                cv2.destroyAllWindows()

    # ── Helpers ──────────────────────────────────────────────────
    def _parse_camera_id(self, value):
        """แปลงค่าจาก parameter ให้เป็น int หรือ string ตามความเหมาะสม"""
        try:
            return int(value)
        except (ValueError, TypeError):
            return value

    # ── Cleanup ──────────────────────────────────────────────────
    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# ──────────────────────────────────────────────────────────────
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