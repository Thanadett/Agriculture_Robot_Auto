#!/home/prukubt/yolo_env/bin/python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from pupil_apriltags import Detector


class AprilTagCameraPublisher(Node):

    def __init__(self):
        super().__init__('apriltag_camera')

        # ---------------- Camera calibration ----------------
        self.fx = 651.50492
        self.fy = 650.39078
        self.cx = 320.62708
        self.cy = 236.91812
        self.tag_size = 0.04

        # ---------------- Publisher ----------------
        self.pub = self.create_publisher(Image, '/vision/image', 1)
        self.bridge = CvBridge()

        # ---------------- Camera ----------------
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            exit()

        # ---------------- AprilTag Detector ----------------
        self.detector = Detector(
            families="tagStandard52h13",
            nthreads=1,
            quad_decimate=2.0
        )

        # Timer ~10 FPS
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("AprilTag Camera Publisher started")


    def loop(self):

        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(self.fx, self.fy, self.cx, self.cy),
            tag_size=self.tag_size
        )

        for tag in tags:
            tx, ty, tz = tag.pose_t.flatten()

            self.get_logger().info(
                f"ID={tag.tag_id}  Z={tz:.3f}m"
            )

            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(frame,
                         tuple(corners[i]),
                         tuple(corners[(i+1)%4]),
                         (0,255,0), 2)

            cv2.putText(frame,
                        f"ID {tag.tag_id} Z={tz:.2f}m",
                        (corners[0][0], corners[0][1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0,255,0),
                        2)

        # Publish image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()