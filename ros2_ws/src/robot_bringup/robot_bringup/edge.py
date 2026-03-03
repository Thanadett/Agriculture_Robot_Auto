#!/home/prukubt/yolo_env/bin/python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class PotEdgeNode(Node):

    def __init__(self):
        super().__init__('pot_edge_node')

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.image_pub = self.create_publisher(Image, '/vision/image', 1)

        self.timer = self.create_timer(1.0/15.0, self.timer_callback)

        self.get_logger().info("Pot Edge Node Started")


    def timer_callback(self):

        # flush old frames
        for _ in range(2):
            self.cap.grab()

        ret, frame = self.cap.read()
        if not ret:
            return

        debug_frame = frame.copy()

        left_line, right_line = self.detect_edges(frame)

        if left_line is not None:
            x1,y1,x2,y2 = left_line
            cv2.line(debug_frame,(x1,y1),(x2,y2),(0,255,0),3)

        if right_line is not None:
            x1,y1,x2,y2 = right_line
            cv2.line(debug_frame,(x1,y1),(x2,y2),(0,0,255),3)

        # ===== X11 Debug Window =====
        cv2.imshow("Pot Edge Debug", debug_frame)
        cv2.waitKey(1)

        # ===== ROS Publish (ถ้าต้องการ) =====
        msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
        self.image_pub.publish(msg)


    def detect_edges(self, frame):

        frame = cv2.resize(frame,(320,240))
        h, w = frame.shape[:2]

        # ===== ROI ครึ่งล่าง =====
        roi = frame[int(h*0.4):h, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # เพิ่ม contrast (ช่วยเส้นขวาเยอะมาก)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)

        blur = cv2.GaussianBlur(gray,(5,5),0)

        edges = cv2.Canny(blur, 80, 200)

        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi/180,
            threshold=60,
            minLineLength=90,
            maxLineGap=25
        )

        if lines is None:
            return None, None

        left_candidates = []
        right_candidates = []

        for line in lines:
            x1,y1,x2,y2 = line[0]

            y1_full = y1 + int(h*0.4)
            y2_full = y2 + int(h*0.4)

            angle = math.degrees(math.atan2((y2-y1),(x2-x1)))

            # รองรับกล้องเอียง
            if abs(angle) > 45:

                mid_x = (x1 + x2) / 2

                if mid_x < w/2:
                    left_candidates.append((x1,y1_full,x2,y2_full))
                else:
                    right_candidates.append((x1,y1_full,x2,y2_full))

        def longest(lines):
            return max(lines, key=lambda l: np.hypot(l[2]-l[0], l[3]-l[1]))

        left_line = longest(left_candidates) if left_candidates else None
        right_line = longest(right_candidates) if right_candidates else None

        return left_line, right_line


def main(args=None):
    rclpy.init(args=args)
    node = PotEdgeNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()