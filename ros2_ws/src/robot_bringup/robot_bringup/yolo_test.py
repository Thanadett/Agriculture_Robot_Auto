#!/home/prukubt/yolo_env/bin/python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time


class YoloVisionNode(Node):

    def __init__(self):
        super().__init__('yolo_vision_node')

        # โหลดโมเดล
        model_path = "/home/prukubt/392_Agri/ros2_ws/bestTop.pt"
        self.model = YOLO(model_path)

        # เปิดกล้อง (USB camera = 0)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.bridge = CvBridge()

        # Publisher
        self.image_pub = self.create_publisher(Image, '/vision/image', 1)

        # Timer loop 30Hz
        self.timer = self.create_timer(1.0/10.0, self.timer_callback)

        self.get_logger().info("YOLO Vision Node Started")

    def timer_callback(self):

        ret, frame = self.cap.read()
        if not ret:
            return

        # YOLO detect (ลด imgsz ถ้า Pi ช้า)
        results = self.model(frame, imgsz=480, conf=0.4, verbose=False)

        # วาด bbox เอง (เบากว่า plot())
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])

            label = f"{self.model.names[cls]} {conf:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, label, (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0,255,0), 2)

        # Publish ROS Image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()