#!/home/t/yolo_env/bin/python

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from pupil_apriltags import Detector


class AprilTagFollower(Node):

    def __init__(self):
        super().__init__('apriltag_follower')

        # =====================
        # Parameters
        # =====================
        self.declare_parameter('camera_id', 4)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('publish_image', True)

        self.declare_parameter('tag_size', 0.070)

        cam_id = self.get_parameter('camera_id').value
        self.W = self.get_parameter('image_width').value
        self.H = self.get_parameter('image_height').value
        self.publish_image_flag = self.get_parameter('publish_image').value
        self.TAG_SIZE = self.get_parameter('tag_size').value

        # =====================
        # Camera Calibration (approx)
        # =====================
        fx = 580
        fy = 580
        cx = self.W / 2.0
        cy = self.H / 2.0
        self.camera_params = (fx, fy, cx, cy)

        # =====================
        # Detector
        # =====================
        self.detector = Detector(
            families="tagStandard52h13",
            nthreads=4,
            quad_decimate=1.0,
            refine_edges=True
        )

        # =====================
        # ROS Publishers
        # =====================
        self.plant_pub = self.create_publisher(Int32, '/apriltag/planting_distance', 10)
        self.gap_pub = self.create_publisher(Int32, '/apriltag/gap_type', 10)
        self.interval_pub = self.create_publisher(Int32, '/apriltag/cabbage_interval', 10)
        self.image_pub = self.create_publisher(Image, '/vision/apriltag', 10)

        self.bridge = CvBridge()

        # =====================
        # Camera
        # =====================
        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            exit()

        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info("AprilTag follower started")

    # =====================================================
    # Decode ID → AB / C / DE
    # =====================================================
    def decode_id(self, tag_id):

        AB = tag_id // 1000
        C  = (tag_id // 100) % 10
        DE = tag_id % 100

        return AB, C, DE

    # =====================================================
    # MAIN LOOP
    # =====================================================
    def update(self):

        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.TAG_SIZE
        )

        for r in results:

            tag_id = r.tag_id
            AB, C, DE = self.decode_id(tag_id)

            # Publish decoded values
            plant_msg = Int32()
            plant_msg.data = AB
            self.plant_pub.publish(plant_msg)

            gap_msg = Int32()
            gap_msg.data = C
            self.gap_pub.publish(gap_msg)

            interval_msg = Int32()
            interval_msg.data = DE
            self.interval_pub.publish(interval_msg)

            self.get_logger().info(
                f"ID:{tag_id} → AB:{AB}cm | C:{C} | DE:{DE}cm"
            )

            # Draw bounding box (for debug image topic)
            corners = r.corners.astype(int)
            for i in range(4):
                cv2.line(frame,
                         tuple(corners[i]),
                         tuple(corners[(i+1)%4]),
                         (0,255,0), 2)

            cv2.putText(frame,
                        f"ID:{tag_id}",
                        (int(r.center[0]), int(r.center[1])),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0,0,255),
                        2)

        # =====================
        # Publish image topic
        # =====================
        if self.publish_image_flag:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(img_msg)


# =====================================================
# MAIN
# =====================================================
def main(args=None):

    rclpy.init(args=args)
    node = AprilTagFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
