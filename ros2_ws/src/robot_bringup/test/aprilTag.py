#!/home/prukubt/yolo_env/bin/python

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from pupil_apriltags import Detector


class AprilTagCamera(Node):

    def __init__(self):
        super().__init__('apriltag_camera_debug')

        # ===== Calibration ใหม่ =====
        self.fx = 535.7020623190093
        self.fy = 536.2286757565952
        self.cx = 328.92740826965303
        self.cy = 244.82018265933337

        self.camera_matrix = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ])

        self.dist_coeff = np.array([
            0.2364165900471923,
            -0.6070316832395167,
            0.0018016917089935594,
            0.0076473276069728155,
            0.5044612186719518
        ])

        self.tag_size = 0.042

        # ===== Detector =====
        self.detector = Detector(
            families="tagStandard52h13",
            nthreads=2,
            quad_decimate=1.5,
            refine_edges=True
        )

        self.sub = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self.image_callback,
            10
        )

        self.get_logger().info("AprilTag Debug Started")


    def image_callback(self, msg):

        # decode image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        # undistort
        frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeff)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detect tag
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(self.fx, self.fy, self.cx, self.cy),
            tag_size=self.tag_size
        )

        for tag in tags:

            tx, ty, tz = tag.pose_t.flatten()

            R = tag.pose_R
            normal = R[:,2]
            camera_forward = np.array([0,0,1])

            cos_angle = np.dot(normal, camera_forward)
            cos_angle = np.clip(cos_angle,-1.0,1.0)

            parallel_error = np.degrees(np.arccos(cos_angle))

            corners = tag.corners.astype(int)

            # draw box
            for i in range(4):
                cv2.line(
                    frame,
                    tuple(corners[i]),
                    tuple(corners[(i+1)%4]),
                    (0,255,0),
                    2
                )

            cv2.putText(frame,
                f"Z={tz:.2f}m",
                (corners[0][0], corners[0][1]-40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,(0,255,0),2)

            cv2.putText(frame,
                f"tx={tx:.2f}m",
                (corners[0][0], corners[0][1]-20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,(0,255,0),2)

            cv2.putText(frame,
                f"parallel={parallel_error:.1f}",
                (corners[0][0], corners[0][1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,(0,255,0),2)

            print(
                f"ID={tag.tag_id}  "
                f"Z={tz:.3f}m  "
                f"tx={tx:.3f}m  "
                f"parallel={parallel_error:.2f}deg"
            )

        cv2.imshow("AprilTag Debug", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = AprilTagCamera()

    rclpy.spin(node)

    node.destroy_node()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()