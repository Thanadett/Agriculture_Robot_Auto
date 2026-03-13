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

        # ---------------- Camera calibration ----------------
        self.fx = 651.50492
        self.fy = 650.39078
        self.cx = 320.62708
        self.cy = 236.91812
        self.tag_size = 0.04

        # ---------------- AprilTag Detector ----------------
        self.detector = Detector(
            families="tagStandard52h13",
            nthreads=1,
            quad_decimate=1.0,
            refine_edges=True
        )

        # ---------------- Subscriber ----------------
        self.sub = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self.image_callback,
            10
        )

        self.get_logger().info("AprilTag X11 Debug started (CompressedImage)")


    def image_callback(self, msg):

        # ---------------- Decode compressed image ----------------
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ---------------- Detect AprilTag ----------------
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(self.fx, self.fy, self.cx, self.cy),
            tag_size=self.tag_size
        )

        for tag in tags:

            tx, ty, tz = tag.pose_t.flatten()
            R = tag.pose_R

            # ===============================
            # 1️⃣ Parallel angle
            # ===============================

            normal = R[:, 2]
            camera_forward = np.array([0, 0, 1])

            cos_angle = np.dot(normal, camera_forward)
            cos_angle = np.clip(cos_angle, -1.0, 1.0)

            parallel_error = np.degrees(np.arccos(cos_angle))

            # ===============================
            # 2️⃣ Lateral error
            # ===============================

            lateral_error = tx
            distance = tz

            self.get_logger().info(
                f"ID={tag.tag_id} "
                f"Z={distance:.3f}m "
                f"tx={lateral_error:.3f}m "
                f"parallel_err={parallel_error:.2f}°"
            )

            # ---------------- Draw box ----------------
            corners = tag.corners.astype(int)

            for i in range(4):
                cv2.line(
                    frame,
                    tuple(corners[i]),
                    tuple(corners[(i+1)%4]),
                    (0,255,0), 2
                )

            # ---------------- Draw info ----------------
            cv2.putText(
                frame,
                f"Z={distance:.2f}m",
                (corners[0][0], corners[0][1]-40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0,255,0), 2
            )

            cv2.putText(
                frame,
                f"tx={lateral_error:.2f}m",
                (corners[0][0], corners[0][1]-20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0,255,0), 2
            )

            cv2.putText(
                frame,
                f"parallel={parallel_error:.1f}deg",
                (corners[0][0], corners[0][1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0,255,0), 2
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


if __name__ == '__main__':
    main()