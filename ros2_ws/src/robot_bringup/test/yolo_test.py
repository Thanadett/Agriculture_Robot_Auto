#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # ── YOLO model ─────────────────────────
        model_path = "/home/t/392_project/ros2_ws/best.pt"
        self.model = YOLO(model_path)

        self.conf_thres = 0.30
        self.frame_cnt = 0

        # ── Subscribe camera topic ─────────────
        self.sub = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self.image_callback,
            10
        )

        cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLO Detection", 800, 600)

        self.get_logger().info("YOLO detector started")


    def image_callback(self, msg):

        # ── Decode compressed image ───────────
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        self.frame_cnt += 1

        # ── Run YOLO ──────────────────────────
        results = self.model(frame, conf=self.conf_thres, verbose=False)

        vis = frame.copy()
        detected = 0

        for r in results:
            for box in r.boxes:

                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                name   = self.model.names.get(cls_id, str(cls_id))

                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]

                detected += 1

                # draw bounding box
                cv2.rectangle(vis, (x1,y1), (x2,y2), (0,255,0), 2)

                label = f"{name} {conf:.2f}"

                cv2.putText(
                    vis,
                    label,
                    (x1, y1-8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0,255,0),
                    2
                )

                print(
                    f"frame={self.frame_cnt} "
                    f"class={name} "
                    f"conf={conf:.3f} "
                    f"bbox=({x1},{y1},{x2},{y2})"
                )

        # ── HUD ───────────────────────────────
        color = (0,255,0) if detected>0 else (0,0,255)

        cv2.putText(
            vis,
            f"frame={self.frame_cnt} detected={detected}",
            (6,22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2
        )

        cv2.imshow("YOLO Detection", vis)

        if cv2.waitKey(1) & 0xFF in (ord('q'),27):
            rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = YoloDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()