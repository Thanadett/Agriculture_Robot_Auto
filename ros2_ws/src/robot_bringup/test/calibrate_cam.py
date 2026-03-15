#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import yaml

from sensor_msgs.msg import CompressedImage


# ==============================
# SETTINGS
# ==============================

CHESSBOARD_SIZE = (7, 5)
SQUARE_SIZE = 0.03   # meter

TOPIC = "/camera/image_raw/compressed"


# ==============================
# Prepare object points
# ==============================

objp = np.zeros((CHESSBOARD_SIZE[0]*CHESSBOARD_SIZE[1],3), np.float32)
objp[:,:2] = np.mgrid[
    0:CHESSBOARD_SIZE[0],
    0:CHESSBOARD_SIZE[1]
].T.reshape(-1,2)

objp *= SQUARE_SIZE


class CameraCalibrator(Node):

    def __init__(self):

        super().__init__('camera_calibrator')

        self.objpoints = []
        self.imgpoints = []

        self.gray = None

        self.sub = self.create_subscription(
            CompressedImage,
            TOPIC,
            self.image_callback,
            10
        )

        self.criteria = (
            cv2.TERM_CRITERIA_EPS +
            cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001
        )

        cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)

        print("\nControls:")
        print("  c = capture frame")
        print("  q = finish calibration\n")


    def image_callback(self, msg):

        # decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.gray = gray

        found, corners = cv2.findChessboardCorners(
            gray,
            CHESSBOARD_SIZE,
            None
        )

        if found:

            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (11,11),
                (-1,-1),
                self.criteria
            )

            cv2.drawChessboardCorners(
                frame,
                CHESSBOARD_SIZE,
                corners2,
                found
            )

            cv2.putText(
                frame,
                "Chessboard detected",
                (20,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0,255,0),
                2
            )

            self.last_corners = corners2

        else:

            cv2.putText(
                frame,
                "No chessboard",
                (20,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0,0,255),
                2
            )

            self.last_corners = None

        cv2.imshow("Calibration", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):

            if found:

                self.objpoints.append(objp)
                self.imgpoints.append(self.last_corners)

                print(f"Captured frame {len(self.objpoints)}")

            else:

                print("Chessboard NOT detected")

        elif key == ord('q'):

            self.finish_calibration()


    def finish_calibration(self):

        if len(self.objpoints) < 5:

            print("\nNot enough frames captured")
            print("Need at least 5-10 images")
            rclpy.shutdown()
            return

        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints,
            self.imgpoints,
            self.gray.shape[::-1],
            None,
            None
        )

        print("\nCalibration Done\n")

        print("Camera Matrix:\n")
        print(camera_matrix)

        print("\nDistortion Coefficients:\n")
        print(dist_coeffs)

        # reprojection error
        mean_error = 0

        for i in range(len(self.objpoints)):

            imgpoints2, _ = cv2.projectPoints(
                self.objpoints[i],
                rvecs[i],
                tvecs[i],
                camera_matrix,
                dist_coeffs
            )

            error = cv2.norm(
                self.imgpoints[i],
                imgpoints2,
                cv2.NORM_L2
            ) / len(imgpoints2)

            mean_error += error

        print("\nReprojection error:", mean_error / len(self.objpoints))

        data = {

            "camera_matrix": camera_matrix.tolist(),
            "dist_coeff": dist_coeffs.tolist()

        }

        with open("camera_calibration.yaml","w") as f:
            yaml.dump(data, f)

        print("\nSaved camera_calibration.yaml")

        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = CameraCalibrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()