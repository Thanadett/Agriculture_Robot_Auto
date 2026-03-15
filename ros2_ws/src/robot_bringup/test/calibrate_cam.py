#!/usr/bin/env python3

import cv2
import numpy as np
import yaml

# ==============================
# SETTINGS
# ==============================

CHESSBOARD_SIZE = (7, 5)   # inner corners
SQUARE_SIZE = 0.025        # meter (2.5 cm)

CAMERA_ID = 6              # /dev/video0

# ==============================
# Prepare object points
# ==============================

objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[
    0:CHESSBOARD_SIZE[0],
    0:CHESSBOARD_SIZE[1]
].T.reshape(-1, 2)

objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

# ==============================
# Start camera
# ==============================

cap = cv2.VideoCapture(CAMERA_ID)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

print("\nControls:")
print("  c = capture frame")
print("  q = finish calibration\n")

criteria = (
    cv2.TERM_CRITERIA_EPS +
    cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001
)

while True:

    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(
        gray,
        CHESSBOARD_SIZE,
        None
    )

    if found:

        corners2 = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria
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
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0,255,0),
            2
        )

    else:

        cv2.putText(
            frame,
            "No chessboard",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0,0,255),
            2
        )

    cv2.imshow("Calibration", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):

        if found:

            objpoints.append(objp)
            imgpoints.append(corners2)

            print(f"Captured frame {len(objpoints)}")

        else:

            print("Chessboard NOT detected")

    elif key == ord('q'):

        break

cap.release()
cv2.destroyAllWindows()

# ==============================
# Check captured images
# ==============================

if len(objpoints) < 5:

    print("\nNot enough frames captured")
    print("Need at least 5-10 images")
    exit()

# ==============================
# Calibration
# ==============================

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    gray.shape[::-1],
    None,
    None
)

print("\nCalibration Done\n")

print("Camera Matrix:\n")
print(camera_matrix)

print("\nDistortion Coefficients:\n")
print(dist_coeffs)

# ==============================
# Reprojection error
# ==============================

mean_error = 0

for i in range(len(objpoints)):

    imgpoints2, _ = cv2.projectPoints(
        objpoints[i],
        rvecs[i],
        tvecs[i],
        camera_matrix,
        dist_coeffs
    )

    error = cv2.norm(
        imgpoints[i],
        imgpoints2,
        cv2.NORM_L2
    ) / len(imgpoints2)

    mean_error += error

print("\nReprojection error:", mean_error / len(objpoints))

# ==============================
# Save YAML
# ==============================

data = {

    "camera_matrix": camera_matrix.tolist(),
    "dist_coeff": dist_coeffs.tolist()

}

with open("camera_calibration.yaml", "w") as f:

    yaml.dump(data, f)

print("\nSaved camera_calibration.yaml")