import cv2
import numpy as np

# Define checkerboard size
CHECKERBOARD = (9, 6)

# Store 3D and 2D points
objpoints = []  # 3D real world points
imgpoints = []  # 2D image points

# Prepare grid for 3D points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

cap = cv2.VideoCapture(0)  # or your RealSense stream

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    
    if ret_corners:
        imgpoints.append(corners)
        objpoints.append(objp)
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret_corners)
        print("Corners detected. Collected:", len(imgpoints))

    cv2.imshow("Calibration", frame)
    if cv2.waitKey(1) & 0xFF == ord('q') or len(imgpoints) > 20:
        break

cap.release()
cv2.destroyAllWindows()

# Calibrate camera
ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save or print calibration
print("Camera Matrix:\n", cameraMatrix)
print("Distortion Coefficients:\n", distCoeffs)
