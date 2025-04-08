import pyrealsense2 as rs
import numpy as np
import cv2

# Checkerboard settings
CHECKERBOARD = (9, 6)
objpoints = []  # 3D points
imgpoints = []  # 2D points

# Real world grid setup
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# RealSense pipeline setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

print("Press 'q' to stop or wait until enough frames are captured.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            print(f"Checkerboard detected. Capturing frame #{len(imgpoints)+1}")
            objpoints.append(objp)
            imgpoints.append(corners)
            cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)

        cv2.imshow('Calibration', frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or len(imgpoints) >= 20:
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()

# Run calibration
print("Running calibration...")
ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\nCalibration Complete")
print("Camera Matrix:\n", cameraMatrix)
print("Distortion Coefficients:\n", distCoeffs)
