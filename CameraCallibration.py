import pyrealsense2 as rs
import numpy as np
import cv2

# Checkerboard configuration
CHECKERBOARD = (9, 6)
objpoints = []
imgpoints = []

# Real-world grid points
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Start RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

print("Press 'c' to capture a frame (when checkerboard is detected). Press 'q' to quit and calibrate.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if frame is None:
            print("Warning: Frame is None")
            continue
        print("Godamnit")
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        print("Line 35")
        display_frame = frame.copy()
        if ret:
            cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret)
            cv2.putText(display_frame, "Checkerboard detected! Press 'c' to capture.", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        cv2.imshow("Calibration (press 'c' to capture)", display_frame)
        key = cv2.waitKey(1)

        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('c') and ret:
            print(f"Captured frame #{len(imgpoints)+1}")
            objpoints.append(objp)
            imgpoints.append(corners)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

if len(objpoints) < 5:
    print("Not enough frames captured for calibration. Capture at least 5.")
    exit()

# Run calibration
print("Calibrating camera...")
ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\nCalibration Complete")
print("Camera Matrix:\n", cameraMatrix)
print("Distortion Coefficients:\n", distCoeffs)

# Save calibration
np.save("cameraMatrix.npy", cameraMatrix)
np.save("distCoeffs.npy", distCoeffs)
