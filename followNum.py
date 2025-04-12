import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco
import maestro
import time

# Load camera matrix and distortion coefficients
camera_matrix = np.load('cameraMatrix.npy')
dist_coeffs = np.load('distCoeffs.npy')

# Define ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# Start RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Maestro setup (adjust port if needed)
maestro_controller = maestro.Controller("/dev/ttyACM0")

# Servo channels (check Maestro Control Center for correct IDs)
PAN_SERVO_ID = 4
TILT_SERVO_ID = 3

# Center positions and current tracking state
PAN_CENTER = 6000
TILT_CENTER = 6000
current_pan = PAN_CENTER
current_tilt = TILT_CENTER

# Dead zone in pixels to prevent jitter
DEAD_ZONE = 5 # Reduced dead zone to make movement more sensitive

# Movement scale — how aggressively to move based on error
PAN_SCALE = 50  # Increased scale for more responsive pan movement
TILT_SCALE = 50  # Increased scale for more responsive tilt movement

# Smoothing factor
smooth_factor = 0.1
last_error_x = 0
last_error_y = 0

# === TEST MOVEMENT TO VALIDATE SERVO CONNECTION ===
print("[TEST] Moving PAN servo...")
maestro_controller.setTarget(PAN_SERVO_ID, 6000)
time.sleep(1)
maestro_controller.setTarget(PAN_SERVO_ID, 7000)
time.sleep(1)
maestro_controller.setTarget(PAN_SERVO_ID, 6000)
time.sleep(1)

print("[TEST] Moving TILT servo...")
maestro_controller.setTarget(TILT_SERVO_ID, 6000)
time.sleep(1)
maestro_controller.setTarget(TILT_SERVO_ID, 5000)
time.sleep(1)
maestro_controller.setTarget(TILT_SERVO_ID, 6000)
time.sleep(1)

# === Marker detection ===
def detect_aruco_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if len(corners) > 0:
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    return frame, corners, ids

# Get marker center
def get_marker_center(corner):
    cX = int(np.mean(corner[0][:, 0]))
    cY = int(np.mean(corner[0][:, 1]))
    return cX, cY

# === MAIN LOOP ===
try:
    while True:
        # Get RealSense frames
        frames = pipeline.wait_for_frames(timeout_ms=10000)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            print("Skipping frame due to missing data.")
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Undistort camera image
        undistorted_image = cv2.undistort(color_image, camera_matrix, dist_coeffs)

        # Detect markers
        frame, corners, ids = detect_aruco_markers(undistorted_image)

        if len(corners) > 0:
            for corner in corners:
                cX, cY = get_marker_center(corner)

                # Error from center
                error_x = cX - (color_image.shape[1] // 2)
                error_y = cY - (color_image.shape[0] // 2)

                # Smooth error
                error_x = smooth_factor * error_x + (1 - smooth_factor) * last_error_x
                error_y = smooth_factor * error_y + (1 - smooth_factor) * last_error_y
                last_error_x = error_x
                last_error_y = error_y

                print(f"Error in X: {error_x:.2f}, Error in Y: {error_y:.2f}")

                # === PAN MOVEMENT ===
                if abs(error_x) > DEAD_ZONE:
                    delta_pan = -int(error_x * PAN_SCALE / color_image.shape[1])
                    current_pan += delta_pan
                    current_pan = max(4000, min(8000, current_pan))
                    print(f"[PAN] Setting target to: {current_pan}")
                    maestro_controller.setTarget(PAN_SERVO_ID, current_pan)
                else:
                    print("[PAN] Within dead zone, no move.")

                # === TILT MOVEMENT ===
                if abs(error_y) > DEAD_ZONE:
                    delta_tilt = -int(error_y * TILT_SCALE / color_image.shape[0])
                    current_tilt += delta_tilt
                    current_tilt = max(4000, min(8000, current_tilt))
                    print(f"[TILT] Setting target to: {current_tilt}")
                    maestro_controller.setTarget(TILT_SERVO_ID, current_tilt)
                else:
                    print("[TILT] Within dead zone, no move.")

        # Display result
        cv2.imshow("ArUco Marker Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("Stopping pipeline and closing windows.")
    pipeline.stop()
    cv2.destroyAllWindows()
