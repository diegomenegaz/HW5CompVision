import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco
from maestro import Controller
import time
import threading

# Maestro Ports
HeadVertPORT = 4
HeadHorPORT = 3
MPORT = 0
MPORT2 = 1
FORWARD = 8000
STOP = 6000

# Load camera calibration data
camera_matrix = np.load('cameraMatrix.npy')
dist_coeffs = np.load('distCoeffs.npy')

# ArUco Setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# RealSense Pipeline Setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Shared data between threads
shared_data = {
    "frame": None,
    "depth": None,
    "corners": [],
    "ids": [],
    "lock": threading.Lock()
}

# Controller Setup
motor_controller = Controller()


# Add initial servo positions for head tracking
servo_pos = {
    "hor": 6000,
    "vert": 6000
}

def head_tracking_thread():
    FRAME_CENTER_X = 320  # Assuming 640x480 resolution
    FRAME_CENTER_Y = 240
    TOLERANCE = 20         # Pixels
    STEP = 100             # Maestro servo step size

    while True:
        with shared_data["lock"]:
            corners = shared_data["corners"]
            ids = shared_data["ids"]

        if ids is not None and len(corners) > 0:
            for corner in corners:
                cX = int(np.mean(corner[0][:, 0]))
                cY = int(np.mean(corner[0][:, 1]))

                # Adjust horizontal head servo
                if cX < FRAME_CENTER_X - TOLERANCE:
                    servo_pos["hor"] += STEP
                    servo_pos["hor"] = min(8000, servo_pos["hor"])
                    motor_controller.setTarget(HeadHorPORT, servo_pos["hor"])
                elif cX > FRAME_CENTER_X + TOLERANCE:
                    servo_pos["hor"] -= STEP
                    servo_pos["hor"] = max(4000, servo_pos["hor"])
                    motor_controller.setTarget(HeadHorPORT, servo_pos["hor"])

                # Adjust vertical head servo
                if cY < FRAME_CENTER_Y - TOLERANCE:
                    servo_pos["vert"] += STEP
                    servo_pos["vert"] = min(8000, servo_pos["vert"])
                    motor_controller.setTarget(HeadVertPORT, servo_pos["vert"])
                elif cY > FRAME_CENTER_Y + TOLERANCE:
                    servo_pos["vert"] -= STEP
                    servo_pos["vert"] = max(4000, servo_pos["vert"])
                    motor_controller.setTarget(HeadVertPORT, servo_pos["vert"])

        time.sleep(0.05)


# Thread: Capture & Detect
def camera_thread():
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        undistorted_image = cv2.undistort(color_image, camera_matrix, dist_coeffs)

        gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        output = cv2.aruco.drawDetectedMarkers(undistorted_image.copy(), corners, ids)

        with shared_data["lock"]:
            shared_data["frame"] = output
            shared_data["depth"] = depth_image
            shared_data["corners"] = corners
            shared_data["ids"] = ids

        cv2.imshow("ArUco Marker Detection", output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Thread: Decision Making
def movement_thread():
    count = 0
    while True:
        with shared_data["lock"]:
            corners = shared_data["corners"]
            ids = shared_data["ids"]
            depth_image = shared_data["depth"]

        if ids is not None and len(corners) > 0:
            for i, corner in enumerate(corners):
                cX = int(np.mean(corner[0][:, 0]))
                cY = int(np.mean(corner[0][:, 1]))
                depth = depth_image[cY, cX] if depth_image is not None else 0
                marker_pos = (cX, cY, depth * 0.001)
                print(f"Marker {ids[i][0]} position: {marker_pos}")

                if ids[i][0] % 2 == 1:
                    print("Arcing Left")
                    motor_controller.setTarget(MPORT, 5500)
                    motor_controller.setTarget(MPORT2, 7000)
                else:
                    print("Arcing Right")
                    motor_controller.setTarget(MPORT2, 5500)
                    motor_controller.setTarget(MPORT, 7000)

                time.sleep(2)
                motor_controller.setTarget(MPORT, 6000)
                motor_controller.setTarget(MPORT2, 6000)
        time.sleep(0.1)

# Start threads
cam_thread = threading.Thread(target=camera_thread, daemon=True)
move_thread = threading.Thread(target=movement_thread, daemon=True)
head_thread = threading.Thread(target=head_tracking_thread, daemon=True)

cam_thread.start()
head_thread.start()
move_thread.start()

cam_thread.join()
pipeline.stop()
cv2.destroyAllWindows()
