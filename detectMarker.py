import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco
from maestro import Controller
import time
import threading
import queue

# ===================== Maestro Ports =====================
HeadVertPORT = 4
HeadHorPORT = 3
MPORT = 0
MPORT2 = 1

# ===================== Camera Calibration =====================
camera_matrix = np.load('cameraMatrix.npy')
dist_coeffs = np.load('distCoeffs.npy')

# ===================== ArUco Setup =====================
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# ===================== RealSense Setup =====================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# ===================== Shared Data & Controller =====================
shared_data = {
    "frame": None,
    "depth": None,
    "corners": [],
    "ids": [],
    "lock": threading.Lock()
}
seen_ids = set()

# Maestro Controller and Motor Command Queue
motor_controller = Controller()
motor_queue = queue.Queue()

def queue_motor_command(port, value):
    motor_queue.put((port, value))

# ===================== Threads =====================

def camera_thread():
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        undistorted = cv2.undistort(color_image, camera_matrix, dist_coeffs)

        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        output = cv2.aruco.drawDetectedMarkers(undistorted.copy(), corners, ids)

        with shared_data["lock"]:
            shared_data["frame"] = output
            shared_data["depth"] = depth_image
            shared_data["corners"] = corners
            shared_data["ids"] = ids

        cv2.imshow("ArUco Detection", output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def head_tracking_thread():
    servo_pos = {"hor": 6000, "vert": 6000}
    FRAME_CENTER_X, FRAME_CENTER_Y = 320, 240
    TOLERANCE, STEP = 20, 100

    while True:
        with shared_data["lock"]:
            corners = shared_data["corners"]
            ids = shared_data["ids"]

        if ids is not None and len(corners) > 0:
            cX = int(np.mean(corners[0][0][:, 0]))
            cY = int(np.mean(corners[0][0][:, 1]))

            if cX < FRAME_CENTER_X - TOLERANCE:
                servo_pos["hor"] += STEP
                servo_pos["hor"] = min(8000, servo_pos["hor"])
                queue_motor_command(HeadHorPORT, servo_pos["hor"])
            elif cX > FRAME_CENTER_X + TOLERANCE:
                servo_pos["hor"] -= STEP
                servo_pos["hor"] = max(4000, servo_pos["hor"])
                queue_motor_command(HeadHorPORT, servo_pos["hor"])

            if cY < FRAME_CENTER_Y - TOLERANCE:
                servo_pos["vert"] += STEP
                servo_pos["vert"] = min(8000, servo_pos["vert"])
                queue_motor_command(HeadVertPORT, servo_pos["vert"])
            elif cY > FRAME_CENTER_Y + TOLERANCE:
                servo_pos["vert"] -= STEP
                servo_pos["vert"] = max(4000, servo_pos["vert"])
                queue_motor_command(HeadVertPORT, servo_pos["vert"])

        time.sleep(0.05)

def movement_thread():
    global seen_ids
    while True:
        with shared_data["lock"]:
            corners = shared_data["corners"]
            ids = shared_data["ids"]

        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in seen_ids and len(seen_ids) < 4:
                    print(f"New Marker {marker_id} Detected.")
                    seen_ids.add(marker_id)

                    if marker_id % 2 == 1:
                        print("Arcing LEFT")
                        queue_motor_command(MPORT, 5500)
                        queue_motor_command(MPORT2, 7000)
                    else:
                        print("Arcing RIGHT")
                        queue_motor_command(MPORT, 7000)
                        queue_motor_command(MPORT2, 5500)

                    time.sleep(2.5)

                    # STOP
                    queue_motor_command(MPORT, 6000)
                    queue_motor_command(MPORT2, 6000)
                    print("Stopping...\n")
        if len(seen_ids) >= 4:
            print("Completed 4 markers. Stopping movement.")
            break
        time.sleep(0.1)

def motor_dispatcher_thread():
    while True:
        try:
            port, value = motor_queue.get(timeout=1)
            motor_controller.setTarget(port, value)
        except queue.Empty:
            continue
        except Exception as e:
            print(f"Motor dispatch error: {e}")

# ===================== Start Threads =====================
threads = [
    threading.Thread(target=camera_thread, daemon=True),
    threading.Thread(target=head_tracking_thread, daemon=True),
    threading.Thread(target=movement_thread),
    threading.Thread(target=motor_dispatcher_thread, daemon=True)
]

for t in threads:
    t.start()

threads[2].join()  # Wait only for movement thread to finish 4 markers
pipeline.stop()
cv2.destroyAllWindows()
print("Program complete.")
