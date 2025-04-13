import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco
import threading
import queue
import time

from testMovement import MovementControl

# Get shared movement controller
movement = MovementControl.getInst()

# Shared memory for camera detection
shared_data = {
    "frame": None,
    "depth": None,
    "corners": [],
    "ids": [],
    "lock": threading.Lock()
}

seen_ids = set()
motor_queue = queue.Queue()

# === Queue Motor Command ===
def queue_motor_command(func, *args):
    motor_queue.put((func, args))

# === Motor Dispatcher Thread ===
def motor_dispatcher_thread():
    while True:
        try:
            func, args = motor_queue.get(timeout=1)
            print(f"[Motor] Executing {func.__name__} with {args}")
            func(*args)
        except queue.Empty:
            continue
        except Exception as e:
            print(f"[Motor Error] {e}")

# === Camera Detection Thread ===
def camera_thread():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    camera_matrix = np.load('cameraMatrix.npy')
    dist_coeffs = np.load('distCoeffs.npy')
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()

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

    pipeline.stop()
    cv2.destroyAllWindows()

# === Head Tracking Thread ===
def head_tracking_thread():
    hor = 6000
    vert = 6000
    FRAME_CENTER_X = 320
    FRAME_CENTER_Y = 240
    TOLERANCE = 20
    STEP = 100

    while True:
        with shared_data["lock"]:
            corners = shared_data["corners"]
            ids = shared_data["ids"]

        if ids is not None and len(corners) > 0:
            cX = int(np.mean(corners[0][0][:, 0]))
            cY = int(np.mean(corners[0][0][:, 1]))

            if abs(cX - FRAME_CENTER_X) > TOLERANCE:
                if cX < FRAME_CENTER_X:
                    hor = min(8000, hor + STEP)
                else:
                    hor = max(4000, hor - STEP)
                queue_motor_command(movement.pan, hor)

            if abs(cY - FRAME_CENTER_Y) > TOLERANCE:
                if cY < FRAME_CENTER_Y:
                    vert = min(8000, vert + STEP)
                else:
                    vert = max(4000, vert - STEP)
                queue_motor_command(movement.tilt, vert)

        time.sleep(0.05)

# === Movement Thread (4 unique markers) ===
def movement_thread():
    while len(seen_ids) < 4:
        with shared_data["lock"]:
            corners = shared_data["corners"]
            ids = shared_data["ids"]

        if ids is not None and len(ids) > 0:
            for marker_id in ids.flatten():
                if marker_id not in seen_ids:
                    seen_ids.add(marker_id)
                    print(f"[Marker] Detected new ID: {marker_id}")

                    if marker_id % 2 == 1:
                        print("[Action] Arc LEFT")
                        queue_motor_command(movement.arc_L)
                    else:
                        print("[Action] Arc RIGHT")
                        queue_motor_command(movement.arc_R)

                    time.sleep(2)
                    queue_motor_command(movement.stop)

        time.sleep(0.1)

    print("✅ Completed 4 markers. Shutting down movement thread.")

# === Start Threads ===
threads = [
    threading.Thread(target=camera_thread, daemon=True),
    threading.Thread(target=head_tracking_thread, daemon=True),
    threading.Thread(target=movement_thread),
    threading.Thread(target=motor_dispatcher_thread, daemon=True)
]

for t in threads:
    t.start()

threads[2].join()  # Wait for movement thread to finish
print("🎉 All done.")
