import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco
from maestro import Controller
import time

#MovementController For Demo Includes Head and Wheels
HeadVertPORT = 4
HeadHorPORT = 3
MPORT = 0
MPORT2 = 1
FORWARD = 8000
STOP = 6000
"""
class MovementControl:
	_instance = None
	@staticmethod
	def getInst():
		if MovementControl._instance == None:
			MovementControl._instance = MovementControl()
		return MovementControl._instance
	def __init__(self):
		self.m = Controller()
		pass
# Wheel Based Movement

	def arc_L(self,duration=2):
		self.m.setTarget(MPORT, 5500)
		self.m.setTarget(MPORT2, 7000)
		time.sleep(duration)
	def arc_R(self,duration=2):
		self.m.setTarget(MPORT2, 5500)
		self.m.setTarget(MPORT,7000)
		time.sleep(duration)
	def breaks(self):
		self.m.setTarget(MPORT, 6000)
		self.m.setTarget(MPORT2, 6000)
"""

# Load camera matrix and distortion coefficients
camera_matrix = np.load('cameraMatrix.npy')
dist_coeffs = np.load('distCoeffs.npy')
#movement = MovementControl().getInst()
# Define ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # Correct way to get dictionary
aruco_params = cv2.aruco.DetectorParameters()  # Use DetectorParameters directly

# Start RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Create ArUco marker detection function
def detect_aruco_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if len(corners) > 0:
        # Draw detected markers
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    return frame, corners, ids

# Create RealSense pipeline to get depth data (for position tracking)
def get_depth_data():
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    # Ensure both color and depth frames are available
    if not color_frame or not depth_frame:
        print("Error: Missing color or depth frame.")
        return None, None

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    return color_image, depth_image

# Function to get the position of ArUco markers in 3D
def get_marker_3d_position(corner, depth_image):
    # Find the center of the marker
    cX = int(np.mean(corner[0][:, 0]))
    cY = int(np.mean(corner[0][:, 1]))

    # Get the depth value at the center of the marker
    depth = depth_image[cY, cX]

    # Convert depth to real-world coordinates using RealSense camera intrinsics
    depth_scale = 0.001  # Typically in meters for RealSense
    depth_value = depth * depth_scale
    return (cX, cY, depth_value)

# Main loop
count = 0
while True:
    color_image, depth_image = get_depth_data()

    if color_image is None or depth_image is None:
        print("Skipping frame due to missing data.")
        continue

    # Undistort the image using the camera matrix and distortion coefficients
    undistorted_image = cv2.undistort(color_image, camera_matrix, dist_coeffs)

    # Detect ArUco markers in the undistorted image
    frame, corners, ids = detect_aruco_markers(undistorted_image)
    # If markers are detected, calculate their position

    if len(corners) > 0:
        for i, corner in enumerate(corners):
            # Get the 3D position of the marker
            marker_pos = get_marker_3d_position(corner, depth_image)
            print(f"Marker {ids[i][0]} position: {marker_pos}")
    """
    if int(ids) % 2 == 1: #odd go left
        print("Arcing Left")
        count = count + 1
    elif int(ids) % 2 == 0: #even Go Right
        print("arcing Left")
        count = count + 1
    if count ==  4 and len(corners) == 0:
        print("BREAKS")
        count = 0
    """
    # Display the image with ArUco markers
    cv2.imshow("ArUco Marker Detection", frame)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break

# Clean up
pipeline.stop()
cv2.destroyAllWindows()
