import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

try:
    pipeline.start(config)
    print("RealSense camera started.")
    for i in range(30):
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            print("No color frame.")
            continue
        print("Got color frame.")

finally:
    pipeline.stop()
