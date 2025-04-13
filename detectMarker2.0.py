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
                motor_queue.put((movement.pan, (hor,)))

            if abs(cY - FRAME_CENTER_Y) > TOLERANCE:
                if cY < FRAME_CENTER_Y:
                    vert = min(8000, vert + STEP)
                else:
                    vert = max(4000, vert - STEP)
                motor_queue.put((movement.tilt, (vert,)))

        time.sleep(0.05)
