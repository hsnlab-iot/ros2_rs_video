import pyrealsense2 as rs
import threading
import numpy as np
import zmq
import time
import argparse

def main(args=None):

    # Create a zmq context and in-memory queue
    context = zmq.Context()
    zmq_c_socket = context.socket(zmq.PUB)
    zmq_c_socket.bind("ipc:///tmp/color_image")

    # Configure depth and color streams
    rs_pipeline = rs.pipeline()

    # Get the list of all connected devices
    ctx = rs.context()
    devices = ctx.query_devices()

    # Print settings for each device
    for device in devices:
        print(f"Device Name: {device.get_info(rs.camera_info.name)}")
        for sensor in device.query_sensors():
            sensor_name = sensor.get_info(rs.camera_info.name)
            print(f"  Sensor Name: {sensor_name}")

    # Set up the RealSense pipeline
    config = rs.config()
    print(f'Width: {args.width}, Height: {args.height}, Rate: {args.rate}')
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.rate)

    profile = rs_pipeline.start(config)

    cc = cs = 0

    # Start a new thread to fill the pipeline
    def fill_pipeline():
        while not video_stop:
        # Get frames from the RealSense pipeline
            frames = rs_pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            nonlocal cc, cs
            cc = cc + 1

            # Convert frame to numpy array
            color_image = np.asanyarray(color_frame.get_data())

            zmq_c_socket.send(color_image.tobytes())
            cs = len(color_image.tobytes())

    video_stop = False
    pipeline_thread = threading.Thread(target=fill_pipeline, daemon=True)
    pipeline_thread.start()

    def stat():
        occ = cc
        now = time.time()
        while not video_stop:
            # Interruptable sleep
            for i in range(20):
                if video_stop:
                    break
                time.sleep(0.5)

            fps = (cc - occ) / (time.time() - now)
            fps = round(fps, 2)
            print(f'Color: {fps} fps. Color: {round(cs)} bytes/frame')


    # Start a new thread to print statistics
    stat_thread = threading.Thread(target=stat, daemon=True)
    stat_thread.start()

    try:
        input("Press Enter to stop streaming...\n")
    except KeyboardInterrupt:
        pass

    # Stop the filling thread
    video_stop = True
    if pipeline_thread.is_alive():
        pipeline_thread.join()
    if stat_thread.is_alive():
        stat_thread.join()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="RealSense Streamer")
    parser.add_argument('--width', type=int, default=1280, help='Frame width')
    parser.add_argument('--height', type=int, default=720, help='Frame height')
    parser.add_argument('--rate', type=int, default=30, help='Frame rate (fps)')
    args = parser.parse_args()

    main(args)
