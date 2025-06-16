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
    zmq_d_socket = context.socket(zmq.PUB)
    zmq_d_socket.bind("ipc:///tmp/depth_image")

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
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.rate)

    profile = rs_pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)

    cc = cs = ds = 0

    # Setup depth
    align = rs.align(rs.stream.color)

    threshold_filter = rs.threshold_filter()
    temporal_filter = rs.temporal_filter()
    spatial_filter = rs.spatial_filter()
    #color_filter = rs.colorizer()

    MIN_DEPTH = 0.3
    MAX_DEPTH = 10.0

    threshold_filter.set_option(rs.option.min_distance, MIN_DEPTH)
    threshold_filter.set_option(rs.option.max_distance, MAX_DEPTH)

    spatial_filter.set_option(rs.option.filter_magnitude, 2)
    spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial_filter.set_option(rs.option.filter_smooth_delta, 20)
    spatial_filter.set_option(rs.option.holes_fill, 4)

    temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.4)
    temporal_filter.set_option(rs.option.holes_fill, 3) # it is called this way

    #color_filter.set_option(rs.option.histogram_equalization_enabled, 0)
    #color_filter.set_option(rs.option.color_scheme, 9)		# Hue colorization
    #color_filter.set_option(rs.option.min_distance, MIN_DEPTH)
    #color_filter.set_option(rs.option.max_distance, MAX_DEPTH)

    # Start a new thread to fill the pipeline
    def fill_pipeline():
        while not video_stop:
        # Get frames from the RealSense pipeline
            frames = rs_pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not depth_frame or not color_frame:
                continue

            nonlocal cc, cs, ds
            cc = cc + 1

            depth_frame = threshold_filter.process(depth_frame)
            depth_frame = spatial_filter.process(depth_frame)
            depth_frame = temporal_filter.process(depth_frame)
            #depth_frame = color_filter.process(depth_frame)

            # Convert frame to numpy array
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            zmq_c_socket.send(color_image.tobytes())
            zmq_d_socket.send(depth_image.tobytes())
            cs = len(color_image.tobytes())
            ds = len(depth_image.tobytes())

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
            print(f'Color/depth: {fps} fps. Color: {round(cs)} bytes/frame, Depth: {round(ds)} bytes/frame')


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
