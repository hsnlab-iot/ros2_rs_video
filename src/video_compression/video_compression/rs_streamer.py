import pyrealsense2 as rs
import threading
import numpy as np
import zmq
import time
import argparse

def main(args=None):

    if args.nocolor and args.nodepth:
        print("Error: Both color and depth streams are disabled. Enable at least one stream to proceed.")
        return
    # Create a zmq context and in-memory queue
    context = zmq.Context()
    if not args.nocolor:
        zmq_c_socket = context.socket(zmq.PUB)
        zmq_c_socket.bind("ipc:///tmp/color_image")
    if not args.nodepth:
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
    if not args.nocolor:
        config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.rate)
    if not args.nodepth:
        config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.rate)

    profile = rs_pipeline.start(config)
    align = rs.align(rs.stream.color)

    cc = cd = cs = ds = 0

    if not args.nodepth:
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , depth_scale)

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
        nonlocal cc, cd, cs, ds
        while not video_stop:
        # Get frames from the RealSense pipeline
            frames = rs_pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not depth_frame and not color_frame:
                continue

            if color_frame and not args.nocolor:
                cc = cc + 1

                color_image = np.asanyarray(color_frame.get_data())
                zmq_c_socket.send(color_image.tobytes())
                cs = len(color_image.tobytes())

            # Only align if we actually have a depth frame
            if depth_frame and not args.nodepth:
                if not args.noalign and not args.nocolor:
                    aligned_frames = align.process(frames)
                    depth_frame = aligned_frames.get_depth_frame()
                depth_frame = threshold_filter.process(depth_frame)
                depth_frame = spatial_filter.process(depth_frame)
                depth_frame = temporal_filter.process(depth_frame)
                #depth_frame = color_filter.process(depth_frame)
                cd = cd + 1

                depth_image = np.asanyarray(depth_frame.get_data())
                zmq_d_socket.send(depth_image.tobytes())
                ds = len(depth_image.tobytes())

    video_stop = False
    pipeline_thread = threading.Thread(target=fill_pipeline, daemon=True)
    pipeline_thread.start()

    def stat():
        nonlocal cc, cd, cs, ds
        while not video_stop:
            occ = cc
            ocd = cd
            now = time.time()

            # Interruptable sleep
            for i in range(20):
                if video_stop:
                    break
                time.sleep(0.5)

            fps_c = round((cc - occ) / (time.time() - now), 2)
            fps_d = round((cd - ocd) / (time.time() - now), 2)
            if not args.nocolor:
                print(f'Color: {fps_c} fps. {round(cs)} bytes/frame')
            if not args.nodepth:
                print(f'Depth: {fps_d} fps. {round(ds)} bytes/frame')

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

    rs_pipeline.stop()
    if not args.nocolor:
        zmq_c_socket.close()
    if not args.nodepth:
        zmq_d_socket.close()
    context.term()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="RealSense Streamer")
    parser.add_argument('--width', type=int, default=1280, help='Frame width')
    parser.add_argument('--height', type=int, default=720, help='Frame height')
    parser.add_argument('--rate', type=int, default=30, help='Frame rate (fps)')
    parser.add_argument('--noalign', action='store_true', default=False, help='Disable depth to color alignment')
    parser.add_argument('--nocolor', action='store_true', default=False, help='Disable color stream')
    parser.add_argument('--nodepth', action='store_true', default=False, help='Disable depth stream')
    args = parser.parse_args()

    main(args)
