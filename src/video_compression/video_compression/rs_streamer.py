import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import pyrealsense2 as rs
import threading
import numpy as np
import zmq
import time
import argparse
from colorizer import depth16_to_12bit_piecewise, depth16_to_12bit_piecewise_numba, encode_12bit_to_rgb_4bit_channels, encode_12bit_to_rgb_4bit_channels_numba

Gst.init(None)

def main(args=None):

    # Create a zmq context and in-memory queue
    context = zmq.Context()
    zmq_c_socket = context.socket(zmq.PUB)
    zmq_c_socket.bind("ipc:///tmp/color_frame_queue")
    zmq_d_socket = context.socket(zmq.PUB)
    zmq_d_socket.bind("ipc:///tmp/depth_frame_queue")
    if (args is not None) and args.raw:
        zmq_cr_socket = context.socket(zmq.PUB)
        zmq_cr_socket.bind("ipc:///tmp/color_raw_frame_queue")
        zmq_dr_socket = context.socket(zmq.PUB)
        zmq_dr_socket.bind("ipc:///tmp/depth_raw_frame_queue")

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

    def on_color_sample(sink):
        nonlocal sc, cc, zmq_c_socket
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR

        buffer = sample.get_buffer()
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        # Publish the frame data to the in-memory queue
        zmq_c_socket.send(map_info.data)
        sc = sc + buffer.get_size()
        buffer.unmap(map_info)

        cc = cc + 1
        return Gst.FlowReturn.OK

    def on_depth_sample(sink):
        nonlocal sd, cd, zmq_d_socket
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR

        buffer = sample.get_buffer()
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        # Publish the frame data to the in-memory queue
        zmq_d_socket.send(map_info.data)
        sd = sd + buffer.get_size()
        buffer.unmap(map_info)

        cd = cd + 1
        return Gst.FlowReturn.OK

    # GStreamer pipeline
    gst_pipeline_c_description = (
        'appsrc name=color ! '
        'videoconvert ! '
        f'video/x-raw,format=I420,width={args.width},height={args.height},framerate={args.rate}/1 ! '
        f'x264enc tune=zerolatency bitrate={args.bitrate} speed-preset=ultrafast b-adapt=false key-int-max=3 sliced-threads=true byte-stream=true ! '
        'video/x-h264,profile=main,stream-format=byte-stream,alignement=nal ! '
        'appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true'
    )

    gst_pipeline_d_description = (
        'appsrc name=depth ! '
        'videoconvert ! '
        f'video/x-raw,format=I420,width={args.width},height={args.height},framerate={args.rate}/1 ! '
        f'x264enc tune=zerolatency bitrate={args.bitrate} speed-preset=ultrafast b-adapt=false key-int-max=3 sliced-threads=true byte-stream=true ! '
        'video/x-h264,profile=main,stream-format=byte-stream,alignement=nal ! '
        'appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true'
    )

    gst_c_pipeline = Gst.parse_launch(gst_pipeline_c_description)
    gst_d_pipeline = Gst.parse_launch(gst_pipeline_d_description)

    cc = 0
    sc = 0
    cd = 0
    sd = 0

    appsink_c = gst_c_pipeline.get_by_name('sink')
    appsink_c.connect('new-sample', on_color_sample)
    
    appsrc_c = gst_c_pipeline.get_by_name('color')
    appsrc_c.set_property('is-live', True)
    appsrc_c.set_property('block', True)
    appsrc_c.set_property('format', Gst.Format.TIME)
    appsrc_c.set_property('caps', Gst.Caps.from_string(f'video/x-raw,format=RGB,width={args.width},height={args.height},framerate={args.rate}/1'))

    appsink_d = gst_d_pipeline.get_by_name('sink')
    appsink_d.connect('new-sample', on_depth_sample)

    appsrc_d = gst_d_pipeline.get_by_name('depth')
    appsrc_d.set_property('is-live', True)
    appsrc_d.set_property('block', True)
    appsrc_d.set_property('format', Gst.Format.TIME)
    appsrc_d.set_property('caps', Gst.Caps.from_string(f'video/x-raw,format=RGB,width={args.width},height={args.height},framerate={args.rate}/1'))

    # Start pipeline
    gst_c_pipeline.set_state(Gst.State.PLAYING)
    gst_d_pipeline.set_state(Gst.State.PLAYING)

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

            depth_frame = threshold_filter.process(depth_frame)
            depth_frame = spatial_filter.process(depth_frame)
            depth_frame = temporal_filter.process(depth_frame)
            #depth_frame = color_filter.process(depth_frame)

            # Convert frame to numpy array
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Colorize
            depth_image = colorizer.encode(depth_image)

            if (args is not None) and args.raw:
                # Publish raw frames to zmq sockets
                zmq_cr_socket.send(color_image.tobytes())
                zmq_dr_socket.send(depth_image.tobytes())

            # Push the frame into the GStreamer pipeline
            buffer = Gst.Buffer.new_wrapped(color_image.tobytes())
            appsrc_c.emit('push-buffer', buffer)
            buffer = Gst.Buffer.new_wrapped(depth_image.tobytes())
            appsrc_d.emit('push-buffer', buffer)

    video_stop = False
    pipeline_thread = threading.Thread(target=fill_pipeline, daemon=True)
    pipeline_thread.start()

    def stat():
        nonlocal cc, sc, cd, sd
        occ = osc = ocd = osd = 0
        while not video_stop:
            acc = (cc - occ) / 10
            asc = (sc - osc) / (cc - occ) if (cc - occ) > 0 else 0
            acd = (cd - ocd) / 10
            asd = (sd - osd) / (cd - ocd) if (cd - ocd) > 0 else 0
            print(f'Color: {acc} fps {round(asc)} bytes/frame, Depth: {acd} fps {round(asd)} bytes/frame')
            occ = cc
            osc = sc
            ocd = cd
            osd = sd

            for i in range(20):
                if video_stop:
                    break
                time.sleep(0.5)

    # Start a new thread to print statistics
    stat_thread = threading.Thread(target=stat, daemon=True)
    stat_thread.start()

    try:
        loop = GLib.MainLoop()
        loop.run()
    except KeyboardInterrupt:
        loop.quit()

    # Stop the filling thread
    video_stop = True
    if pipeline_thread.is_alive():
        pipeline_thread.join()
    if stat_thread.is_alive():
        stat_thread.join()

    gst_c_pipeline.set_state(Gst.State.NULL)
    gst_d_pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="RealSense Streamer")
    parser.add_argument('--raw', action='store_true', help='Send frames in raw as well')
    parser.add_argument('--width', type=int, default=1280, help='Frame width')
    parser.add_argument('--height', type=int, default=720, help='Frame height')
    parser.add_argument('--rate', type=int, default=30, help='Frame rate (fps)')
    parser.add_argument('--bitrate', type=int, default=2000, help='Bitrate for video encoding (kbps)')
    args = parser.parse_args()

    main(args)
