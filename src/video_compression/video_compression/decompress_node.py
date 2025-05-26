import rclpy
from rclpy.node import Node

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp

import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import yaml
import os

class GStreamerDecoder(Node):
    def __init__(self):
        super().__init__('gstreamer_decoder_node')

        self.bridge = CvBridge()
        self.frame_name = 'camera'

        # Declare ROS2 parameters for width, height, and framerate
        self.declare_parameter('width', 1280,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Width of the video stream in pixels.",
            ),)
        self.declare_parameter('height', 720,
            descriptor=ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description="Height of the video stream in pixels.",
            ),)
        self.declare_parameter('rate', 30,
            descriptor=ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description="Framerate of the video stream in frames per second.",
            ),)                   
        self.declare_parameter('frame_name', 'camera',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Frame name for the published image.",
            ),) 
        self.declare_parameter('camera_info_file', '',
            descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Path to the camera info YAML file.",
            ),
        )

        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value
        self.frame_name = self.get_parameter('frame_name').get_parameter_value().string_value

        # Initialize GStreamer
        Gst.init(None)

        # Build GStreamer pipeline
        self.pipeline = Gst.parse_launch(
            'appsrc name=src is-live=true format=time ! '
            'video/x-h264, stream-format=byte-stream, alignment=nal, profile=main ! '
            'h264parse ! avdec_h264 ! videoconvert ! '
            'video/x-raw, format=BGR ! '
            'appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true'
        )

        # Setup appsrc and appsink
        self.appsrc = self.pipeline.get_by_name('src')
        caps = Gst.Caps.from_string(
            f'video/x-h264, stream-format=byte-stream, alignment=nal, profile=main, width={self.width}, height={self.height}, framerate={self.rate}/1'
        )
        self.appsrc.set_property('caps', caps)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect('new-sample', self.on_new_sample)

        video_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )

        self.publisher = self.create_publisher(
            Image,
            '/video/image_raw',
            video_qos
        )
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/video/camera_info', 3)

        # Load camera info from file
        camera_info_file = self.get_parameter('camera_info_file').get_parameter_value().string_value
        self.camera_info = None
        if camera_info_file != '':
            if not os.path.exists(camera_info_file):
                self.get_logger().error(f"Camera info file {camera_info_file} does not exist.")
                raise FileNotFoundError(f"Camera info file {camera_info_file} does not exist.")
            else:
                self.get_logger().info(f"Loading camera info from {camera_info_file}")
                self.camera_info = self.load_camera_info(camera_info_file)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/video/h264',
            self.compressed_callback,
            video_qos
        )

        self.get_logger().info("GStreamer pipeline initialized.")

    def load_camera_info(self, file_path):
        with open(file_path, 'r') as f:
            calib_data = yaml.safe_load(f)

        camera_info = CameraInfo()
        camera_info.width = calib_data['image_width']
        camera_info.height = calib_data['image_height']
        camera_info.k = calib_data['camera_matrix']['data']
        camera_info.d = calib_data['distortion_coefficients']['data']
        camera_info.r = calib_data['rectification_matrix']['data']
        camera_info.p = calib_data['projection_matrix']['data']
        camera_info.distortion_model = calib_data['distortion_model']

        return camera_info

    def publish_camera_info(self):
        self.camera_info.header.stamp = self.get_clock().now().to_msg()
        self.camera_info.header.frame_id = self.frame_name
        self.camera_info_publisher.publish(self.camera_info)

    def start(self):
        # Start the GStreamer pipeline
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info("GStreamer pipeline started.")

        if self.camera_info is not None:
            self.timer = self.create_timer(1.0, self.publish_camera_info)

    def compressed_callback(self, msg: CompressedImage):
        # Push H.264 bytes to appsrc
        buf = Gst.Buffer.new_allocate(None, len(msg.data), None)
        buf.fill(0, msg.data)
        self.appsrc.emit('push-buffer', buf)

    def on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if not sample:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()
        width = caps.get_structure(0).get_value('width')
        height = caps.get_structure(0).get_value('height')

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        try:
            frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3))
            ros_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = self.frame_name
            self.publisher.publish(ros_img)
        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

def main(args=None):
    rclpy.init(args=args)

    node = GStreamerDecoder()
    node.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
