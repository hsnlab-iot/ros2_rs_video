import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import zmq
import threading

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

class ZMQGStreamerPublisher(Node):
    def __init__(self):
        print("D3")
        Gst.init(None)  # Initialize GStreamer
        print("D3.1")
        super().__init__('zmq_h264_gstreamer_publisher')
        print("D4")
        rclpy.logging.set_logger_level(self.get_logger().name, rclpy.logging.LoggingSeverity.INFO)
        print("DEUG")
        self.get_logger().info("Initializing ZMQ GStreamer Publisher Node")

        # Declare parameters
        self.declare_parameter('zmq_url', 'ipc:///tmp/image')
        self.declare_parameter('mode', 'color')

        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('rate', 30)
        self.declare_parameter('bitrate', 2000)  # Default bitrate in kbps

        self.zmq_url = self.get_parameter('zmq_url').get_parameter_value().string_value
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value
        self.bitrate = self.get_parameter('bitrate').get_parameter_value().integer_value

        if self.mode not in ['color', 'depth']:
            self.get_logger().error("Invalid mode specified. Use 'color' or 'depth'.")
            raise ValueError("Invalid mode specified.")
        
        self.gst_pipeline = (
            'appsrc name=src ! '
            'videoconvert ! '
            f'video/x-raw,format=I420,width={self.width},height={self.height},framerate={self.rate}/1 ! '
            f'x264enc tune=zerolatency bitrate={self.bitrate} speed-preset=ultrafast b-adapt=false key-int-max=3 sliced-threads=true byte-stream=true ! '
            'video/x-h264,profile=main,stream-format=byte-stream,alignement=nal ! '
            'appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true'
        )

        self.get_logger().info("Starting ZMQ GStreamer Publisher")

        self.publisher = self.create_publisher(CompressedImage, "video/h264", 3)
                
        #self.mainloop = GLib.MainLoop()
        self.pipeline = Gst.parse_launch(self.gst_pipeline)
        self.appsrc = self.pipeline.get_by_name('appsrc')
        self.appsink = self.pipeline.get_by_name('appsink')

        if self.appsrc is None or self.appsink is None:
            self.get_logger().error("GStreamer pipeline must contain named 'appsrc' and 'appsink' elements.")
            raise RuntimeError("Invalid GStreamer pipeline.")

        self.appsink.connect('new-sample', self.on_new_sample)

        # Start GStreamer pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

        # Start ZMQ thread
        self.zmq_thread = threading.Thread(target=self.zmq_listener, daemon=True)
        self.zmq_thread.start()

    def zmq_listener(self):
        self.get_logger().warn(f"Listening for ZMQ messages on {self.zmq_url}")
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(self.zmq_url)
        socket.setsockopt_string(zmq.SUBSCRIBE, '')
        self.get_logger().info(f"Listening for ZMQ messages on {self.zmq_url}")

        while rclpy.ok():
            try:
                data = socket.recv()
                # Feed data to GStreamer appsrc
                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)
                self.appsrc.emit('push-buffer', buf)
            except Exception as e:
                self.get_logger().error(f"ZMQ error: {e}")

    def on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "h264"
        msg.data = mapinfo.data

        self.publisher.publish(msg)
        buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

def main(args=None):
    print("Starting ZMQ GStreamer Publisher Node")
    rclpy.init(args=args)
    print("D1")
    node = ZMQGStreamerPublisher()
    print("D2")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pipeline.set_state(Gst.State.NULL)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()