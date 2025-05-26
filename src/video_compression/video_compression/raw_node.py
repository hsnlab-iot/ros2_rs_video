import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import threading
import zmq
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ZMQRawPublisher(Node):
    def __init__(self):
        super().__init__('raw_publisher_node')

        video_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )

        self.publisher_ = self.create_publisher(Image, 'image_raw', video_qos)
        self.get_logger().info("Starting raw Publisher node")
        
        self.declare_parameter('zmq_endpoint', 'ipc:///tmp/frame_queue', 
            ParameterDescriptor(type_=ParameterType.PARAMETER_STRING, description='ZMQ endpoint, the source of the frames'))  
        zmq_endpoint = self.get_parameter('zmq_endpoint').get_parameter_value().string_value

        self.declare_parameter('frame_id', 'camera',
            ParameterDescriptor(type_=ParameterType.PARAMETER_STRING, description='Frame ID for the published images'))
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Set up ZeroMQ context and subscriber socket
        context = zmq.Context()
        subscriber = context.socket(zmq.SUB)
        subscriber.setsockopt(zmq.CONFLATE, 1)
        subscriber.connect(zmq_endpoint)
        subscriber.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

        def zmq_listener():
            self.c = 0
            while rclpy.ok():
                try:
                    # Receive message from ZMQ
                    message = subscriber.recv(flags=zmq.NOBLOCK)
                    if message:
                        # Create and publish Image message
                        image = Image()
                        image.data = message
                        image.height = 720  # Set appropriate height
                        image.width = 1280
                        image.encoding = 'bgr8'  # Set appropriate encoding
                        image.is_bigendian = False
                        image.step = image.width * 3
                        # Set header information
                        image.header.stamp = self.get_clock().now().to_msg()
                        image.header.frame_id = frame_id
                        self.publisher_.publish(image)

                        self.c += 1
                        if self.c % 100 == 0:
                            self.get_logger().info(f"Published {self.c} messages")
                except zmq.Again:
                    pass  # No message received, continue loop

        # Start ZMQ listener in a separate thread
        self.zmq_thread = threading.Thread(target=zmq_listener, daemon=True)
        self.zmq_thread.start()

    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    # Initialize ROS2 node

    rclpy.init(args=args)
    node = ZMQRawPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
