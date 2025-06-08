from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_compression',
            executable='compress',
            name='zmq_h264_color_publisher_node',
            output='screen',
            parameters=[
                {'mode': 'color'},  # Mode for the node, can be 'color' or 'depth'
                {'width': 1280},  # Width of the video frames
                {'height': 720},  # Height of the video frames
                {'rate': 30},  # Frame rate for the video stream
                {'bitrate': 2000},  # Bitrate for H264 encoding in kbps
                {'zmq_url': 'ipc:///tmp/color_frame_queue'},  # ZMQ endpoint for receiving frames
                {'frame_id': 'camera'}  # Frame ID for the published images
            ],
            remappings=[
                ('video/h264', '/video/color/h264')  # Remap the outgoing topic
            ]
        )
    ])