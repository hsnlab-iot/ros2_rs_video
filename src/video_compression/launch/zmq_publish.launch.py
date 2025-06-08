from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_compression',
            executable='compress',
            name='h264_color_publisher_node',
            output='screen',
            parameters=[
                {'zmq_endpoint': 'ipc:///tmp/color_frame_queue'},  # ZMQ endpoint for receiving frames
                {'frame_id': 'camera'}  # Frame ID for the published images
            ],
            remappings=[
                ('video/h264', '/video/color/h264')  # Remap the outgoing topic
            ]
        )
    ])