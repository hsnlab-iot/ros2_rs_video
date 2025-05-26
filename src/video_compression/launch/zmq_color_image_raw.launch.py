from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_compression',
            executable='raw',
            name='image_raw_color_publisher_node',
            output='screen',
            parameters=[
                {'zmq_endpoint': 'ipc:///tmp/color_raw_frame_queue'},  # ZMQ endpoint for receiving frames
                {'frame_id': 'camera'}  # Frame ID for the published images
            ],
            remappings=[
                ('image_raw', '/video/color/image_raw')  # Remap the outgoing topic
            ]
        )
    ])