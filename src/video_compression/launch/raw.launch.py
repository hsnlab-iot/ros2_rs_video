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
                {'mode': 'color'},  # Mode can be 'color' or 'depth'
                {'width': 1280},  # Width of the image
                {'height': 720},  # Height of the image
                {'zmq_endpoint': 'ipc:///tmp/color_image'},  # ZMQ endpoint for receiving frames
                {'frame_id': 'camera'}  # Frame ID for the published images
            ],
            remappings=[
                ('image_raw', '/video/color/image_raw')  # Remap the outgoing topic
            ]
        ),
        Node(
            package='video_compression',
            executable='raw',
            name='image_raw_cdepth_publisher_node',
            output='screen',
            parameters=[
                {'mode': 'depth'},  # Mode can be 'color' or 'depth'
                {'width': 1280},  # Width of the image
                {'height': 720},  # Height of the image
                {'zmq_endpoint': 'ipc:///tmp/depth_image'},  # ZMQ endpoint for receiving frames
                {'frame_id': 'camera'}  # Frame ID for the published images
            ],
            remappings=[
                ('image_raw', '/video/depth/image_raw')  # Remap the outgoing topic
            ]
        )
    ])