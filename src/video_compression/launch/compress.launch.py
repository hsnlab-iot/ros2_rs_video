import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Read bitrate from environment, default to 2000 if not set
    color_bitrate = int(os.environ.get('COLOR_BITRATE', 2000))
    depth_bitrate = int(os.environ.get('DEPTH_BITRATE', 2000))
    depth_mode = os.environ.get('DEPTH_MODE', 'depth_yuv_12')

    return LaunchDescription([
        Node(
            package='video_compression',
            executable='compress',
            name='compress_color',
            output='screen',
            remappings=[
                ('video/h264', 'video/color_h264')
            ],
            parameters=[
                {'zmq_url': 'ipc:///tmp/color_image'},
                {'mode': 'color'},
                {'width': 1280},
                {'height': 720},
                {'bitrate': color_bitrate}
            ]
        ),
        Node(
            package='video_compression',
            executable='compress',
            name='compress_depth',
            output='screen',
            remappings=[
                ('video/h264', 'video/depth_h264')
            ],
            parameters=[
                {'zmq_url': 'ipc:///tmp/depth_image'},
                {'mode': depth_mode},
                {'width': 1280},
                {'height': 720},
                {'bitrate': depth_bitrate}
            ]
        )
    ])
