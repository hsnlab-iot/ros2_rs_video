from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_compression',
            executable='compress',
            name='zmq_h264_gstreamer_publisher',
            output='screen',
            remappings=[
                ('video/h264', 'video/color_h264')
            ],
            parameters=[
                {'zmq_url': 'ipc:///tmp/color_image'},
                {'mode': 'color'},
            ]
        )
    ])