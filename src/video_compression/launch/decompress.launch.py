import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    depth_mode = os.environ.get('DEPTH_MODE', 'depth_yuv_12')

    return LaunchDescription([
        Node(
            package='video_compression',
            executable='decompress',
            name='gstreamer_color_decoder_node',
            output='screen',
            parameters=[
                {'width': 1280},
                {'height': 720},
                {'rate': 30},
                {'frame_name': 'camera'},
                {'camera_info_file': '/opt/ws/calibration.yaml'}
            ],
            remappings=[
                ('/video/h264', '/video/color_h264'),
                ('/video/image_raw', '/video/color/image_raw'),
                ('/video/camera_info', '/video/color/camera_info')
            ]
        ),
        Node(
            package='video_compression',
            executable='decompress',
            name='gstreamer_depth_decoder_node',
            output='screen',
            parameters=[
                {'width': 1280},
                {'height': 720},
                {'rate': 30},
                {'frame_name': 'camera'},
                {'camera_info_file': '/opt/ws/calibration.yaml'},
                {'mode': depth_mode}
            ],
            remappings=[
                ('/video/h264', '/video/depth_h264'),
                ('/video/image_raw', '/video/depth/image_raw'),
                ('/video/camera_info', '/video/depth/camera_info')
            ]
        ),
    ])
