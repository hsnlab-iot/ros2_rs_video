from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_compression',
            executable='decompress',
            name='gstreamer_depth_decoder_node',
            output='screen',
            parameters=[
                {'width': 1280},  # Set the width of the video stream
                {'height': 720},  # Set the height of the video stream
                {'rate': 30},  # Set the framerate of the video stream
                {'frame_name': 'camera'},  # Set the frame name for the published image
                {'camera_info_file': '/opt/ws/src/realsense_d455_ros_calibration_factory_1280x720.yaml'}  # Path to the camera info YAML file
            ],
            remappings=[
                ('/video/h264', '/video/depth/h264'),  # Remap the input topic for compressed H.264 video
                ('/video/image_raw', '/robot/depth/image_raw'),  # Remap the output topic for raw images
                ('/video/camera_info', '/robot/depth/camera_info')  # Remap the output topic for camera info
            ]
        )
    ])