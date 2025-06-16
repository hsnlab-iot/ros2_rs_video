#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import numpy as np
import cv2
import struct
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def write_ply(filename, points, colors):
    with open(filename, 'w') as f:
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write(f'element vertex {len(points)}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('property uchar red\nproperty uchar green\nproperty uchar blue\n')
        f.write('end_header\n')
        for p, c in zip(points, colors):
            f.write(f"{p[0]} {p[1]} {p[2]} {c[0]} {c[1]} {c[2]}\n")

class Save2PLYNode(Node):
    def __init__(self):
        super().__init__('save2ply')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.color_sub = message_filters.Subscriber(self, Image, 'color/image_raw', qos_profile=qos)
        self.depth_sub = message_filters.Subscriber(self, Image, 'depth/image_raw', qos_profile=qos)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, 'color/camera_info', qos_profile=qos)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)
        self.saved = False

    def callback(self, color_msg, depth_msg, info_msg):
        if self.saved:
            return
        self.saved = True
        self.get_logger().info("Received all messages, saving PLY...")

        # Convert ROS images to numpy
        color = np.frombuffer(color_msg.data, dtype=np.uint8).reshape((color_msg.height, color_msg.width, -1))
        if color.shape[2] == 1:
            color = cv2.cvtColor(color, cv2.COLOR_GRAY2RGB)
        elif color.shape[2] == 4:
            color = cv2.cvtColor(color, cv2.COLOR_RGBA2RGB)
        depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape((depth_msg.height, depth_msg.width))

        # Camera intrinsics
        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        points = []
        colors = []
        for v in range(depth.shape[0]):
            for u in range(depth.shape[1]):
                z = depth[v, u] * 0.001  # assuming depth in mm, convert to meters
                if z == 0:
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])
                colors.append(color[v, u, :3])

        write_ply('pointcloud.ply', points, colors)
        self.get_logger().info("PLY file saved as pointcloud.ply")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = Save2PLYNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()