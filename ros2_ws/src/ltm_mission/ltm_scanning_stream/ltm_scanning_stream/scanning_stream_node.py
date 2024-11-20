""" This node create a 2D video stream, and records the total 3D radar mapping.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-11-19
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs_py import point_cloud2 as pc2
from std_srvs.srv import Trigger

import os
import cv2
from datetime import datetime
import numpy as np
import open3d as o3d
from ctypes import *

VIDEO_STREAM_WIDTH = 1280
VIDEO_STREAM_HEIGHT = 720
VIDEO_STREAM_FPS = 14.25

LTM_RECORDINGS_STREAM_DIRECTORY = os.environ.get('LTM_RECORDINGS_STREAM_DIRECTORY')

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

class ScanningStreamNode(Node):

    def __init__(self):
        super().__init__('scanning_stream_node')
        self.create_directory()
        self.initialize_trigger_service()
        self.initialize_pointcloud_subscriber()
        self.initialize_image_subscriber()
        self.get_logger().info('Scanning stream node initialized.')

    def pointcloud_callback(self, msg):
        self.pointcloud = msg

    def image_callback(self, msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.frames.append((frame, msg.header.stamp.sec, msg.header.stamp.nanosec))

    def trigger_callback(self, request, response):
        self.get_logger().info('Triggered scanning stream.')
        _ = request

        # Create video
        video_path = os.path.join(self.directory, 'video.avi')
        video_writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'XVID'), VIDEO_STREAM_FPS, (VIDEO_STREAM_WIDTH, VIDEO_STREAM_HEIGHT))

        for frame, _, _ in self.frames:
            video_writer.write(frame)

        video_writer.release()

        # Save current pointcloud as .pcd
        converted_pointcloud = self.convert_pointcloud2_to_open3d(self.pointcloud)
        self.save_pointcloud(converted_pointcloud)

        return response        

    def convert_pointcloud2_to_open3d(self, point_cloud2: PointCloud2) -> o3d.geometry.PointCloud:
        # Get cloud data from point_cloud2
        field_names=[field.name for field in point_cloud2.fields]
        cloud_data = list(pc2.read_points(point_cloud2, skip_nans=True, field_names = field_names))

        # Check empty
        o3d_cloud = o3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set o3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ]

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            # combine
            o3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
            o3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            o3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

        # return
        return o3d_cloud
    
    def save_pointcloud(self, pointcloud: o3d.geometry.PointCloud) -> bool:
        file_path = os.path.join(self.directory, 'radar_mapping.pcd')
        return o3d.io.write_point_cloud(file_path, pointcloud, compressed=True)

    def initialize_pointcloud_subscriber(self):
        self.pointcloud = None
        self.pointcloud_subscriber = self.create_subscription(PointCloud2,
            'point_cloud/mapping', self.pointcloud_callback, 1)
        
    def initialize_image_subscriber(self):
        self.frames = [] # List of frames in tuple (frame, timestamp)
        self.image_subscriber = self.create_subscription(Image,
            'camera/raw', self.image_callback, 1000)

    def initialize_trigger_service(self):
        self.trigger_service = self.create_service(Trigger,
            'scanning_stream/trigger', self.trigger_callback)
        
    def create_directory(self):
        current_datetime = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.directory = os.path.join(LTM_RECORDINGS_STREAM_DIRECTORY, current_datetime)

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        self.get_logger().info(f'Created directory: {self.directory}')


def main():
    rclpy.init()
    node = ScanningStreamNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
