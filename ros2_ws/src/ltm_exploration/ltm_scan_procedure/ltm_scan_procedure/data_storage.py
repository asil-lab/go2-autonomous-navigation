""" This is the script to store data from the LTM project.
Author: Alexander James Becoy
Revision: 1.0
Date: 10-09-2024
"""

import numpy as np
from ctypes import *

import os
from datetime.datetime import now
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import open3d as o3d

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

class DataStorage:

    def __init__(self) -> None:
        # Configure storage directory
        self.configure_storage_directory()

        # Initialize attributes
        self.reset_point_cloud()

    def convert_point_cloud2_to_open3d(self, point_cloud2: PointCloud2) -> o3d.geometry.PointCloud:
        """ Converts a PointCloud2 message to an o3d point cloud.
        Credits: https://github.com/felixchenfy/open3d_ros_pointcloud_conversion

        Args:
            point_cloud2 (PointCloud2): The PointCloud2 message to be converted.

        Returns:
            o3d.geometry.PointCloud: The o3d point cloud.
        """
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

    def add_point_cloud(self, point_cloud: o3d.geometry.PointCloud) -> None:
        """ Adds a point cloud to the data storage.

        Args:
            point_cloud (o3d.geometry.PointCloud): The point cloud to be added.
        """
        self.point_cloud += point_cloud

    def reset_point_cloud(self) -> None:
        """ Clears the point cloud data. """
        self.point_cloud = o3d.geometry.PointCloud()

    def save_point_cloud(self, file_name: str) -> bool:
        """ Saves the point cloud data to a PCD file.

        Args:
            file_name (str): The name of the file to be saved.

        Returns:
            (bool): Has the point cloud been saved successfully. 
        """
        file_path = os.path.join(self.storage_directory, file_name)
        return o3d.io.write_point_cloud(file_path, self.point_cloud, compressed=True)

    def create_storage_subdirectory(self, subdirectory_name: str) -> None:
        """ Creates a subdirectory in the storage directory.

        Args:
            subdirectory_name (str): The name of the subdirectory to be created.
        """
        subdirectory_path = os.path.join(self.storage_directory, subdirectory_name)

        # Create the subdirectory if it does not exist
        if not os.path.exists(subdirectory_path):
            os.makedirs(subdirectory_path)
            print(f'Subdirectory has been created at {subdirectory_path}.')

    def configure_storage_directory(self) -> None:
        """ Configures the storage directory for the data. """
        self.storage_directory = os.path.join(os.path.expanduser('~'), 'ltm_data')

        # Create the storage directory if it does not exist
        if not os.path.exists(self.storage_directory):
            os.makedirs(self.storage_directory)
            print(f'Storage directory has been created at {self.storage_directory}.')

        # Create a directory for the current session based on the current date and time
        self.storage_directory = os.path.join(self.storage_directory, now().strftime('%Y-%m-%d_%H-%M-%S'))
