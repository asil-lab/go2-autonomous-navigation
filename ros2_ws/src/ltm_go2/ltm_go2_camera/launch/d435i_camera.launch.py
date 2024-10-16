"""
Project Lava Tube Mapping, Technical University of Delft.
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription

def generate_launch_description():

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                            'launch', 'rs_launch.py')
        ),
        launch_arguments=[
            ('camera_namespace', 'd435i'),
            ('enable_depth', 'true'),
            ('enable_infra', 'true'),
            ('enable_sync', 'true'),
            ('enable_rgbd', 'false'),
            ('enable_gyro', 'false'),
            ('enable_accel', 'false'),
            ('publish_tf', 'false'),
            ('pointcloud.enable', 'true'),
        ],
    )

    return LaunchDescription([
        realsense_node
    ])
    