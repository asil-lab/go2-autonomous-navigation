"""
Project Lava Tube Mapping, Technical University of Delft.
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 
                'launch', 'rs_launch.py')
        ),
        launch_arguments=[
            ('camera_name', 'd435i_camera'),
            ('camera_namespace', ''),
            ('initial_reset', 'true'),
            ('accelerate_gpu_with_glsl', 'true'),
            ('enable_infra1', 'true'),
            ('enable_infra2', 'true'),
            ('publish_tf', 'true'),
        ],
    )

    # Depth image to pointcloud node
    depth_image_to_pointcloud_node = Node(
        package='depthimage_to_pointcloud2',
        executable='depthimage_to_pointcloud2_node',
        name='depthimage_to_pointcloud2_node',
        output='screen',
        #parameters=[
        #    {
        #        'range_max': '0.0',
        #        'use_quiet_nan': 'true',
        #        'colorful': 'false'
        #    }
        #],
        remappings=[
            # ('image', 'd435i_camera/color/image_raw'),
            ('depth', 'd435i_camera/depth/image_rect_raw'),
            ('depth_camera_info', 'd435i_camera/depth/camera_info'),
            ('pointcloud2', 'point_cloud/camera/raw'),
        ],
    )

    return LaunchDescription([
        realsense_node,
        depth_image_to_pointcloud_node,
    ])
    
