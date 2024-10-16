"""
Project Lava Tube Mapping, Technical University of Delft.
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # realsense_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('realsense2_camera'),
    #                         'launch', 'rs_launch.py')
    #     ),
    #     launch_arguments=[
    #         ('camera_namespace', 'd435i'),
    #         ('enable_depth', 'true'),
    #         ('enable_infra', 'true'),
    #         ('enable_sync', 'true'),
    #         ('enable_rgbd', 'true'),
    #         ('enable_gyro', 'true'),
    #         ('enable_accel', 'true'),
    #         ('publish_tf', 'true'),
    #         ('pointcloud.enable', 'true'),
    #     ],
    # )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        output='screen',
        parameters=[
            {
                'camera_name': 'd435i',
                'camera_namespace': '',
                # 'enable_depth': True,
                # 'enable_infra': True,
                # 'enable_sync': True,
                # 'enable_rgbd': True,
                # 'enable_gyro': True,
                # 'enable_accel': True,
                # 'publish_tf': True,
                'pointcloud.enable': True
            }
        ]

    return LaunchDescription([
        realsense_node
    ])
    