"""
Project Lava Tube Mapping, Technical University of Delft.
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

import numpy as np

def generate_launch_description():

    # Crop box filter node
    crop_box_filter_node = Node(
        package='ltm_pointcloud_filter',
        executable='crop_box_filter_node',
        name='crop_box_filter_node',
        output='screen',
        parameters=[{
            'input_pointcloud_topic_name': 'point_cloud/raw',
            'output_pointcloud_topic_name': 'point_cloud/cropped',
            'min_x': -0.1,
            'max_x': 0.65,
            'min_y': -0.25,
            'max_y': 0.25,
            'min_z': -0.4,
            'max_z': 0.25,
        }],
    )

    # Pointcloud buffer node
    pointcloud_buffer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_pointcloud_buffer'), 
                'launch', 'pointcloud_buffer.launch.py')
        )
    )

    # Voxel grid filter node
    voxel_grid_filter_node = Node(
        package='ltm_pointcloud_filter',
        executable='voxel_grid_filter_node',
        name='voxel_grid_filter_node',
        output='screen',
        parameters=[{
            'input_pointcloud_topic_name': 'point_cloud/buffered',
            'output_pointcloud_topic_name': 'point_cloud/downsampled',
            'leaf_size_x': 0.01,
            'leaf_size_y': 0.01,
            'leaf_size_z': 0.01,
        }],
    )

    # Voxel grid filter node
    ground_plane_segmentation_node = Node(
        package='ltm_pointcloud_filter',
        executable='ground_plane_segmentation_node',
        name='ground_plane_segmentation_node',
        output='screen',
        parameters=[{
            'input_pointcloud_topic_name': 'point_cloud/downsampled',
            'output_pointcloud_topic_name': 'point_cloud/plane_segmented',
            'distance_threshold': 0.1,
            'max_iterations': 1000,
            'probability': 0.9,
        }],
    )

    # Pointcloud to laserscan node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[('cloud_in', 'point_cloud/plane_segmented')],
        parameters=[{
            'min_height': 0.0,
            'max_height': 0.3,
            'angle_increment': np.pi / 720.0,
            'queue_size': 1,
            'scan_time': 0.01,
            'range_min': 0.0,
            'range_max': 5.0,
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.05,
            'use_inf': True,
        }],
    )

    # Online synchronous SLAM node
    online_sync_slam_config_filename = 'mapper_params_online_sync.yaml'
    online_sync_slam_config_filepath = os.path.join(
        get_package_share_directory('ltm_exploration_core'), 'config', online_sync_slam_config_filename)
    online_sync_slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')
        ),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('params_file', online_sync_slam_config_filepath),
            # ('use_map_saver', LaunchConfiguration('use_map_saver')),
        ],
    )

    # Octomapping node
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_node',
        output='screen',
        parameters=[{
            'use_sim_time': 'false',
            'resolution': 0.0001,
            'frame_id': 'map',
            'base_frame_id': 'base_footprint',
            'filter_ground': 'false',
        }],
        remappings=[
            ('cloud_in', 'point_cloud/cropped'),                     # Input pointcloud
            ('octomap_point_cloud_centers', 'point_cloud/mapping')   # Output pointcloud
        ],
    )

    # Voxel grid filter node
    pointcloud_transformer_node = Node(
        package='ltm_pointcloud_transformer',
        executable='pointcloud_transformer_node',
        name='pointcloud_transformer_node',
        output='screen',
        parameters=[{
            'input_pointcloud_topic_name': 'point_cloud/cropped',
            'output_pointcloud_topic_name': 'point_cloud/sampled',
            'source_frame_id': 'radar',
            'target_frame_id': 'map',
        }],
    )

    # Return launch description
    return LaunchDescription([
        crop_box_filter_node,
        pointcloud_buffer_node,
        voxel_grid_filter_node,
        ground_plane_segmentation_node,
        pointcloud_to_laserscan_node,
        online_sync_slam_node,
        # octomap_server_node,
        pointcloud_transformer_node,
    ])