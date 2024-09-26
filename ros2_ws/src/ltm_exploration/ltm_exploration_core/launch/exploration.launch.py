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

def generate_launch_description():
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'mapping',
            default_value='true',
            description='Enable mapping. If false, only localization is performed.'
        ),
    ]

    # Pointcloud buffer node
    pointcloud_buffer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_pointcloud_buffer'), 
                'launch', 'pointcloud_buffer.launch.py')
        )
    )
    
    # Pointcloud filter node
    pointcloud_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_pointcloud_filter'), 
                'launch', 'pointcloud_filter.launch.py')
        ),
        launch_arguments=[('in_simulation', 'false')]
    )
    
    # Pointcloud-to-laserscan node
    # pointcloud_to_laserscan_config = os.path.join(
    #     get_package_share_directory('ltm_exploration_core'), 'config', 'parameters.yaml')
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[('cloud_in', 'point_cloud/filtered')],
        # parameters=[pointcloud_to_laserscan_config],
    )

    # Online synchronous SLAM node
    # Launch only if mapping is enabled
    online_sync_slam_config_filename = 'mapper_params_online_sync.yaml'
    online_sync_slam_config_filepath = os.path.join(
        get_package_share_directory('ltm_exploration_core'), 'config', online_sync_slam_config_filename)
    
    online_sync_slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                            'launch', 'online_sync_launch.py')
        ),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('params_file', online_sync_slam_config_filepath),
        ],
        condition=IfCondition(LaunchConfiguration('mapping')),
    )

    # Localization SLAM node
    # Launch only if mapping is disabled
    localization_slam_config_filename = 'mapper_params_localization.yaml'
    localization_slam_config_filepath = os.path.join(
        get_package_share_directory('ltm_exploration_core'), 'config', localization_slam_config_filename)

    localization_slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                            'launch', 'localization_launch.py')
        ),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('params_file', localization_slam_config_filepath),
        ],
        condition=UnlessCondition(LaunchConfiguration('mapping')),
    )
    
    # Return launch description
    return LaunchDescription(declared_arguments + [
        pointcloud_buffer_node,
        pointcloud_filter_node,
        pointcloud_to_laserscan_node,
        online_sync_slam_node,
        localization_slam_node,
    ])