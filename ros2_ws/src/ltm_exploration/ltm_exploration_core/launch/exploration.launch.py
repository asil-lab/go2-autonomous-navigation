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
from launch_ros.actions import Node

def generate_launch_description():
    
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

    # Online asynchronous SLAM node
    online_async_slam_config_filename = 'mapper_params_online_async.yaml'
    online_async_slam_config_filepath = os.path.join(
        get_package_share_directory('ltm_exploration_core'), 'config', online_async_slam_config_filename)
    online_async_slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                            'launch', 'online_async_launch.py')
        ),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('params_file', online_async_slam_config_filepath),
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        pointcloud_buffer_node,
        pointcloud_filter_node,
        pointcloud_to_laserscan_node,
        online_async_slam_node,
    ])