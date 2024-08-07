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
    
    # Get the path to the parameters file
    config_dir = os.path.join(get_package_share_directory('ltm_pointcloud_buffer'), 'config')
    config_params = os.path.join(config_dir, 'parameters.yaml')
    
    # Create the launch description and start the pointcloud_buffer_node
    return LaunchDescription([
        Node(
            package='ltm_pointcloud_buffer',
            executable='pointcloud_buffer_node',
            name='pointcloud_buffer_node',
            output='screen',
            parameters=[config_params],
        )
    ])