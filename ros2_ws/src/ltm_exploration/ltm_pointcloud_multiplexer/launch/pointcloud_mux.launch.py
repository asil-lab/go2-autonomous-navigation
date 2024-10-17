"""
Project Lava Tube Mapping, Technical University of Delft.
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the path to the parameters file
    config_dir = os.path.join(get_package_share_directory('ltm_pointcloud_multiplexer'), 'config')
    config_params = os.path.join(config_dir, 'parameters.yaml')

    # Create the launch description and start the pointcloud_mux_node
    return LaunchDescription([
        Node(
            package='ltm_pointcloud_multiplexer',
            executable='pointcloud_mux_node',
            name='pointcloud_mux_node',
            output='screen',
            parameters=[config_params],
        )
    ])
