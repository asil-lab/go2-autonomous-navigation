"""
Project Lava Tube Mapping, Technical University of Delft.
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare the launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'in_simulation',
            default_value='true',
            description='Flag to indicate if the system is running in simulation mode'
        ),
    ]
    
    # Get the launch configuration variables
    in_simulation = LaunchConfiguration('in_simulation', default='true')
    
    # Get the path to the parameters file
    config_dir = os.path.join(get_package_share_directory('ltm_pointcloud_filter'), 'config')
    config_params = os.path.join(config_dir, 'parameters.yaml')

    # Create the launch description and start the ik_test_node
    return LaunchDescription(declared_arguments + [
        Node(
            package='ltm_pointcloud_filter',
            executable='pointcloud_filter_node',
            name='pointcloud_filter_node',
            output='screen',
            parameters=[
                config_params,
                {'in_simulation': in_simulation},
            ],
        )
    ])