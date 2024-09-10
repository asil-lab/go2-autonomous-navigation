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

    # Obtain the parameters file path
    navigation_service_config_directory = os.path.join(
        get_package_share_directory('ltm_navigation_service'), 'config')
    navigation_service_config_filename = 'parameters.yaml'
    navigation_service_config_filepath = os.path.join(
        navigation_service_config_directory, navigation_service_config_filename)

    # LTM Navigation Service Node
    navigation_service_node = Node(
        package='ltm_navigation_service',
        executable='navigation_service_node',
        name='navigation_service_node',
        output='screen',
        parameters=[navigation_service_config_filepath]
    )

    return LaunchDescription([
        navigation_service_node,
    ])
