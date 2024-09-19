"""
Project Lava Tube Mapping, Technical University of Delft.
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the parameters
    config_direrctory = os.path.join(
        get_package_share_directory('ltm_go2_camera'), 'config')
    config_filepath = os.path.join(config_direrctory, 'parameters.yaml')

    # Go2 Camera node
    ltm_go2_camera_node = Node(
        package='ltm_go2_camera',
        executable='go2_camera_node',
        name='go2_camera_node',
        output='screen',
        parameters=[config_filepath]
    )

    return LaunchDescription([
        ltm_go2_camera_node
    ])