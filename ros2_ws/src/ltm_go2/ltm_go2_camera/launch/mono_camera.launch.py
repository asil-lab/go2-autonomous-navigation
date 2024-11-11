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

    go2_flag_argument = DeclareLaunchArgument(
        'go2',
        default_value='false',
        description='Flag to indicate if the script is running on the Go2',
    )

    # Get the parameters
    if LaunchConfiguration('go2'):
        parameters_filename = 'monocamera_go2_params.yaml'
    else:
        parameters_filename = 'monocamera_ext_params.yaml'

    config_directory = os.path.join(
        get_package_share_directory('ltm_go2_camera'), 'config')
    config_filepath = os.path.join(config_directory, parameters_filename)

    # Go2 Camera node
    ltm_go2_camera_node = Node(
        package='ltm_go2_camera',
        executable='go2_camera_node',
        name='go2_camera_node',
        output='screen',
        parameters=[config_filepath]
    )

    return LaunchDescription([
        go2_flag_argument,
        ltm_go2_camera_node,
    ])