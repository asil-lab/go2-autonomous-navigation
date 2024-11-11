"""
Project Lava Tube Mapping, Technical University of Delft.
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    go2_flag_argument = DeclareLaunchArgument(
        'go2',
        default_value='false',
        description='Flag to indicate if the script is running on the Go2',
        choices=['true', 'false'],
    )

    # Get the parameters
    if LaunchConfiguration('go2').perform(None).lower() == 'true':
        multicast_iface_address = 'eth0'
    else:
        multicast_iface_address = 'wlp2s0'
    config_directory = os.path.join(get_package_share_directory('ltm_go2_camera'), 'config')
    config_filepath = os.path.join(config_directory, 'monocamera_params.yaml')

    # Go2 Camera node
    ltm_go2_camera_node = Node(
        package='ltm_go2_camera',
        executable='go2_camera_node',
        name='go2_camera_node',
        output='screen',
        parameters=[config_filepath, {'camera_stream.multicast_iface': multicast_iface_address}]
    )

    return LaunchDescription([
        go2_flag_argument,
        ltm_go2_camera_node,
    ])