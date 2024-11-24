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

    # Go2 driver launch file
    navigation_planner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_stack'), 
                'launch', 'go2_stack.launch.py')
        )
    )

    # Scan procedure launch file
    streaming_scan_node = Node(
        package='ltm_scanning_stream',
        executable='scanning_stream_node',
        name='scanning_stream_node',
        output='screen',
    )

    return LaunchDescription([
        # Launch arguments
        # Launch files
        navigation_planner_node,
        streaming_scan_node,
    ])