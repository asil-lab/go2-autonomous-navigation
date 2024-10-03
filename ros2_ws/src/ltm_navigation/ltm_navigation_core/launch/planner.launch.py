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

    # LTM Navigation Planner Node
    navigation_planner_node = Node(
        package='ltm_navigation_planner',
        executable='navigation_planner_node',
        name='navigation_planner_node',
        output='screen',
    )

    # LTM Navigation Visualizer Node
    navigation_visualizer_node = Node(
        package='ltm_navigation_visualizer',
        executable='navigation_visualizer_node',
        name='navigation_visualizer_node',
        output='screen',
    )

    return LaunchDescription([
        navigation_planner_node,
        navigation_visualizer_node,
    ])