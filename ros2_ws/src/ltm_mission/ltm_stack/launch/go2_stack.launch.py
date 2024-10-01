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
    mapping_argument = DeclareLaunchArgument(
        'mapping',
        default_value='true',
        description='Enable mapping. If false, only localization is performed.'
    )

    navigation_argument = DeclareLaunchArgument(
        'navigation',
        default_value='true',
        description='Enable navigation. Default to true.'
    )

    # Go2 driver launch file
    go2_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_go2_driver'), 
                'launch', 'go2_driver.launch.py')
        )
    )

    # Exploration core launch file
    exploration_core_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_exploration_core'), 
                'launch', 'exploration.launch.py')
        ),
        launch_arguments=[('mapping', LaunchConfiguration('mapping'))],
    )

    # Navigation core launch file
    navigation_core_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_navigation_core'), 
                'launch', 'navigation.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('navigation')),
    )

    return LaunchDescription([
        # Launch arguments
        mapping_argument,
        navigation_argument,
        # Launch files
        go2_driver_node,
        exploration_core_node,
        navigation_core_node,
    ])