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

    # LTM stack launch file
    ltm_stack_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_stack'), 
                'launch', 'go2_stack.launch.py')
        ),
        launch_arguments=[('mapping', 'true')]
    )

    # # Map creator node
    # map_creator_node = Node(
    #     package='ltm_map_creation',
    #     executable='map_creator_node',
    #     name='map_creator_node',
    #     output='screen'
    # )

    return LaunchDescription([
        ltm_stack_node,
        # map_creator_node,
    ])