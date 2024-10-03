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

    # ROS2 nav2_bringup launch file
    nav2_params_filename = 'nav2_params.yaml'
    nav2_params_filepath = os.path.join(
        get_package_share_directory('ltm_navigation_core'), 'config', nav2_params_filename)
    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments=[('params_file', nav2_params_filepath)]
    )

    # LTM Navigation Planner Node
    navigation_planner_node = Node(
        package='ltm_navigation_planner',
        executable='navigation_planner_node',
        name='navigation_planner_node',
        output='screen',
    )

    # LTM Navigation Service Node
    navigation_service_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_navigation_service'), 
                         'launch', 'navigation_service.launch.py')
        )
    )

    return LaunchDescription([
        navigation_node,
        navigation_planner_node,
        navigation_service_node,
    ])