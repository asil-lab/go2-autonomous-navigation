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

    # RViz with Go2
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_go2_description'), 
                'launch', 'go2_rviz.launch.py')
        ),
    )

    # URDF file location
    urdf_location = os.path.join(
        get_package_share_directory("ltm_go2_description"), "urdf", "go2_description.urdf")
    
    # Read the URDF file content
    with open(urdf_location, 'r') as infp:
        robot_description = infp.read()

    # Go2 driver node
    go2_driver_node = Node(
        package='ltm_hardware_interface',
        executable='hardware_interface_node',
        name='hardware_interface_node',
        output='screen',
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        arguments=[urdf_location],
    )

    return LaunchDescription([
        go2_driver_node,
        robot_state_publisher_node,
        rviz_node,
    ])