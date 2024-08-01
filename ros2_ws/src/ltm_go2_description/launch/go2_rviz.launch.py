"""
Lava Tube Mapping Project - LTM Go2 Description Package
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-08-01
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    declared_arguments = [
        DeclareLaunchArgument(
            name="use_sim_true",
            default_value="false",
            description="Use simulation time",
            choices=["true", "false"],
        ),
    ]
    
    use_sim_time = LaunchConfiguration("use_sim_true", default="false")
    
    # Get the URDF file location
    urdf_location = os.path.join(
        get_package_share_directory("ltm_go2_description"), "urdf", "go2_description.urdf")
    
    # Read the URDF file content
    with open(urdf_location, 'r') as infp:
        robot_description = infp.read()
    
    return LaunchDescription(declared_arguments + [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time, 
                'robot_description': robot_description
            }],
            arguments=[urdf_location],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('ltm_go2_description'), 'rviz', 'go2.rviz')],
        ),
    ])