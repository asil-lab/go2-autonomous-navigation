"""
Lava Tube Mapping Project - LTM Go2 Description Package
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-08-01
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('ltm_go2_description'), 'rviz', 'go2.rviz')],
        ),
    ])