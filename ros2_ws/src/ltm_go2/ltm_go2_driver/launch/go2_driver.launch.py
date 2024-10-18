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

    #Declare the launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Flag to indicate if RViz should be launched'
        ),
        DeclareLaunchArgument(
            'go2',
            default_value='false',
            description='Flag to indicate if the script is running on the Go2'
        )
    ]

    # URDF file location
    urdf_location = os.path.join(
        get_package_share_directory("ltm_go2_description"), "urdf", "go2_description.urdf")

    # Read the URDF file content
    with open(urdf_location, 'r') as infp:
        robot_description = infp.read()

    # Go2 driver node
    go2_driver_node = Node(
        package='ltm_go2_driver',
        executable='go2_driver_node',
        output='screen',
    )

    # Go2 camera node
    go2_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_go2_camera'), 
                'launch', 'camera.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('go2')),
    )

    # D435i camera node
    d435i_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_go2_camera'),
                'launch', 'd435i_camera.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('go2')),
    )

    # Go2 state handler node
    go2_state_handler_node = Node(
        package='ltm_go2_state',
        executable='go2_state_handler_node',
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

    # RViz with Go2

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ltm_go2_description'), 
                'launch', 'go2_rviz.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('go2')),
    )

    return LaunchDescription(declared_arguments + [
        go2_driver_node,
        go2_camera_node,
        d435i_camera_node,
        go2_state_handler_node,
        robot_state_publisher_node,
        rviz_node,
    ])
