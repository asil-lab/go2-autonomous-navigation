"""
Project Lava Tube Mapping, Technical University of Delft.
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Obtain the parameters file path
    scan_procedure_config_directory = os.path.join(
        get_package_share_directory('ltm_scan_procedure'), 'config')
    scan_procedure_config_filename = 'parameters.yaml'
    scan_procedure_config_filepath = os.path.join(
        scan_procedure_config_directory, scan_procedure_config_filename)
    
    # LTM Scan Procedure Node
    scan_procedure_node = Node(
        package='ltm_scan_procedure',
        executable='scan_procedure_node',
        name='scan_procedure_node',
        output='screen',
        parameters=[scan_procedure_config_filepath],
    )

    # LTM Auxiliary Sensing Node
    auxiliary_sensors_node = Node(
        package='ltm_go2_auxiliary_sensors',
        executable='auxiliary_sensors_node',
        name='auxiliary_sensors_node',
        output='screen',
    )

    return LaunchDescription([
        scan_procedure_node,
        # auxiliary_sensors_node,
    ])
