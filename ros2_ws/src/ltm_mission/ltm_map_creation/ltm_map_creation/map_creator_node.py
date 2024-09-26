""" This is a node to mock up the state services.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-26
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String
from slam_toolbox.srv import SaveMap

import os
import datetime
import shutil

class LMapCreatorNode(Node):

    def __init__(self) -> None:
        super().__init__('map_creator_node')
        self.get_logger().info('Initializing map creator node...')
        self.map_name = None
        self.configure_save_map_client()
        self.configure_input_subscriber()
        self.get_logger().info('Map creator node initialized.')

    def __del__(self) -> None:
        self.get_logger().warn('Stopping map creator...')
        self.save_map_client.destroy()
        self.destroy_node()
        rclpy.shutdown()

    def input_callback(self, msg) -> None:
        """ Callback function for the input subscriber."""
        self.get_logger().info(f'Input received. Saving map as {msg.data}.')
        self.map_name = msg.data
        self.save_map(msg)

    def save_map(self, name: String) -> None:
        """ Send a request to save the map."""
        request = SaveMap.Request()
        request.name = name
        future = self.save_map_client.call_async(request)
        future.add_done_callback(self.move_map)
        self.get_logger().info('Saving map...')

    def move_map(self, future) -> None:
        """ Move the map to directory src/ltm_mission/maps/current_datetime/."""
        self.get_logger().info('Future received.')
        current_datetime = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        # Create the maps directory if it does not exist
        maps_directory = os.path.join(os.environ.get('LTM_ROS2_WS'), 'src', 'ltm_mission', 'maps')
        if not os.path.exists(maps_directory):
            os.makedirs(maps_directory)
            self.get_logger().info(f'Directory {maps_directory} created.')

        # Create the directory if it does not exist
        directory = os.path.join(maps_directory, current_datetime)
        if not os.path.exists(directory):
            os.makedirs(directory)
            self.get_logger().info(f'Directory {directory} created.')

        self.get_logger().info(f'Moving map {self.map_name} to {directory}')
        self.get_logger().warn('Stopping map creator...')
        rclpy.shutdown()

        # Get the original filepaths of the newly saved map
        pgm_filepath = os.path.join(os.environ.get('LTM_ROS2_WS'), f'{self.map_name}.pgm')
        yaml_filepath = os.path.join(os.environ.get('LTM_ROS2_WS'), f'{self.map_name}.yaml')

        # Move the map to the desired directory
        shutil.move(pgm_filepath, os.path.join(directory, f'{self.map_name}.pgm'))
        shutil.move(yaml_filepath, os.path.join(directory, f'{self.map_name}.yaml'))

        self.get_logger().info(f'Map {self.map_name} moved to {directory}.')

    def configure_input_subscriber(self) -> None:
        """ Configures the input subscriber."""
        self.input_callback_group = MutuallyExclusiveCallbackGroup()
        self.input_subscriber = self.create_subscription(
            String, 'input', self.input_callback, 1, callback_group=self.input_callback_group)
        self.get_logger().info('Input subscriber configured.')

    def configure_save_map_client(self) -> None:
        """ Configures the save map client."""
        self.save_map_callback_group = MutuallyExclusiveCallbackGroup()
        self.save_map_client = self.create_client(
            SaveMap, 'slam_toolbox/save_map', callback_group=self.save_map_callback_group)
        # TODO: Wait until servvice ready            
        self.get_logger().info('Save map client configured.')


def main():
    rclpy.init()
    node = LMapCreatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
