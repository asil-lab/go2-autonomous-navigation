""" This is a node to mock up the state services.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-26
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from slam_toolbox.srv import SaveMap

class LMapCreatorNode(Node):

    def __init__(self) -> None:
        super().__init__('lmap_creator_node')
        self.get_logger().info('Initializing lmap creator node...')
        self.configure_save_map_client()
        self.configure_input_subscriber()
        self.get_logger().info('LMap creator node initialized.')

    def __del__(self) -> None:
        self.save_map_client.destroy()
        self.destroy_node()

    def input_callback(self, msg) -> None:
        """ Callback function for the input subscriber."""
        self.get_logger().info('Input received.')
        self.save_map()

    def save_map(self) -> None:
        """ Send a request to save the map."""
        request = SaveMap.Request()
        future = self.save_map_client.call_async(request)
        self.get_logger().info('Saving map...')

    def configure_input_subscriber(self) -> None:
        """ Configures the input subscriber."""
        self.input_subscriber = self.create_subscription(
            Empty, 'input', self.input_callback, 1)
        self.get_logger().info('Input subscriber configured.')

    def configure_save_map_client(self) -> None:
        """ Configures the save map client."""
        self.save_map_client = self.create_client(SaveMap, 'save_map')
        self.get_logger().info('Save map client configured.')

def main():
    rclpy.init()
    node = LMapCreatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
