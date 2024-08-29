""" This script is responsible for handling the state machine for the navigation stack.
Author: Alexander James Becoy
Revision: 1.0
Date: 28-08-2024
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid

class NavigationHandlerNode(Node):
    def __init__(self):
        super().__init__('navigation_handler_node')
        self.get_logger().info('Navigation Handler Node has been initialized.')
        
        self.map = None

        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.state_subscriber = self.create_subscription(Empty, 'state', self.state_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'buffered_map', 10)

    def __del__(self):
        self.get_logger().info('Navigation Handler Node has been terminated.')

    def map_callback(self, msg):
        self.map = msg

    def state_callback(self, msg):
        if self.map is not None:
            self.publish_map()
            self.get_logger().info('State message received, publishing the map of the environment.')
        else:
            self.get_logger().info('State message received, but the map is None')

    def publish_map(self):
        self.map_publisher.publish(self.map)


def main(args=None):
    rclpy.init(args=args)
    navigation_handler_node = NavigationHandlerNode()
    rclpy.spin(navigation_handler_node)
    navigation_handler_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
