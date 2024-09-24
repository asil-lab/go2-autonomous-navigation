""" This is the main node for the ltm_navigation_state package.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-24
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from ltm_shared_msgs.srv import PerformState

INPUT_TOPIC_NAME = 'state_machine/input'
INPUT_TOPIC_QUEUE_SIZE = 1

CREATE_MAP_SERVICE_NAME = 'state_machine/create_map'

class NavigationStateHandlerNode(Node):
    
    def __init__(self) -> None:
        super().__init__('navigation_state_handler_node')
        self.get_logger().info('Initializing navigation state handler node...')

        self.input = None
        self.configure_create_map_service()
        self.configure_input_subscriber()

        self.get_logger().info('Navigation state handler node initialized.')

    def input_callback(self, msg) -> None:
        self.get_logger().info('Input received.')
        self.input = msg.data

    def create_map_callback(self, request, response) -> PerformState.Response:
        self.get_logger().info('Create map service called.')
        while self.input is None or self.input != 'stop':
            self.get_logger().info('Waiting to complete map creation...', throttle_duration_sec=5)
        response.success = True
        return response

    def configure_input_subscriber(self) -> None:
        self.input_callback_group = MutuallyExclusiveCallbackGroup()
        self.input_subscriber = self.create_subscription(
            String, INPUT_TOPIC_NAME, self.input_callback, 
            INPUT_TOPIC_QUEUE_SIZE, callback_group=self.input_callback_group)

    def configure_create_map_service(self) -> None:
        self.create_map_callback_group = MutuallyExclusiveCallbackGroup()
        self.create_map_service = self.create_service(
            PerformState, CREATE_MAP_SERVICE_NAME, 
            self.create_map_callback, callback_group=self.create_map_callback_group)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = NavigationStateHandlerNode()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
