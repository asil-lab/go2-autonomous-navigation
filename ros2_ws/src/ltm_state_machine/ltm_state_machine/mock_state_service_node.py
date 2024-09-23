""" This is a node to mock up the state services.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-23
"""

import rclpy
from rclpy.node import Node

from ltm_state_machine.states import get_all_states, get_state_by_id

from ltm_shared_msgs.srv import PerformState

class MockStateServiceNode(Node):

    def __init__(self) -> None:
        super().__init__('mock_state_service_node')
        self.get_logger().info('Initializing mock state service node...')
        self.configure_state_service()
        self.get_logger().info('Mock state service node initialized.')

    def state_service_callback(self, request, response) -> PerformState.Response:
        """ Callback function for the state services."""
        self.get_logger().info(f'State service {get_state_by_id(request.current_state)} called.')
        self.get_logger().info(f'First time: {request.first_time}')
        response.success = True
        return response

    def configure_state_service(self) -> None:
        """ Configures the state service server for each state."""
        self.state_services = {}
        for state in get_all_states():
            self.state_services[state.name] = self.create_service(PerformState, state.get_service_name(), self.state_service_callback)
            self.get_logger().info(f'State service for {state.name} configured.')


def main(args=None):
    rclpy.init(args=args)
    node = MockStateServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()