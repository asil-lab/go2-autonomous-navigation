""" This is a node to mock up the state services.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-23
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ltm_state_machine.states import get_all_states, get_state_by_id

from std_msgs.msg import Empty
from ltm_shared_msgs.srv import PerformState

class MockStateServiceNode(Node):

    def __init__(self) -> None:
        super().__init__('mock_state_service_node')
        self.get_logger().info('Initializing mock state service node...')
        self.is_triggered = False
        self.configure_trigger_subscriber()
        self.configure_state_service()
        self.get_logger().info('Mock state service node initialized.')

    def trigger_callback(self, msg) -> None:
        """ Callback function for the trigger subscriber."""
        self.get_logger().info('Trigger received.')
        self.is_triggered = True

    def state_service_callback(self, request, response) -> PerformState.Response:
        """ Callback function for the state services."""
        self.get_logger().info(f'State service {get_state_by_id(request.current_state)} called.')
        self.get_logger().info(f'First time: {request.first_time}')
        while not self.is_triggered:
            self.get_logger().info('Waiting for trigger...', throttle_duration_sec=5)
        response.success = True
        self.is_triggered = False
        return response

    def configure_trigger_subscriber(self) -> None:
        """ Configures the trigger subscriber."""
        self.trigger_callback_group = MutuallyExclusiveCallbackGroup()
        self.trigger_subscriber = self.create_subscription(
            Empty, 'trigger', self.trigger_callback, 10, callback_group=self.trigger_callback_group)
        self.get_logger().info('Trigger subscriber configured.')

    def configure_state_service(self) -> None:
        """ Configures the state service server for each state."""
        self.state_services = {}
        self.state_service_callback_groups = {}
        for state in get_all_states():
            self.state_service_callback_groups[state.name] = MutuallyExclusiveCallbackGroup()
            self.state_services[state.name] = self.create_service(
                PerformState, state.get_service_name(), self.state_service_callback, callback_group=self.state_service_callback_groups[state.name])
            self.get_logger().info(f'State service for {state.name} configured.')


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = MockStateServiceNode()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()