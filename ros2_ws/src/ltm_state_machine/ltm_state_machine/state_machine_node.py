""" This is the main node for the ltm_state_machine package.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-07-28
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import ltm_state_machine.states as states

from std_msgs.msg import String
from ltm_shared_msgs.msg import MissionState
from ltm_shared_msgs.srv import PerformState

class StateMachineNode(Node):
    """ This is the main node for the ltm_state_machine package."""
    
    def __init__(self) -> None:
        super().__init__('state_machine_node')
        self.get_logger().info('Initializing state machine node...')
        
        self.state = states.BootUp()
        self.history = []
        self.input = None

        self.mission_state_publisher = self.create_publisher(MissionState, 'ltm/mission_state', 10)
        
        self.get_logger().info('State machine node initialized.')

        self.configure_timer()
        self.configure_input_subscriber()
        self.configure_mission_state_publisher()
        self.configure_state_service_clients()

    def timer_callback(self) -> None:
        """ Callback function for the timer that triggers the state machine."""
        if self.request_action():
            self.state = self.state.transition()
            self.history.append(self.state)
            self.publish_mission_state()

    def input_callback(self, msg) -> None:
        """ Callback function for the input subscriber."""
        self.get_logger().info(f'Received input: {msg.data}')
        self.state.set_input(msg.data)

    def state_transition_history_callback(self, request, response) -> PerformState.Response:
        """ Callback function for the state transition history service."""
        _ = request
        response.states = [state.name for state in self.history]
        response.id = [state.id for state in self.history]
        return response

    def publish_mission_state(self) -> None:
        """ Publishes the current mission state to the mission state topic."""
        self.mission_state_publisher.publish(self.state.id)

    def request_action(self) -> bool:
        """ Trigger the current state to do its action."""
        perform_state_request = self.state.get_service_request()
        perform_state_future = self.state_service_clients[self.state].call_async(perform_state_request)
        rclpy.spin_until_future_complete(self, perform_state_future)
        return perform_state_future.result().success

    def configure_timer(self) -> None:
        """ Configures the timer."""
        self.timer = self.create_timer(1, self.timer_callback)

    def configure_input_subscriber(self) -> None:
        """ Configures the input subscriber."""
        self.input_subscriber = self.create_subscription(
            String, 'ltm/input', self.input_callback, 10)

    def configure_mission_state_publisher(self) -> None:
        """ Configures the mission state publisher."""
        self.mission_state_publisher = self.create_publisher(
            MissionState, 'ltm/state', 1)
        
    def configure_state_service_clients(self) -> None:
        """ Configures the state service clients."""
        self.state_service_callback_group = {}
        self.state_service_clients = {}

        for state in states.get_all_states():
            self.state_service_callback_group[state] = MutuallyExclusiveCallbackGroup()
            self.state_service_clients[state] = self.create_client(
                PerformState, state.get_service_name(), 
                callback_group=self.state_service_callback_group[state])
            
    def configure_state_transition_history_service(self) -> None:
        """ Configures the state transition history service."""
        self.state_transition_history_service = self.create_service(
            PerformState, 'state_machine/state_transition_history', self.state_transition_history_callback)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = StateMachineNode()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
