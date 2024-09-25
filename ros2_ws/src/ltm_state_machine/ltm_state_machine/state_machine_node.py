""" This is the main node for the ltm_state_machine package.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-23
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from time import sleep
import ltm_state_machine.states as states
import ltm_state_machine.utils as utils

from std_msgs.msg import String
from ltm_shared_msgs.msg import MissionState
from ltm_shared_msgs.srv import PerformState, GetStateTransitionHistory

class StateMachineNode(Node):
    """ This is the main node for the ltm_state_machine package."""
    
    def __init__(self) -> None:
        super().__init__('state_machine_node')
        self.get_logger().info('Initializing state machine node...')
        
        self.state = states.BootUp()
        self.history = [states.Undefined()]
        self.input = None

        self.configure_state_service_clients()
        self.configure_state_transition_history_service()
        self.configure_input_subscriber()
        self.configure_mission_state_publisher()

        self.get_logger().info('State machine node initialized.')
        self.get_logger().info(f'Current state: {self.state.name}')
        self.request_action()

    def run(self) -> None:
        """ Callback function for the timer that triggers the state machine."""
        self.get_logger().info('Timer triggered.')
        if self.request_action():
            self.update_history()
            self.update_state()
            self.publish_mission_state()
            self.get_logger().info(f'Transitioned to state: {self.state.name}')

    def future_callback(self, future) -> None:
        """ Callback function for the future after the state service has been called."""
        self.get_logger().info('Future callback triggered.')
        success = future.result().success

        # If the state service failed, transition to the error state
        if not success:
            self.get_logger().info('State service failed.')
            # TODO: Implement error state transition
            return
        
        # If the state machine has reached a terminal state, do not transition
        if self.state.is_terminal:
            self.get_logger().warn('State machine has reached a terminal state.')
            return

        # If the state service was successful, transition to the next state
        # Update the state transition history, update the current state, and request the next action
        self.update_history()
        self.update_state()
        self.publish_mission_state()
        self.get_logger().info(f'Transitioned to state: {self.state.name}')
        self.request_action()

    def input_callback(self, msg) -> None:
        """ Callback function for the input subscriber."""
        self.get_logger().info(f'Received input: {msg.data}')
        self.state.set_input(msg.data)

    def state_transition_history_callback(self, request, response) -> PerformState.Response:
        """ Callback function for the state transition history service."""
        _ = request
        self.get_logger().info('State transition history requested.')
        response.states = [state.name for state in self.history]
        response.ids = [state.id for state in self.history]
        return response

    def publish_mission_state(self) -> None:
        """ Publishes the current mission state to the mission state topic."""
        self.mission_state_publisher.publish(MissionState(state=self.state.id))

    def request_action(self) -> None:
        """ Trigger the current state to do its action."""
        perform_state_request = self.state.get_service_request()
        perform_state_request.first_time = self.state.id != self.history[-1].id
        perform_state_future = self.state_service_clients[self.state].call_async(perform_state_request)
        perform_state_future.add_done_callback(self.future_callback)

    def configure_input_subscriber(self) -> None:
        """ Configures the input subscriber."""
        self.input_subscriber_callback_group = MutuallyExclusiveCallbackGroup()
        self.input_subscriber = self.create_subscription(
            String, 'ltm/input', self.input_callback, 10, callback_group=self.input_subscriber_callback_group)

    def configure_mission_state_publisher(self) -> None:
        """ Configures the mission state publisher."""
        self.mission_state_publisher = self.create_publisher(
            MissionState, 'ltm/state', 1)
        
    def configure_state_service_clients(self) -> None:
        """ Configures the state service clients."""
        self.state_service_callback_group = {}
        self.state_service_clients = {}

        self.get_logger().info('Configuring state service clients...')
        for state in states.get_all_states():
            # Skip states that are not of interest
            if type(state) == states.Undefined:
                continue
            self.get_logger().info(f'Configuring state {state.name} with service name {state.get_service_name()}')

            self.state_service_callback_group[state] = MutuallyExclusiveCallbackGroup()
            self.state_service_clients[state] = self.create_client(
                PerformState, state.get_service_name(), 
                callback_group=self.state_service_callback_group[state])
            while not self.state_service_clients[state].wait_for_service():
                self.get_logger().info(f'Waiting for service {state.get_service_name()}...', throttle_duration_sec=1.0)
            self.get_logger().info(f'State service client for {state.name} configured.')
            
    def configure_state_transition_history_service(self) -> None:
        """ Configures the state transition history service."""
        self.state_transition_history_callback_group = MutuallyExclusiveCallbackGroup()
        self.state_transition_history_service = self.create_service(
            GetStateTransitionHistory, 'state_machine/state_transition_history', 
            self.state_transition_history_callback, callback_group=self.state_transition_history_callback_group)

    def update_history(self) -> None:
        """ Updates the state transition history."""
        self.history.append(self.state)
        self.history = utils.reduce_consecutive_duplicates(self.history)

    def update_state(self) -> None:
        """ Updates the current state."""
        self.state = self.state.transition()

def main():#
    rclpy.init()
    node = StateMachineNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
