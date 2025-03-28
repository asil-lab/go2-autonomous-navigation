""" This is the main node for the ltm_state_machine package.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-09-23
"""

import rclpy
import ltm_state_machine.states as states

def main(args=None):
    rclpy.init(args=args)

    # Initialize the state machine at state LoadMap
    state_node = states.LoadMapState()
    
    # Determine if the state has error
    if state_node.is_error():
        state_node.print_error()
        state_node.get_logger().warn('Terminating the state machine.')
        rclpy.shutdown()
        return

    # Continue to transition until the state is terminal
    while not state_node.is_terminal():
        state_node = state_node.transition()

    # Shutdown the node
    state_node.get_logger().warn('State machine has been terminated.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
