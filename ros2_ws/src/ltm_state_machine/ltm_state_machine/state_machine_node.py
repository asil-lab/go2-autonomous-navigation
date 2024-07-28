""" This is the main node for the ltm_state_machine package.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-07-28
"""

import rclpy
from rclpy.node import Node

from ltm_shared_msgs.msg import MissionState

class StateMachineNode(Node):
    
    def __init__(self):
        super().__init__('state_machine_node')
        self.get_logger().info('Initializing state machine node...')
        # self.declare_parameter('mission_state', MissionState())
        # self.mission_state = self.get_parameter('mission_state').value
        # self.mission_state_subscriber = self.create_subscription(MissionState, 'mission_state', self.mission_state_callback, 10)
        self.mission_state_publisher = self.create_publisher(MissionState, 'ltm/mission_state', 10)
        self.get_logger().info('State machine node initialized.')


def main():
    rclpy.init()
    state_machine_node = StateMachineNode()
    rclpy.spin(state_machine_node)
    state_machine_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
