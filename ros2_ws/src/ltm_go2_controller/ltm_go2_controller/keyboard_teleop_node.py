""" This is the main node for the ltm_state_machine package.
Author: Alexander James Becoy
Revision: 1.0
Date: 07-08-2024
"""

import rclpy
from rclpy.node import Node

from unitree_go.msg import WirelessController

import sys, select, termios, tty

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        
        self.settings = termios.tcgetattr(sys.stdin)
        msg = """
        Reading from the keyboard and Publishing to Wireless Controller!
        ---------------------------
        W: Forward
        S: Backward
        A: Strafe Left
        D: Strafe Right
        Q: Turn Left
        E: Turn Right
        ---------------------------
        """
        self.get_logger().info(msg)
        
        self.max_linear_x = 0.1
        self.max_linear_y = 0.1
        self.max_angular_z = 0.1
        self.key_mapping = {    # (lx, ly, rx, ry)
            'a': (-self.max_linear_y, 0.0, 0.0, 0.0),
            'd': (self.max_linear_y, 0.0, 0.0, 0.0),
            'w': (0.0, self.max_linear_x, 0.0, 0.0),
            's': (0.0, -self.max_linear_x, 0.0, 0.0),
            'q': (0.0, 0.0, -self.max_angular_z, 0.0),
            'e': (0.0, 0.0, self.max_angular_z, 0.0),
        }
        
        self.controller_msg = WirelessController()
        self.controller_pub = self.create_publisher(WirelessController, 'wirelesscontroller', 10)
        
        self.get_logger().info('Keyboard Teleop Node has been initialized.')

        self.key = None
        self.done = False
        self.get_logger().info('Press Ctrl+C to exit...')
    
    def publish_controller_msg(self):
        self.controller_pub.publish(self.controller_msg)
        
    def reset_controller_msg(self):
        self.controller_msg.lx = 0.0
        self.controller_msg.ly = 0.0
        self.controller_msg.rx = 0.0
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
    def run(self):
        while not self.done:
            # Get the key
            self.get_key()
            
            # Check if the key is in the key mapping
            if self.key in self.key_mapping.keys():
                lx, ly, rx, ry = self.key_mapping[self.key]
                self.controller_msg.lx = lx
                self.controller_msg.ly = ly
                self.controller_msg.rx = rx
                self.controller_msg.ry = ry
            else:
                if self.key == '\x03':
                    self.done = True
                self.reset_controller_msg()
                
            # Publish the controller message
            self.publish_controller_msg()
            self.get_logger().info('Publishing Controller Message: {self.controller_msg}')
    
    def destroy_node(self):
        self.reset_controller_msg()
        self.publish_controller_msg()
        self.get_logger().info('Destroying Node...')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()
    

def main(args=None):
    rclpy.init(args=args)
    keyboard_teleop_node = KeyboardTeleopNode()
    try:
        keyboard_teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    