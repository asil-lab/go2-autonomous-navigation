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
        
        self.max_linear_x = 0.2
        self.max_linear_y = 0.2
        self.max_angular_z = 0.5
        
        self.controller_msg = WirelessController()
        self.controller_pub = self.create_publisher(WirelessController, 'wireless_controller', 10)
        
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
            self.get_key()
            if self.key == 'w':
                self.controller_msg.lx = self.max_linear_x
            elif self.key == 's':
                self.controller_msg.lx = -self.max_linear_x
            elif self.key == 'a':
                self.controller_msg.ly = self.max_linear_y
            elif self.key == 'd':
                self.controller_msg.ly = -self.max_linear_y
            elif self.key == 'q':
                self.controller_msg.rx = self.max_angular_z
            elif self.key == 'e':
                self.controller_msg.rx = -self.max_angular_z
            else:
                self.reset_controller_msg()
            self.publish_controller_msg()
            
            if self.key == '\x03':
                self.done = True
                self.reset_controller_msg()
                self.publish_controller_msg()
                self.get_logger().info('Exiting...')
                break
            
            self.get_logger().info('Publishing Wireless Controller Message: {}'.format(self.controller_msg))
    
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
    