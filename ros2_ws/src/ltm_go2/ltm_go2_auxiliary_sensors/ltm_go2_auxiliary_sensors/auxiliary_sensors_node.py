""" This node reads and publishes data from the auxiliary sensors of the LTM Go2 robot.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-11-04
"""

import rclpy
from rclpy.node import Node

# from ltm_shared_msgs.msg import AuxiliarySensorState

from serial import Serial, SerialException

class AuxiliarySensorsNode(Node):

    def __init__(self):
        super().__init__('auxiliary_sensors_node')

        self.initialize_serial_port()
        # self.initialize_auxiliary_sensor_state_publisher()
        self.initialize_timer()

        self.get_logger().info('Auxiliary sensors node initialized')

    def timer_callback(self):
        if self.serial_port.in_waiting == 0:
            return
        
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f'Received line: {line}')
        except UnicodeDecodeError as e:
            self.get_logger().error('Failed to decode serial line: %s', str(e))
            return

    def initialize_serial_port(self):
        try:
            self.serial_port = Serial('/dev/ttyACM0', 115200, timeout=5.0)
            self.get_logger().info('Opened serial port')
        except SerialException as e:
            self.get_logger().error('Failed to open serial port: %s', str(e))
            rclpy.shutdown()

    # def initialize_auxiliary_sensor_state_publisher(self):
    #     self.auxiliary_sensor_state_publisher = self.create_publisher(
    #         AuxiliarySensorState, 'auxiliary_sensor_state', 10)
        
    def initialize_timer(self):
        self.timer = self.create_timer(0.1, self.timer_callback)
        

def main(args=None):
    rclpy.init(args=args)
    auxiliary_sensors_node = AuxiliarySensorsNode()
    rclpy.spin(auxiliary_sensors_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
