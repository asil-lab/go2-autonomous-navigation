""" This node reads and publishes data from the auxiliary sensors of the LTM Go2 robot.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-11-04
"""

import rclpy
from rclpy.node import Node

from ltm_shared_msgs.msg import AuxiliarySensorState

from serial import Serial, SerialException
import threading
import struct

class AuxiliarySensorsNode(Node):

    def __init__(self):
        super().__init__('auxiliary_sensors_node')

        self.initialize_serial_port()
        self.initialize_auxiliary_sensor_state_publisher()
        self.initialize_thread()

        self.get_logger().info('Auxiliary sensors node initialized')

    def read_serial_port(self):
        while rclpy.ok():
            if self.serial_port.in_waiting == 0:
                continue
            
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f'Received line: {line}')

                # Split the lines given the delimiter " "
                data = line.split(' ')

                # Check if the line is valid
                if len(data) != 3:
                    self.get_logger().error('Invalid line format')
                    continue

                # Convert each data to uint32 in their IEEE 754 floating point representation
                # for i in range(3):
                #     data[i] = struct.unpack('f', struct.pack('I', int(data[i], 32)))[0]

                # Publish the message
                msg = AuxiliarySensorState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.temperature = data[0]
                msg.humidity = data[1]
                msg.luminosity = data[2]
                self.auxiliary_sensor_state_publisher.publish(msg)

            except UnicodeDecodeError as e:
                self.get_logger().error('Failed to decode serial line: %s', str(e))
                continue

    def initialize_serial_port(self):
        try:
            self.serial_port = Serial('/dev/ttyACM0', 115200, timeout=5.0)
            self.get_logger().info('Opened serial port')
        except SerialException as e:
            self.get_logger().error('Failed to open serial port: %s', str(e))
            rclpy.shutdown()

    def initialize_auxiliary_sensor_state_publisher(self):
        self.auxiliary_sensor_state_publisher = self.create_publisher(
            AuxiliarySensorState, 'auxiliary_sensor_state', 10)
        
    def initialize_thread(self):
        self.thread = threading.Thread(target=self.read_serial_port)
        self.thread.daemon = True
        self.thread.start()
        

def main(args=None):
    rclpy.init(args=args)
    auxiliary_sensors_node = AuxiliarySensorsNode()
    try:
        rclpy.spin(auxiliary_sensors_node)
    except KeyboardInterrupt:
        pass
    finally:
        auxiliary_sensors_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
