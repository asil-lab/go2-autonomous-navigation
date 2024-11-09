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
from codecs import decode
from enum import Enum

BYTE_SIZE   = 8     # bits
FLOAT_SIZE  = 4     # bytes
NUM_SENSORS = 3     # temperature, humidity, light

class MISO(Enum):
    TEMPERATURE_OFFSET  = 0
    HUMIDITY_OFFSET     = 4 * BYTE_SIZE
    LIGHT_OFFSET        = 8 * BYTE_SIZE

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

                # Split the line into a list of strings per 4 bytes
                data = [line[offset.value:offset.value+(FLOAT_SIZE*BYTE_SIZE)] for offset in MISO]
                for value in data:
                    self.get_logger().info(f'Value: {value}')

                # Reverse the bits of each sensor value in the list
                for i in range(NUM_SENSORS):
                    data[i] = data[i][::-1]
                    self.get_logger().info(f'Reversed value: {data[i]}')

                # Convert the binary string to a float
                for i in range(NUM_SENSORS):
                    data[i] = self.bin_to_float(data[i])
                    self.get_logger().info(f'Float value: {data[i]}')

            except UnicodeDecodeError as e:
                self.get_logger().error('Failed to decode serial line: %s', str(e))
                continue

    def bin_to_float(self, b):
        # Credits: https://stackoverflow.com/questions/8751653/how-to-convert-a-binary-string-into-a-float-value
        bf = self.int_to_bytes(int(b, 2), FLOAT_SIZE) # 4 bytes needed for float
        return struct.unpack('>f', bf)[0]

    def int_to_bytes(self, n, length):
        return decode('%%0%dx' % (length << 1) % n, 'hex')[-length:]

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
