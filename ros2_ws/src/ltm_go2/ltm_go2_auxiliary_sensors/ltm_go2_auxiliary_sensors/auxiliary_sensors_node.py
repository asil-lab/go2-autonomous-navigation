""" This node reads and publishes data from the auxiliary sensors of the LTM Go2 robot.
Author: Alexander James Becoy
Revision: 1.0
Date: 2024-11-04
"""

import rclpy
from rclpy.node import Node

from ltm_shared_msgs.msg import AuxiliarySensorState

import os
import csv
from datetime import datetime
from serial import Serial, SerialException
import threading
import struct
from codecs import decode
from enum import Enum

BYTE_SIZE   = 8     # bits
FLOAT_SIZE  = 4     # bytes
NUM_SENSORS = 3     # temperature, humidity, light

LTM_RECORDINGS_AMBIENCE_DIRECTORY = os.environ.get('LTM_RECORDINGS_AMBIENCE_DIRECTORY')

class MISO(Enum):
    TEMPERATURE_OFFSET  = 0
    HUMIDITY_OFFSET     = 4 * BYTE_SIZE
    LIGHT_OFFSET        = 8 * BYTE_SIZE

class SensorData(Enum):
    TEMPERATURE = 0
    HUMIDITY    = 1
    LUMINOSITY  = 2

class AuxiliarySensorsNode(Node):

    def __init__(self):
        super().__init__('auxiliary_sensors_node')

        self.create_csv()
        self.initialize_serial_port()
        self.initialize_auxiliary_sensor_state_publisher()
        self.initialize_thread()

        self.get_logger().info('Auxiliary sensors node initialized')

    def read_serial_port(self):
        while rclpy.ok():
            # If no data is available, continue to next iteration
            if self.serial_port.in_waiting == 0:
                continue
            
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().debug(f'Received line: {line}')
                data = self.process_stream(line)
                self.publish_auxiliary_sensor_state(data)

            except UnicodeDecodeError as e:
                self.get_logger().error('Failed to decode serial line: %s', str(e))
                continue

    def publish_auxiliary_sensor_state(self, data):
        # Create an AuxiliarySensorState message
        msg = AuxiliarySensorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.temperature = data[SensorData.TEMPERATURE.value]
        msg.humidity = data[SensorData.HUMIDITY.value]
        msg.luminosity = data[SensorData.LUMINOSITY.value]

        # Publish the data as an AuxiliarySensorState message
        self.auxiliary_sensor_state_publisher.publish(msg)

        # Write the data to the CSV
        self.write_csv(msg)

    def process_stream(self, line):
        data = self.split_line(line)
        return self.convert_data(data)

    def split_line(self, line):
        return [line[offset.value:offset.value+(FLOAT_SIZE*BYTE_SIZE)] for offset in MISO]

    def convert_data(self, data):
        return [self.bin_to_float(self.reverse_bits(data[i])) for i in range(NUM_SENSORS)]

    def reverse_bits(self, b):
        return b[::-1]

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
            AuxiliarySensorState, 'ambient_state', 10)
        
    def initialize_thread(self):
        self.thread = threading.Thread(target=self.read_serial_port)
        self.thread.daemon = True
        self.thread.start()
        
    def create_csv(self):
        # Create directory if it does not exist
        if not os.path.exists(LTM_RECORDINGS_AMBIENCE_DIRECTORY):
            os.makedirs(LTM_RECORDINGS_AMBIENCE_DIRECTORY)

        # Create a CSV in LTM_RECORDINGS_AMBIENCE_DIRECTORY with the following headers:
        # timestamp, temperature, humidity, light
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.filepath = f'{LTM_RECORDINGS_AMBIENCE_DIRECTORY}/ambient_data_{timestamp}.csv'
        with open(self.filepath, mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(['sec', 'nsec', 'temperature', 'humidity', 'light'])

    def write_csv(self, ambient_state: AuxiliarySensorState):
        # Append data as a new row in the CSV
        with open(self.filepath, mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([
                ambient_state.header.stamp.sec,
                ambient_state.header.stamp.nanosec,
                ambient_state.temperature,
                ambient_state.humidity,
                ambient_state.luminosity
            ])

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
