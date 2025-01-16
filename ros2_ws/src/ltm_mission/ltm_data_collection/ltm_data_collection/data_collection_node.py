"""
TODO:
- [ ] Feasibility: demonstration
- [ ] Multiple trials: success/fail (RMSE), reachability to target
- [ ] Time-to-reach target
- [ ] Implement data collection on autonomous navigation

Assumptions:
- [ ] Waypoint resolution (distance between waypoints)

"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

import os
import csv
import numpy as np
from datetime import datetime
from tf2_ros import Buffer, TransformListener

class DataCollectionNode(Node):

    def __init__(self) -> None:
        super().__init__('data_collection_node')

        # Create data directory if it does not exist
        self.data_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'data')
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
        self.get_logger().info(f'Data directory: {self.data_dir}')

        # Create a CSV file to store data
        self.data_headers = ['sec','nanosec', 'x', 'y', 'yaw', 'x_target', 'y_target', 'yaw_target', 'distance_to_target', 'time_to_target_sec', 'time_to_target_nanosec']
        current_datetime = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.data_filename = os.path.join(self.data_dir, f'data_{current_datetime}.csv')
        with open(self.data_filename, mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(self.data_headers)
        self.get_logger().info(f'Data file: {self.data_filename}')

        # Create TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create timer to collect data
        self.timer_period = 0.1 # seconds, 10 Hz
        self.timer = self.create_timer(self.timer_period, self.collect_data)

        # Create subscriber to get waypoints
        self.current_waypoint = None
        self.time_to_target = rclpy.time.Time()
        self.waypoint_sub = self.create_subscription(PoseStamped, 'goal_pose', self.waypoint_callback, 10)

        self.get_logger().info('Data collection node has been initialized')

    def waypoint_callback(self, msg: PoseStamped):
        self.current_waypoint = msg
        self.time_to_target = rclpy.time.Time()
        self.get_logger().info(f'Waypoint received: {self.current_waypoint}')

    def collect_data(self):
        # Get robot pose in the map frame
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base', rclpy.time.Time())
            robot_translation = transform.transform.translation
            robot_x = robot_translation.x
            robot_y = robot_translation.y
            robot_yaw = transform.transform.rotation.z
        except Exception as e:
            self.get_logger().error(f'Failed to get robot pose: {e}')
            robot_translation = None
            robot_x = None
            robot_y = None
            robot_yaw = None
        
        # Get target pose in the map frame
        if self.current_waypoint is not None:
            target_translation = self.current_waypoint.pose.position
            target_x = target_translation.x
            target_y = target_translation.y
            target_yaw = self.current_waypoint.pose.orientation.z
        else:
            target_translation = None
            target_x = None
            target_y = None
            target_yaw = None

        # Calculate distance to target
        if target_translation is not None and robot_translation is not None:
            distance_to_target = np.sqrt((target_translation.x - robot_translation.x)**2 + (target_translation.y - robot_translation.y)**2)
        else:
            distance_to_target = None

        # Calculate time to target
        if target_translation is not None and robot_translation is not None:
            self.time_to_target = self.current_waypoint.header.stamp - rclpy.time.Time()
            time_to_target_sec = self.time_to_target.seconds
            time_to_target_nanosec = self.time_to_target.nanoseconds
        else:
            time_to_target_sec = None
            time_to_target_nanosec = None

        # Write data to CSV file
        current_sec, current_nanosec = self.get_clock().now().seconds_nanoseconds()
        data = [current_sec, current_nanosec, robot_x, robot_y, robot_yaw, target_x, target_y, target_yaw, distance_to_target, time_to_target_sec, time_to_target_nanosec]
        with open(self.data_filename, mode='a') as file:
            writer = csv.writer(file)
            writer.writerow(data)


def main():
    rclpy.init()
    node = DataCollectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
