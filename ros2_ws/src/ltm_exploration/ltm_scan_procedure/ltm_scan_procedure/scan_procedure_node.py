""" This is the main node for the scanning procedure of the LTM project.
Author: Alexander James Becoy
Revision: 1.0
Date: 09-09-2024
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Empty

import os
import numpy as np
from time import sleep

class ScanProcedureNode(Node):

    def __init__(self):
        super().__init__('scan_procedure_node')

        self.number_of_orientations = 8

        self.current_robot_position = Point()
        self.current_robot_yaw = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trigger_sub = self.create_subscription(Empty, 'perform_scan', self.trigger_callback, 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        self.get_logger().info('Scan procedure node has been initialized.')

    def trigger_callback(self, msg) -> None:
        self.get_logger().info('Trigger has been received.')
        self.perform_scan()

    def publish_goal_pose(self, position: Point, orientation: Quaternion) -> None:
        """Publishes the goal pose to the goal_pose topic.
        
        Args:
            position (Point): The position of the goal pose.
            orientation (Quaternion): The orientation of the goal pose.
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = position
        goal_pose.pose.orientation = orientation
        self.goal_pose_pub.publish(goal_pose)

    def perform_scan(self) -> None:
        """ Perform scan procedure in the following order:
        1. Get the current robot position and yaw
        2. for number of orientation steps:
            3. for pose in [stand, looking down, sit]:
                3. Capture 2D image data
                4. Collect 3D point cloud data
                5. Stand up and recover robot pose
            6. Rotate robot by 360/number of orientations
        7. Save the 2D images and 3D point cloud data
        """
        self.get_current_robot_pose()
        for _ in range(self.number_of_orientations):
            # for pose in ['stand', 'look_down', 'sit']:
            #     self.capture_image_data(pose)
            #     self.collect_point_cloud_data()
            #     if pose == 'stand':
            #         self.stand_up()
            #     elif pose == 'sit':
            #         self.sit_down()
            self.rotate_robot(2 * np.pi / self.number_of_orientations)
            sleep(10.0)
        # self.save_data()
        self.get_logger().info('Scan procedure at x: %f, y: %f, yaw: %f' % 
                               (self.current_robot_position.x, self.current_robot_position.y, self.current_robot_yaw))

    def get_current_robot_pose(self) -> None:
        """Gets the current pose representation of the robot in the map frame.
        The current robot position is stored in the attribute self.current_robot_position.
        The current robot yaw is stored in the attribute self.current_robot_yaw.
        """
        try:
            # Get the current robot pose
            current_robot_pose = self.tf_buffer.lookup_transform('map', 'base', rclpy.time.Time()).transform
            # Store the current robot position and yaw
            self.current_robot_position.x = current_robot_pose.translation.x
            self.current_robot_position.y = current_robot_pose.translation.y
            self.current_robot_position.z = current_robot_pose.translation.z
            self.current_robot_yaw = self.quaternion_to_yaw(current_robot_pose.rotation)
        except Exception as e:
            self.get_logger().error('Failed to get current robot position: %s' % str(e))
            self.current_robot_position = None

    def rotate_robot(self, angle: float) -> None:
        """Rotates the robot by the specified angle.
        
        Args:
            angle (float): The angle to rotate the robot by.
        """
        if self.current_robot_position is None:
            self.get_logger().error('The robot position is None.')
            return
        
        self.current_robot_yaw = self.normalize_yaw(self.current_robot_yaw + angle)
        self.publish_goal_pose(
            self.current_robot_position, 
            self.yaw_to_quaternion(self.current_robot_yaw)
        )

    def normalize_yaw(self, yaw: float) -> float:
        """Normalizes a yaw angle to be within the range [-pi, pi].
        Args:
            yaw (float): The yaw angle to normalize.

        Returns:
            float: The normalized yaw angle.
        """
        return yaw % (2 * np.pi)

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Converts a yaw angle to a quaternion.
        Args:
            yaw (float): The yaw angle to convert.

        Returns:
            Quaternion: The quaternion representation of the yaw angle.
        """
        quaternion = Quaternion()
        quaternion.z = np.sin(yaw / 2)
        quaternion.w = np.cos(yaw / 2)
        return quaternion

    def quaternion_to_yaw(self, quaternion: Quaternion) -> float:
        """Converts a quaternion to a yaw angle.
        Args:
            quaternion (Quaternion): The quaternion to convert.

        Returns:
            float: The yaw angle representation of the quaternion.
        """
        return np.arctan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1 - 2 * (quaternion.y**2 + quaternion.z**2))

def main():
    rclpy.init()
    scan_procedure_node = ScanProcedureNode()
    rclpy.spin(scan_procedure_node)
    scan_procedure_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
