""" This is the main node for the scanning procedure of the LTM project.
Author: Alexander James Becoy
Revision: 1.0
Date: 09-09-2024
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Empty
from ltm_shared_msgs.srv import NavigateToPose

import os
from typing import Any
import numpy as np
from time import sleep
from datetime import datetime

from ltm_scan_procedure.data_storage import DataStorage

class ScanProcedureNode(Node):

    def __init__(self):
        super().__init__('scan_procedure_node')

        self.number_of_orientations = 8

        # Initialize attributes
        self.current_robot_position = Point()
        self.current_robot_yaw = 0.0
        self.is_scanning = False

        # Initialize data storage
        self.data_storage = DataStorage()
        self.get_logger().info(f"Saving files in {self.data_storage.storage_directory}.")

        # Configure ROS2 entities'
        self.configure_scan_procedure_parameters()
        self.configure_pointcloud_sub()
        self.configure_trigger_sub()
        self.configure_navigate_to_pose_client()
        self.configure_tf_listener()

        self.get_logger().info('Scan procedure node has been initialized.')

    def point_cloud_callback(self, msg: PointCloud2) -> None:
        """Callback function for the point cloud subscriber.
        
        Args:
            msg (PointCloud2): The point cloud message received.
        """
        # Ignore point cloud data if the node is not scanning
        if not self.is_scanning:
            return
        
        # Store the current point cloud data
        self.get_logger().info('Point cloud received.')
        self.data_storage.add_point_cloud(
            self.data_storage.convert_point_cloud2_to_open3d(msg))

    def trigger_callback(self, msg) -> None:
        self.get_logger().info('Trigger has been received.')
        self.perform_scan()

    def request_goal_pose(self, position: Point, orientation: Quaternion) -> bool:
        """Sends a request to /navigate_to_pose service to navigate the robot to the desired pose.
        
        Args:
            position (Point): The position of the goal pose.
            orientation (Quaternion): The orientation of the goal pose.
        """
        navigate_to_pose_request = NavigateToPose.Request()
        navigate_to_pose_request.goal.header.frame_id = 'map'
        navigate_to_pose_request.goal.header.stamp = self.get_clock().now().to_msg()
        navigate_to_pose_request.goal.pose.position = position
        navigate_to_pose_request.goal.pose.orientation = orientation

        navigate_to_pose_future = self.navigate_to_pose_client.call_async(navigate_to_pose_request)
        rclpy.spin_until_future_complete(self, navigate_to_pose_future)
        self.get_logger().info('Success: %s' % (navigate_to_pose_future.result().success))
        return navigate_to_pose_future.result().success

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
        self.switch_scan_mode(False)
        self.data_storage.create_storage_subdirectory(self.get_robot_state_stamp())

        for _ in range(self.number_of_orientations):
            # for pose in ['stand', 'look_down', 'sit']:
            #     self.capture_image_data(pose)
            #     self.collect_point_cloud_data()
            #     if pose == 'stand':
            #         self.stand_up()
            #     elif pose == 'sit':
            #         self.sit_down()
            
            self.get_logger().info("Moving to next orientation...")
            self.rotate_robot(2 * np.pi / self.number_of_orientations)
            self.collect_point_cloud_data()
            self.save_point_cloud_data()

        self.save_point_cloud_data()
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
            self.get_logger().error('The robot position is None. Ignoring...')
            return
        
        self.current_robot_yaw = self.normalize_yaw(self.current_robot_yaw + angle)
        # is_goal_reached = False
        # while not is_goal_reached:
            # self.get_logger().info('Moving robot to yaw: %f' % self.current_robot_yaw)
            # is_goal_reached = self.request_goal_pose(self.current_robot_position, 
            #                                          self.yaw_to_quaternion(self.current_robot_yaw))
            # self.get_logger().info('Robot reached goal: %s' % str(is_goal_reached))
        is_goal_reached = self.request_goal_pose(self.current_robot_position, 
            self.yaw_to_quaternion(self.current_robot_yaw))
        self.get_logger().info('Robot reached goal: %s' % str(is_goal_reached))

    def collect_point_cloud_data(self) -> None:
        """Collects 3D point cloud data from the environment."""
        self.get_logger().info("Collecting pointcloud...")
        self.switch_scan_mode(True)
        sleep(self.point_cloud_buffer_time)
        self.switch_scan_mode(False)
        self.get_logger().info("Pointcloud collected.")

    def save_point_cloud_data(self) -> None:
        """Saves the point cloud data to a PCD file."""
        self.get_logger().info("Saving pointcloud...")
        point_cloud_file_name = self.get_robot_state_stamp(yaw=self.current_robot_yaw) + '.pcd'
        self.data_storage.save_point_cloud(point_cloud_file_name)
        self.data_storage.reset_point_cloud()
        self.get_logger().info("Pointcloud saved.")

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
        assert isinstance(yaw, float), "Input to argument yaw is not of type float."

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
        assert isinstance(quaternion, Quaternion), "Input to argument quaternion is not of type Quaternion."

        return np.arctan2(
            2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 
            1 - 2 * (quaternion.y**2 + quaternion.z**2)
        )
    
    def get_robot_state_stamp(self, yaw=None, pose=None) -> str:
        """Returns the current robot state as a string. 
        
        Args:
            yaw (float): the orientation of the robot in the z-direction in the map frame. Default to None.
            pose (str): the pose of the robot when scanning the environment. Default to None.

        Returns:
            str: robot stamp in (x, y, yaw, pose, time)
        """
        # Round the robot position to 3 decimal places
        x = self.current_robot_position.x.round(3)
        y = self.current_robot_position.y.round(3)

        # Get current time
        current_time = datetime.now().strftime('%H-%M-%S')

        # If yaw is not provided, send x and y
        if yaw is None:
            return 'x_%s_y_%s_%s' % (str(x), str(y), current_time)

        # pose is not provided, send x, y and yaw
        if pose is None:
            return 'x_%s_y_%s_yaw_%s_%s' % (str(x), str(y), str(yaw.round(3)), current_time)
        
        return 'x_%s_y_%s_yaw_%s_pose_%s_%s' % (str(x), str(y), str(yaw.round(3)), pose, current_time)

    def switch_scan_mode(self, mode: bool) -> None:
        """Switches the scan mode to the specified mode.
        
        Args:
            mode (bool): The mode to switch to.
        """
        self.is_scanning = mode
    
    def configure_scan_procedure_parameters(self) -> None:
        """Configures the parameters for the scan procedure node. """

        # Number of orientations to scan the environment at each waypoint
        self.declare_parameter('scan_number_of_orientations', 8)
        self.number_of_orientations = self.get_parameter('scan_number_of_orientations').value
        self.get_logger().info('Number of orientations: %d' % (self.number_of_orientations))

        # Time delay between each orientation step
        self.declare_parameter('scan_delay_between_orientations', 1.0)
        self.orientation_delay = self.get_parameter('scan_delay_between_orientations').value
        self.get_logger().info('Time delay between each orientation step: %f s' % (self.orientation_delay))

        # Time to buffer 3D point cloud data
        self.declare_parameter('scan_pointcloud_buffer_time', 1.0)
        self.point_cloud_buffer_time = self.get_parameter('scan_pointcloud_buffer_time').value
        self.get_logger().info('Time to buffer 3D point cloud data: %f s' % (self.point_cloud_buffer_time))

    def configure_pointcloud_sub(self) -> None:
        """Configures the subscriber to the point cloud topic."""
        self.declare_parameter('point_cloud_subscriber_topic_name', 'point_cloud/buffer')
        self.declare_parameter('point_cloud_subscriber_queue_size', 10)

        self.point_cloud_callback_group = MutuallyExclusiveCallbackGroup()
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, self.get_parameter('point_cloud_subscriber_topic_name').value, 
            self.point_cloud_callback, self.get_parameter('point_cloud_subscriber_queue_size').value,
            callback_group=self.point_cloud_callback_group)

    def configure_trigger_sub(self) -> None:
        """Configures the subscriber to the trigger topic."""
        self.declare_parameter('trigger_subscriber_topic_name', 'perform_scan')
        self.declare_parameter('trigger_subscriber_queue_size', 10)

        self.trigger_callback_group = MutuallyExclusiveCallbackGroup()
        self.trigger_sub = self.create_subscription(
            Empty, self.get_parameter('trigger_subscriber_topic_name').value, 
            self.trigger_callback, self.get_parameter('trigger_subscriber_queue_size').value,
            callback_group=self.trigger_callback_group)

    def configure_navigate_to_pose_client(self) -> None:
        """Configures the client to the navigate_to_pose service."""
        self.declare_parameter('navigate_to_pose_service_name', 'send_robot')

        self.navigate_to_pose_callback_group = MutuallyExclusiveCallbackGroup()
        self.navigate_to_pose_client = self.create_client(
            NavigateToPose, self.get_parameter('navigate_to_pose_service_name').value, 
            callback_group=self.navigate_to_pose_callback_group)
        
    def configure_tf_listener(self) -> None:
        """Configures the tf listener for the node."""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


def main():
    rclpy.init()
    scan_procedure_node = ScanProcedureNode()
    executor = MultiThreadedExecutor()
    executor.add_node(scan_procedure_node)
    executor.spin()
    scan_procedure_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
