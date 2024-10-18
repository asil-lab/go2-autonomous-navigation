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

from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Empty, String
from ltm_shared_msgs.srv import GetImage, GetPointCloud, NavigateToPose, ScanEnvironment

import os
from typing import Any
import numpy as np
from time import sleep
from datetime import datetime

from ltm_scan_procedure.data_storage import DataStorage
from ltm_scan_procedure.utils import yaw_to_quaternion, quaternion_to_yaw, normalize_yaw, image_msg_to_numpy

class ScanProcedureNode(Node):

    def __init__(self):
        super().__init__('scan_procedure_node')

        self.number_of_orientations = 8

        # Initialize attributes
        self.configure_scan_procedure_parameters()
        self.current_robot_position = Point()
        self.current_robot_yaw = 0.0
        self.current_robot_gesture = 'stand'
        self.current_subdirectory = ""
        self.is_scanning = False

        # Initialize data storage
        self.data_storage = DataStorage()
        self.get_logger().info(f"Saving files in {self.data_storage.storage_directory}.")

        # Configure ROS2 entities'
        self.configure_gesture_pub()
        self.configure_record_environment_service()
        self.configure_navigate_to_pose_client()
        self.configure_get_image_client()
        self.configure_get_pointcloud_client()
        self.configure_tf_listener()

        self.get_logger().info('Scan procedure node has been initialized.')

    def record_environment_callback(self, request, response) -> ScanEnvironment.Response:
        self.get_logger().info('Record environment service has been called.')
        self.number_of_orientations = request.num_orientations
        self.point_cloud_buffer_time = request.scan_time

        self.perform_scan()
        response.success = True
        return response

    def publish_gesture(self, gesture: str) -> None:
        """Publishes the gesture to the gesture topic.
        
        Args:
            gesture (str): The gesture to publish.
        """
        assert isinstance(gesture, str), "Input to argument gesture is not of type str."

        if len(gesture) == 0:
            self.get_logger().error('Gesture is empty. Ignoring...')
            return

        gesture_msg = String()
        gesture_msg.data = gesture
        self.gesture_pub.publish(gesture_msg)

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
        # rclpy.spin_until_future_complete(self, navigate_to_pose_future)
        self.executor.spin_until_future_complete(self, navigate_to_pose_future)
        self.get_logger().info('Success: %s' % (navigate_to_pose_future.result().success))
        return navigate_to_pose_future.result().success

    def request_image(self) -> Image:
        """Requests an image from the /get_image service."""
        get_image_request = GetImage.Request()
        get_image_future = self.get_image_client.call_async(get_image_request)
        # rclpy.spin_until_future_complete(self, get_image_future)
        self.executor.spin_until_future_complete(self, get_image_future)
        return get_image_future.result().image
    
    def request_point_cloud(self) -> PointCloud2:
        """Requests a point cloud from the /get_pointcloud service."""
        get_pointcloud_request = GetPointCloud.Request()
        get_pointcloud_future = self.get_pointcloud_client.call_async(get_pointcloud_request)
        # rclpy.spin_until_future_complete(self, get_pointcloud_future)
        self.executor.spin_until_future_complete(self, get_pointcloud_future)
        return get_pointcloud_future.result().point_cloud

    def perform_scan(self) -> None:
        """ Perform scan procedure in the following order:
        1. Get the current robot position and yaw
        2. for number of orientation steps:
            3. for gesture in [stand, looking down, sit]:
                3. Capture 2D image data
                4. Collect 3D point cloud data
                5. Stand up and recover robot gesture
            6. Rotate robot by 360/number of orientations
        7. Save the 2D images and 3D point cloud data
        """
        self.get_current_robot_pose()

        self.current_subdirectory = self.get_robot_state_stamp()
        self.data_storage.create_storage_subdirectory(self.current_subdirectory)

        # Perform scan procedure at each orientation
        for _ in range(self.number_of_orientations):
            # Perform gestures at each orientation
            for gesture in self.gesture_sequence:
                self.perform_gesture(gesture)

                # Stop if gesture is recover
                if gesture == 'recover':
                    continue
                
                # Collect 3D point cloud data
                self.collect_point_cloud_data()
                self.save_point_cloud_data()

                # Collect 2D image data
                self.collect_image_data()
                self.save_image_data()
            
            self.get_logger().info("Moving to next orientation...")
            self.rotate_robot(2 * np.pi / self.number_of_orientations)

        self.get_logger().info('Scan procedure at x: %f, y: %f, yaw: %f' % 
            (self.current_robot_position.x, self.current_robot_position.y, self.current_robot_yaw))

    def get_current_robot_pose(self) -> None:
        """Gets the current pose representation of the robot in the map frame.
        The current robot position is stored in the attribute self.current_robot_position.
        The current robot yaw is stored in the attribute self.current_robot_yaw.
        """
        try:
            # Get the current robot pose
            current_robot_pose = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform

            # Store the current robot position and yaw
            self.current_robot_position.x = current_robot_pose.translation.x
            self.current_robot_position.y = current_robot_pose.translation.y
            self.current_robot_position.z = current_robot_pose.translation.z
            self.current_robot_yaw = quaternion_to_yaw(current_robot_pose.rotation)
            
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
        
        self.current_robot_yaw = normalize_yaw(self.current_robot_yaw + angle)
        is_goal_reached = False

        while not is_goal_reached:
            self.get_logger().info('Moving robot to yaw: %f' % self.current_robot_yaw)
            is_goal_reached = self.request_goal_pose(self.current_robot_position, 
                                                     yaw_to_quaternion(self.current_robot_yaw))
            self.get_logger().info('Robot reached goal: %s' % str(is_goal_reached))

        is_goal_reached = self.request_goal_pose(self.current_robot_position, 
            yaw_to_quaternion(self.current_robot_yaw))
        self.get_logger().info('Robot reached goal: %s' % str(is_goal_reached))

    def perform_gesture(self, gesture: str) -> None:
        """Performs the specified gesture.
        
        Args:
            gesture (str): The gesture to perform.
        """
        self.get_logger().info('Performing gesture: %s' % gesture)
        self.current_robot_gesture = gesture
        self.publish_gesture(gesture)
        sleep(self.gesture_delay)

    def collect_point_cloud_data(self) -> None:
        """Collects 3D point cloud data from the environment. """
        self.get_logger().info("Collecting pointcloud...")
        start_time = self.get_clock().now()

        while self.get_clock().now() - start_time < rclpy.time.Duration(seconds=self.point_cloud_buffer_time):
            pointcloud = self.request_point_cloud()
            self.data_storage.add_point_cloud(pointcloud)
            sleep(0.1)  # TODO: Parameterize this

        self.get_logger().info("Pointcloud collected.")

    def save_point_cloud_data(self) -> None:
        """Saves the point cloud data to a PCD file. """
        self.get_logger().info("Saving pointcloud...")
        point_cloud_file_name = self.get_robot_state_stamp(
            yaw=self.current_robot_yaw, gesture=self.current_robot_gesture) + '.pcd'
        is_save_successful = self.data_storage.save_point_cloud(
            os.path.join(self.current_subdirectory, point_cloud_file_name))
        self.data_storage.reset_point_cloud()
        self.get_logger().info("Pointcloud saved: %s" % (str(is_save_successful)))

    def collect_image_data(self) -> None:
        """Collects 2D image data from the environment. """
        self.get_logger().info("Collecting image...")
        image_msg = self.request_image()
        self.data_storage.add_image(image_msg_to_numpy(image_msg))
        self.get_logger().info("Image collected.")

    def save_image_data(self) -> None:
        """Saves the image data to a JPG file. """
        self.get_logger().info("Saving image...")
        image_file_name = self.get_robot_state_stamp(
            yaw=self.current_robot_yaw, gesture=self.current_robot_gesture) + '.jpg'
        self.data_storage.save_image(os.path.join(self.current_subdirectory, image_file_name))
        self.get_logger().info("Image saved.")
    
    def get_robot_state_stamp(self, yaw=None, gesture=None) -> str:
        """Returns the current robot state as a string. 
        
        Args:
            yaw (float): the orientation of the robot in the z-direction in the map frame. Default to None.
            gesture (str): the gesture of the robot when scanning the environment. Default to None.

        Returns:
            str: robot stamp in (x, y, yaw, gesture, time)
        """
        # Round the robot position to 3 decimal places
        x = str(round(self.current_robot_position.x, 3))
        y = str(round(self.current_robot_position.y, 3))

        # Get current time
        current_time = datetime.now().strftime('%H-%M-%S')

        # If yaw is not provided, send x and y
        if yaw is None:
            return 'x_%s_y_%s_%s' % (x, y, current_time)
        yaw_str = str(round(yaw, 3))

        # gesture is not provided, send x, y and yaw
        if gesture is None:
            return 'x_%s_y_%s_yaw_%s_%s' % (x, y, yaw_str, current_time)
        
        return 'x_%s_y_%s_yaw_%s_gesture_%s_%s' % (x, y, yaw_str, gesture, current_time)
    
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

        # Sequence of gestures to scan the environment
        self.declare_parameter('scan_gesture_sequence', ['stand', 'recover'])
        self.gesture_sequence = self.get_parameter('scan_gesture_sequence').value
        self.get_logger().info('Gesture sequence: %s' % (str(self.gesture_sequence)))

        # Time delay after each gesture
        self.declare_parameter('scan_delay_between_gestures', 1.0)
        self.gesture_delay = self.get_parameter('scan_delay_between_gestures').value
        self.get_logger().info('Time delay between each gesture: %f s' % (self.gesture_delay))

    def configure_gesture_pub(self) -> None:
        """Configures the publisher to the gesture topic."""
        self.declare_parameter('gesture_publisher_topic_name', 'gesture')
        self.declare_parameter('gesture_publisher_queue_size', 10)

        self.gesture_pub = self.create_publisher(
            String, self.get_parameter('gesture_publisher_topic_name').value, 
            self.get_parameter('gesture_publisher_queue_size').value)

    def configure_navigate_to_pose_client(self) -> None:
        """Configures the client to the navigate_to_pose service."""
        self.declare_parameter('navigate_to_pose_service_name', 'send_robot')

        self.navigate_to_pose_callback_group = MutuallyExclusiveCallbackGroup()
        self.navigate_to_pose_client = self.create_client(
            NavigateToPose, self.get_parameter('navigate_to_pose_service_name').value, 
            callback_group=self.navigate_to_pose_callback_group)
        
    def configure_get_image_client(self) -> None:
        """Configures the client to the get_image service."""
        self.declare_parameter('get_image_service_name', 'get_image')

        self.get_image_callback_group = MutuallyExclusiveCallbackGroup()
        self.get_image_client = self.create_client(
            GetImage, self.get_parameter('get_image_service_name').value, 
            callback_group=self.get_image_callback_group)
        
        while not self.get_image_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service %s not available, waiting again...' % \
                                   (self.get_parameter('get_image_service_name').value))
        
    def configure_get_pointcloud_client(self) -> None:
        """Configures the client to the get_pointcloud service."""
        self.declare_parameter('get_pointcloud_service_name', 'get_pointcloud')

        self.get_pointcloud_callback_group = MutuallyExclusiveCallbackGroup()
        self.get_pointcloud_client = self.create_client(
            GetPointCloud, self.get_parameter('get_pointcloud_service_name').value, 
            callback_group=self.get_pointcloud_callback_group)
        
        while not self.get_pointcloud_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service %s not available, waiting again...' % \
                                   (self.get_parameter('get_pointcloud_service_name').value))

    def configure_record_environment_service(self) -> None:
        """Configures the client to the record_environment service."""
        self.declare_parameter('record_environment_service_name', 'record_environment')

        self.record_environment_callback_group = MutuallyExclusiveCallbackGroup()
        self.record_environment_service = self.create_service(
            ScanEnvironment, self.get_parameter('record_environment_service_name').value, 
            self.record_environment_callback, callback_group=self.record_environment_callback_group)

    def configure_tf_listener(self) -> None:
        """Configures the tf listener for the node."""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


def main():
    rclpy.init()
    scan_procedure_node = ScanProcedureNode()
    executor = MultiThreadedExecutor()
    executor.add_node(scan_procedure_node)
    scan_procedure_node.executor = executor
    rclpy.spin(scan_procedure_node, executor)
    scan_procedure_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
