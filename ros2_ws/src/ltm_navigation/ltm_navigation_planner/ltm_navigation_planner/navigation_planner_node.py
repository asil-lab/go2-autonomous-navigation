""" This is the main node for the ltm_navigation_planner package.
Author: Alexander James Becoy
Revision: 1.0
Date: 19-08-2024
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener

import os
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Pose2D

from ltm_shared_msgs.srv import LoadMap, GenerateWaypoints
from nav_msgs.srv import GetMap
from slam_toolbox.srv import DeserializePoseGraph

from ltm_navigation_planner.map_reader import MapReader
from ltm_navigation_planner.path_planner import PathPlanner
from ltm_navigation_planner.utils import quaternion_to_yaw

LTM_MAPS_DIRECTORY = os.environ.get('LTM_MAPS_DIRECTORY')

class NavigationPlannerNode(Node):

    def __init__(self) -> None:
        super().__init__('navigation_planner_node')

        self.map_reader = MapReader()
        self.path_planner = PathPlanner()

        self.configure_load_map_service()
        self.configure_generate_waypoints_service()
        self.configure_dynamic_map_client()
        self.configure_deserialize_map_client()
        self.configure_tf_listener()

        # Create the subscribers
        self.waypoint_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.waypoints_pub = self.create_publisher(PoseArray, 'all_waypoints', 10)

        self.get_logger().info('Navigation Planner Node has been initialized.')

    # def map_callback(self, msg) -> None:
    #     self.map_reader.configure_metadata(msg.info.resolution, msg.info.origin.position, msg.info.width, msg.info.height)
    #     self.map_reader.read_map_list(msg.data)
    #     waypoints = self.map_reader.read()
    #     self.get_logger().info('Map has been read. There are %d waypoints' % len(waypoints))
    #     for i, waypoint in enumerate(waypoints):
    #         self.get_logger().info('Waypoint %d: (%f, %f, %f)' % (i, waypoint[0], waypoint[1], waypoint[2]))
    #     self.publish_waypoints(waypoints)

    #     maps = self.map_reader.get_maps()
    #     for key, value in maps.items():
    #         self.save_map(value, key)

    #     self.path_planner.set_waypoints(waypoints)
    #     robot_x, robot_y = self.get_robot_position()
    #     self.get_logger().info('Robot position is (%f, %f)' % (robot_x, robot_y))
    #     self.path_planner.set_start(robot_x, robot_y)
    #     _ = self.path_planner.plan()
    #     self.get_logger().info('Path has been planned.')

    def load_map_callback(self, request, response) -> LoadMap.Response:
        self.get_logger().info('Map has been requested.')

        # Request to deserialize the map into the map server
        deserialize_map_request = DeserializePoseGraph.Request()
        deserialize_map_request.filename = os.path.join(LTM_MAPS_DIRECTORY, request.filename, request.filename)
        deserialize_map_request.match_type = DeserializePoseGraph.Request.LOCALIZE_AT_POSE # START_AT_GIVEN_POSE
        deserialize_map_request.initial_pose = self.get_robot_pose()
        deserialize_map_future = self.deserialize_map_client.call_async(deserialize_map_request)
        self.get_logger().info(f'Loading map from file {deserialize_map_request.filename}...')
        rclpy.spin_until_future_complete(self, deserialize_map_future)
        self.get_logger().info('Map has been loaded from file.')

        # Request the map from the SLAM Toolbox map server
        dynamic_map_request = GetMap.Request()
        dynamic_map_future = self.dynamic_map_client.call_async(dynamic_map_request)
        self.get_logger().info('Requesting map from server...')
        rclpy.spin_until_future_complete(self, dynamic_map_future)
        self.get_logger().info('Map has been received from server.')

        # Load the map from the future
        map = dynamic_map_future.result().map
        self.map_reader.configure_metadata(map.info.resolution, map.info.origin.position, map.info.width, map.info.height)
        self.map_reader.read_map_list(map.data)
        self.get_logger().info('Map has been read.')

        return response

    def load_map_future_callback(self, future) -> None:
        # Get the map from the future and read it
        self.get_logger().info('Map received from server.')
        map = future.result().map
        self.map_reader.configure_metadata(map.info.resolution, map.info.origin.position, map.info.width, map.info.height)
        self.map_reader.read_map_list(map.data)
        self.get_logger().info('Load map completed')

    def generate_waypoints_callback(self, request, response) -> GenerateWaypoints.Response:
        self.get_logger().info('Generate waypoints has been requested.')

        # Generate waypoints from the map
        self.get_logger().info('Generating waypoints...')
        waypoints = self.map_reader.read()
        self.publish_waypoints(waypoints)
        self.get_logger().info('Waypoints have been generated.')

        # Plan the path
        self.path_planner.set_waypoints(waypoints)
        robot_x, robot_y = self.get_robot_position()
        self.path_planner.set_start(robot_x, robot_y)
        _ = self.path_planner.plan()
        self.get_logger().info('Path has been planned.')

        # Return the response
        response.success = True
        response.num_waypoints = len(waypoints)

        return response

    def publish_waypoint(self, waypoint: dict) -> None:
        if len(waypoint) == 0:
            self.get_logger().info('The path has been completed.')
            return
        
        pose = PoseStamped()

        pose.pose.position.x = waypoint['x']
        pose.pose.position.y = waypoint['y']
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = np.sin(waypoint['yaw'] / 2)
        pose.pose.orientation.w = np.cos(waypoint['yaw'] / 2)

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        self.waypoint_pub.publish(pose)
        self.get_logger().info('Waypoint has been published.')

    def publish_waypoints(self, waypoints: np.ndarray) -> None:
        assert waypoints is not None, 'The waypoints are None'
        
        poses_msg = PoseArray()
        poses_msg.header.frame_id = 'map'
        poses_msg.header.stamp = self.get_clock().now().to_msg()

        for waypoint in waypoints:
            pose = Pose()
            pose.position.x = waypoint[0]
            pose.position.y = waypoint[1]
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = np.sin(waypoint[2] / 2)
            pose.orientation.w = np.cos(waypoint[2] / 2)
            poses_msg.poses.append(pose)
        
        self.waypoints_pub.publish(poses_msg)
        self.get_logger().info('Waypoints have been published.')

    def get_robot_position(self) -> tuple:
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return transform.transform.translation.x, transform.transform.translation.y
        except Exception as e:
            self.get_logger().error('Failed to get the robot position: %s' % str(e))
            return None
        
    def get_robot_pose(self) -> Pose2D:
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            pose = Pose2D()
            pose.x = transform.transform.translation.x
            pose.y = transform.transform.translation.y
            pose.theta = quaternion_to_yaw(transform.transform.rotation)
            return pose
        except Exception as e:
            self.get_logger().error('Failed to get the robot pose: %s' % str(e))
            return None
        
    def save_map(self, map: np.ndarray, filename: str) -> None:
        save_directory = os.path.dirname(__file__) + '/../results/'
        plt.imsave(save_directory + filename + '.png', map, cmap='gray')
        self.get_logger().info('Map has been saved to %s' % filename)

    def configure_deserialize_map_client(self) -> None:
        self.deserialize_map_callback_group = MutuallyExclusiveCallbackGroup()
        self.deserialize_map_client = self.create_client(
            DeserializePoseGraph, 'slam_toolbox/deserialize_map', callback_group=self.deserialize_map_callback_group)
        while not self.deserialize_map_client.wait_for_service():
            self.get_logger().warn('Service slam_toolbox/deserialize_map is not available.', throttle_duration_sec=5.0)
        self.get_logger().info('Deserialize Map client has been configured')

    def configure_dynamic_map_client(self) -> None:
        self.dynamic_map_callback_group = MutuallyExclusiveCallbackGroup()
        self.dynamic_map_client = self.create_client(GetMap, 'slam_toolbox/dynamic_map', callback_group=self.dynamic_map_callback_group)
        while not self.dynamic_map_client.wait_for_service():
            self.get_logger().warn('Service slam_toolbox/dynamic_map is not available.', throttle_duration_sec=5.0)
        self.get_logger().info('Dynamic Map client has been configured.')

    def configure_load_map_service(self) -> None:
        self.load_map_service_callback_group = MutuallyExclusiveCallbackGroup()
        self.load_map_service = self.create_service(
            LoadMap, 'state_machine/load_map', 
            self.load_map_callback, callback_group=self.load_map_service_callback_group)
        self.get_logger().info('Load Map service has been configured.')
    
    def configure_generate_waypoints_service(self) -> None:
        self.generate_waypoints_service_callback_group = MutuallyExclusiveCallbackGroup()
        self.generate_waypoints_service = self.create_service(
            GenerateWaypoints, 'state_machine/generate_waypoints',
            self.generate_waypoints_callback, callback_group=self.generate_waypoints_service_callback_group)
        self.get_logger().info('Generate Waypoints service has been configured.')

    def configure_tf_listener(self) -> None:
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


def main():
    rclpy.init()
    node = NavigationPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
