""" This is the main node for the ltm_navigation_planner package.
Author: Alexander James Becoy
Revision: 1.0
Date: 19-08-2024
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import os
import numpy as np
import matplotlib.pyplot as plt
from tf2_ros import Buffer, TransformListener

from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from ltm_shared_msgs.srv import LoadMap
from nav_msgs.srv import GetMap

from ltm_navigation_planner.map_reader import MapReader
from ltm_navigation_planner.path_planner import PathPlanner

class NavigationPlannerNode(Node):
    """ TODO: 
    - [ ] Read the map from the map server.
    - [ ] Get the area to be explored that is defined by the obstacle map (in the form of a polygon).
    - [ ] Create waypoints along the boundary of the area to be explored.
    - [ ] Get the current position of the robot.
    - [ ] Plan the optimal path to visit all the waypoints.
    - [ ] Publish the next waypoint according to the state machine.
    """

    def __init__(self) -> None:
        super().__init__('navigation_planner_node')

        self.map_reader = MapReader()
        self.path_planner = PathPlanner()

        self.configure_dynamic_map_client()
        self.configure_load_map_service()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create the subscribers
        self.trigger_sub = self.create_subscription(Empty, 'trigger', self.trigger_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'buffered_map', self.map_callback, 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.waypoints_pub = self.create_publisher(PoseArray, 'all_waypoints', 10)

        self.get_logger().info('Navigation Planner Node has been initialized.')

    def trigger_callback(self, msg) -> None:
        assert self.path_planner.path is not None, 'The path does not exist'
        self.publish_waypoint(self.path_planner.get_next_waypoint())
        self.get_logger().info('Trigger has been received.')

    def map_callback(self, msg) -> None:
        self.map_reader.configure_metadata(msg.info.resolution, msg.info.origin.position, msg.info.width, msg.info.height)
        self.map_reader.read_map_list(msg.data)
        waypoints = self.map_reader.read()
        self.get_logger().info('Map has been read. There are %d waypoints' % len(waypoints))
        for i, waypoint in enumerate(waypoints):
            self.get_logger().info('Waypoint %d: (%f, %f, %f)' % (i, waypoint[0], waypoint[1], waypoint[2]))
        self.publish_waypoints(waypoints)

        maps = self.map_reader.get_maps()
        for key, value in maps.items():
            self.save_map(value, key)

        self.path_planner.set_waypoints(waypoints)
        robot_x, robot_y = self.get_robot_position()
        self.get_logger().info('Robot position is (%f, %f)' % (robot_x, robot_y))
        self.path_planner.set_start(robot_x, robot_y)
        _ = self.path_planner.plan()
        self.get_logger().info('Path has been planned.')

    def load_map_callback(self, request, response) -> LoadMap.Response:
        self.get_logger().info('Map has been requested.')
        _ = request

        # Request the map from the SLAM Toolbox map server
        request = GetMap.Request()
        future = self.dynamic_map_client.call_async(request)
        self.get_logger().info('Requesting map from server...')
        rclpy.spin_until_future_complete(self, future)

        # Get the map from the future and read it
        self.get_logger().info('Map received from server.')
        map = future.result().map
        self.map_reader.configure_metadata(map.info.resolution, map.info.origin.position, map.info.width, map.info.height)
        self.map_reader.read_map_list(map.data)
        self.get_logger().info('Load map completed.')

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
        
    def save_map(self, map: np.ndarray, filename: str) -> None:
        save_directory = os.path.dirname(__file__) + '/../results/'
        plt.imsave(save_directory + filename + '.png', map, cmap='gray')
        self.get_logger().info('Map has been saved to %s' % filename)

    def configure_dynamic_map_client(self) -> None:
        self.dynamic_map_client = self.create_client(GetMap, 'slam_toolbox/dynamic_map')
        while not self.dynamic_map_client.wait_for_service():
            self.get_logger().warn('Service slam_toolbox/dynamic_map is not available.', throttle_duration_sec=5.0)
        self.get_logger().info('Dynamic Map client has been configured.')

    def configure_load_map_service(self) -> None:
        self.load_map_service_callback_group = MutuallyExclusiveCallbackGroup()
        self.load_map_service = self.create_service(
            LoadMap, 'state_machine/load_map', 
            self.load_map_callback, callback_group=self.load_map_service_callback_group)
        self.get_logger().info('Load Map service has been configured.')
    


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    navigation_planner_node = NavigationPlannerNode()
    executor.add_node(navigation_planner_node)
    executor.spin()
    rclpy.shutdown()
    navigation_planner_node.destroy_node()


if __name__ == '__main__':
    main()
