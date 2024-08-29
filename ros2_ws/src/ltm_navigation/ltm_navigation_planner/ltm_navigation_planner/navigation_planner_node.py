""" This is the main node for the ltm_navigation_planner package.
Author: Alexander James Becoy
Revision: 1.0
Date: 19-08-2024
"""

import rclpy
from rclpy.node import Node

import numpy as np
from tf2_ros import Buffer, TransformListener

from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

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
        self.get_logger().info('Navigation Planner Node has been initialized.')

        self.map_reader = MapReader()
        self.path_planner = PathPlanner()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create the subscribers
        self.trigger_sub = self.create_subscription(Empty, 'trigger', self.trigger_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'buffered_map', self.map_callback, 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

    def __del__(self):
        self.get_logger().info('Navigation Planner Node has been terminated.')

    def trigger_callback(self, msg) -> None:
        assert self.path_planner.waypoints is not None, 'The waypoints are None'
        self.publish_waypoint(self.path_planner.get_next_waypoint())
        self.get_logger().info('Trigger has been received.')

    def map_callback(self, msg) -> None:
        self.map_reader.configure_metadata(msg.info.resolution, msg.info.origin.position, msg.info.width, msg.info.height)
        self.map_reader.read_map_list(msg.data)
        waypoints = self.map_reader.read()
        self.get_logger().info('Map has been read.')

        self.path_planner.set_waypoints(waypoints)
        self.path_planner.set_start(self.get_robot_position())
        _ = self.path_planner.plan()
        self.get_logger().info('Path has been planned.')

    def publish_waypoint(self, waypoint: dict) -> None:
        assert waypoint is not None, 'The waypoint is None'

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

    def get_robot_position(self) -> tuple:
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return transform.transform.translation
        except Exception as e:
            self.get_logger().error('Failed to get the robot position: %s' % str(e))
            return None


def main():
    rclpy.init()
    node = NavigationPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
