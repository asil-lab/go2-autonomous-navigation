""" This is the main node for the ltm_navigation_planner package.
Author: Alexander James Becoy
Revision: 1.0
Date: 03-10-2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray, Marker

class NavigationVisualizerNode(Node):
    def __init__(self):
        super().__init__('navigation_visualizer_node')
        
        self.initialize_all_waypoints_subscriber()
        self.initialize_visualized_waypoints_publisher()
        
        self.get_logger().info('Navigation Visualizer Node has been initialized.')
        
    def all_waypoints_callback(self, msg):
        markers = MarkerArray()

        for i, pose in enumerate(msg.poses):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            markers.markers.append(marker)

        self.visualized_waypoints_publisher.publish(markers)
        self.get_logger().info('All Waypoints have been published to Visualized Waypoints.')

    def initialize_all_waypoints_subscriber(self):
        qos_profile = QoSProfile(depth=10)
        self.all_waypoints_subscriber = self.create_subscription(
            PoseArray, 'all_waypoints', self.all_waypoints_callback, qos_profile)
        self.get_logger().info('All Waypoints Subscriber has been initialized.')

    def initialize_visualized_waypoints_publisher(self):
        qos_profile = QoSProfile(depth=10)
        self.visualized_waypoints_publisher = self.create_publisher(
            MarkerArray, 'visualized_waypoints', qos_profile)
        self.get_logger().info('Visualized Waypoints Publisher has been initialized.')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationVisualizerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
