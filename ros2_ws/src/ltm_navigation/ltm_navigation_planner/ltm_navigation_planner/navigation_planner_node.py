""" This is the main node for the ltm_navigation_planner package.
Author: Alexander James Becoy
Revision: 1.0
Date: 19-08-2024
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseArray


class NavigationPlannerNode(Node):
    """ TODO: 
    - [ ] Read the map from the map server.
    - [ ] Get the area to be explored that is defined by the obstacle map (in the form of a polygon).
    - [ ] Create waypoints along the boundary of the area to be explored.
    - [ ] Get the current position of the robot.
    - [ ] Plan the optimal path to visit all the waypoints.
    - [ ] Publish the next waypoint according to the state machine.
    """

    def __init__(self):
        super().__init__('navigation_planner_node')
        self.get_logger().info('Navigation Planner Node has been initialized.')

        # Create the subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)


def main():
    rclpy.init()
    node = NavigationPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
