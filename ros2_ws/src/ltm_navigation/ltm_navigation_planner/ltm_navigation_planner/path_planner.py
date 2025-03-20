""" This script is responsible for planning the path for the robot to follow along the waypoints.
Author: Alexander James Becoy
Revision: 1.0
Date: 19-08-2024
"""

import numpy as np

from scipy.spatial import KDTree
import networkx as nx

class PathPlanner:

    def __init__(self) -> None:
        self.graph = None
        self.start = None
        self.path = None
        self.waypoints = None
        self.resolution = None
        self.offset = None

    def set_waypoints(self, waypoints: np.ndarray, resolution: float, distance=0.0) -> None:
        # Set the waypoints and create a tree for the waypoints
        self.waypoints = waypoints
        self.resolution = resolution
        tree = KDTree(waypoints)

        # Set the offset for the waypoints is described as number of waypoints in a distance
        # Default is 0.0, which means we take all the waypoints
        self.offset = int(distance / resolution)

        # Initialize the graph
        self.graph = nx.Graph()

        # Add the waypoints as nodes to the graph
        for i, waypoint in enumerate(waypoints):
            self.graph.add_node(i, pos=tuple(waypoint))

        # Recursive function to link nodes based on the map's resolution
        def link_nodes(current_index, visited):
            visited.add(current_index)
            current_point = waypoints[current_index]

            # Query for points within the resolution from the current point
            WEIGHT = 1.5
            indices = tree.query_ball_point(current_point, resolution * WEIGHT)

            # Loop over the found indices
            for index in indices:
                if index in visited:
                    continue

                if index == current_index:
                    continue

                # Add an edge between the current node and the found node
                self.graph.add_edge(current_index, index)

                # Recursively link the found node
                link_nodes(index, visited)

        # Start linking the nodes
        visited = set()
        link_nodes(0, visited)

    def set_start(self, x: float, y: float) -> None:
        # Store the starting position of the robot
        self.start = np.array([x, y])

    def plot_graph(self) -> None:
        import matplotlib.pyplot as plt

        # Plot the graph
        pos = nx.get_node_attributes(self.graph, 'pos')
        nx.draw(self.graph, pos, with_labels=False, node_color='lightblue', node_size=5)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        plt.show()

    def plan(self) -> np.ndarray:
        assert self.start is not None, 'The starting position of the robot has not been set'
        assert self.graph is not None, 'The graph has not been created'

        # Instantiate sets of nodes, and leaf nodes
        visited_nodes = set()
        visited_leaf_nodes = set()
        leaf_nodes = self.find_leaf_nodes()

        # Find the nearest leaf node to the starting position
        starting_leaf_node = self.find_nearest_leaf_node(self.start, leaf_nodes)

        # Loop until all leaf nodes have been visited
        while len(visited_leaf_nodes) < len(leaf_nodes):

            # Update the visited leaf nodes
            visited_leaf_nodes.add(starting_leaf_node)
            leaf_nodes.remove(starting_leaf_node)

            # Get the nearest leaf node from the current node
            next_leaf_node = self.find_nearest_leaf_node(self.waypoints[starting_leaf_node], leaf_nodes)

            # Get the path from the current node to the next leaf node
            path = self.get_path(starting_leaf_node, next_leaf_node)

            # Iterate over every node in the path
            for node in path[::self.offset]:
                if node in visited_nodes:
                    continue
                visited_nodes.add(node)

            # Update the starting leaf node
            starting_leaf_node = next_leaf_node

        # Convert the visited nodes to array (n, 2) format
        self.path = np.array([self.waypoints[node] for node in visited_nodes])
        return path[::2]

    def find_leaf_nodes(self) -> list:
        leaf_nodes = [node for node in self.graph.nodes if self.graph.degree(node) == 1]
        return leaf_nodes
    
    def find_nearest_leaf_node(self, current_position: np.ndarray, leaf_nodes: list) -> int:
        nearest_leaf_node = min(leaf_nodes, key=lambda x: np.linalg.norm(current_position - np.array(self.graph.nodes[x]['pos'])))
        return nearest_leaf_node
    
    def get_path(self, source, target) -> list:
        path = nx.shortest_path(self.graph, source=source, target=target)
        return path
    
    def get_next_waypoint(self) -> np.ndarray:
        if self.path is None:
            print('The path has not been planned.')
            return None
        
        if len(self.path) == 0:
            print('The path is completed.')
            return None

        waypoint = self.path[0]
        self.path = self.path[1:]
        return waypoint
