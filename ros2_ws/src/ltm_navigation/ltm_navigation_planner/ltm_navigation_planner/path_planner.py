""" This script is responsible for planning the path for the robot to follow along the waypoints.
Author: Alexander James Becoy
Revision: 1.0
Date: 19-08-2024
"""

from collections import deque
import numpy as np
import pyomo.environ as pyo

from scipy.spatial import KDTree
import networkx as nx

"""TODO: 
- [ ] Implement the A* algorithm.
- [ ] Visiting all of the waypoints is equivalent to solving the Traveling Salesman Problem (TSP).
    - [ ] Implement the TSP algorithm: https://www.routific.com/blog/travelling-salesman-problem. 
"""

class Vertex:
    def __init__(self, id: int, x: float, y: float, yaw=0.0) -> None:
        self.id = id
        self.x = x
        self.y = y
        self.yaw = yaw

    def __eq__(self, other) -> bool:
        return self.id == other.id
    
    def __str__(self) -> str:
        return f'({self.x}, {self.y}, {self.yaw})'
    
    def __repr__(self) -> str:
        return f'({self.x}, {self.y}, {self.yaw})'
    
    def __hash__(self) -> int:
        return hash((self.x, self.y, self.yaw))


class Edge:
    def __init__(self, a: Vertex, b: Vertex, weight=1.0) -> None:
        self.vertex_a = a
        self.vertex_b = b
        self.weight = weight
        self.distance = self.calculate_distance(a, b)

    def __str__(self) -> str:
        return f'{self.start} -> {self.end}'
    
    def __repr__(self) -> str:
        return f'{self.start} -> {self.end}'

    def calculate_distance(self, a: Vertex, b: Vertex) -> float:
        return np.sqrt((b.x - a.x)**2 + (b.y - a.y)**2)
    

class Graph:
    def __init__(self):
        self.vertices = []
        self.edges = []
        self.adjacency_list = {}

    def __str__(self) -> str:
        return f'The graph has {len(self.vertices)} vertices and {len(self.edges)} edges'
    
    def add_point(self, x: float, y: float, yaw=0.0) -> None:
        vertex = Vertex(len(self.vertices), x, y, yaw)
        self.add_vertex(vertex)

    def add_vertex(self, vertex: Vertex) -> None:
        # Check if the vertex already exists
        if vertex in self.vertices:
            print('Vertex %s already exists in the graph' % vertex)
            return
        
        # Add the vertex to the graph, and create an empty adjacency list for the vertex
        self.vertices.append(vertex)
        self.adjacency_list[vertex] = []
    
    def add_edge(self, edge: Edge) -> None:
        # Check if the edge already exists
        if edge in self.edges:
            print('Edge %s already exists in the graph' % edge)
            return
        
        # Check if the vertices of the edge are in the graph
        if edge.vertex_a not in self.vertices or edge.vertex_b not in self.vertices:
            print('Vertices %s and %s of edge %s are not in the graph' % (edge.vertex_a, edge.vertex_b, edge))
            return

        # Add the edge to the graph and adjacency list
        self.edges.append(edge)
        self.adjacency_list[edge.vertex_a].append(edge)
        self.adjacency_list[edge.vertex_b].append(edge)
    
    def get_vertex(self, x: float, y: float) -> Vertex:
        for vertex in self.vertices:
            if np.isclose(vertex.x, x) and np.isclose(vertex.y, y):
                return vertex
        return None
    
    def get_edge(self, a: Vertex, b: Vertex) -> Edge:
        for edge in self.edges:
            if edge.vertex_a == a and edge.vertex_b == b:
                return edge
            if edge.vertex_a == b and edge.vertex_b == a:
                return edge
        return None
    
    def get_neighbors(self, vertex: Vertex) -> list:
        return self.adjacency_list[vertex]
    
    def get_distance(self, a: Vertex, b: Vertex) -> float:
        # return self.get_edge(a, b).distance
        return np.sqrt((b.x - a.x)**2 + (b.y - a.y)**2)
    
    def get_vertices(self) -> list:
        return self.vertices
    
    def get_edges(self) -> list:
        return self.edges
    
    def get_adjacency_list(self) -> dict:
        return self.adjacency_list
    
    def get_num_vertices(self) -> int:
        return len(self.vertices)
    
    def get_num_edges(self) -> int:
        return len(self.edges)
    
    def get_num_neighbors(self, vertex: Vertex) -> int:
        return len(self.adjacency_list[vertex])
    
    def get_vertex_index(self, vertex: Vertex) -> int:
        return self.vertices.id


class TSPSolver:
    """The Traveling Salesman Problem (TSP) solver class using Pyomo and GLPK: http://www.opl.ufc.br/post/tsp/.
    The solver is a whole linear programming model that solves the TSP problem for a given number of vertices.

    NOTE: The solver uses the Miller-Tucker-Zemlin (MTZ) formulation to solve the TSP problem which is not the
    most computationally efficient method. However, it is a good starting point for solving the TSP problem and
    has ECS-coding style.

    TODO: Look into more efficient TSP algorithms like the Christofides algorithm: https://en.wikipedia.org/wiki/Christofides_algorithm.
    """

    def __init__(self, graph: Graph) -> None:
        self.graph = graph
        self.construct_model()

    def solve(self) -> None:
        solver = pyo.SolverFactory('glpk')
        solver.solve(self.model, tee=False)

    def get_solution(self) -> list:
        solution = []
        for i in list(self.model.x.keys()):
            if self.model.x[i]() != 0 and self.model.x[i]() != None:
                solution.append(i)
        return solution
    
    def get_path(self, start: Vertex) -> list:
        path = [self.graph.get_vertex_index(start) + 1]
        solution = self.get_solution()
        for _ in solution:
            for ii in solution:
                last_vertex = path[-1]
                if ii[0] == last_vertex:
                    if ii[1] == path[0]:
                        # Stop if we have reached the start
                        break
                    path.append(ii[1])
        return [self.graph.vertices[i-1] for i in path]
    
    def construct_model(self) -> None:
        self.model = pyo.ConcreteModel()
        self.define_variables()
        self.define_cost()
        self.define_objective()
        self.define_constraints()
    
    def define_variables(self) -> None:
        # Define the sets
        self.model.M = pyo.RangeSet(1, self.graph.get_num_vertices())
        self.model.N = pyo.RangeSet(1, self.graph.get_num_vertices())

        # Define the dummy variable u
        self.model.U = pyo.RangeSet(2, self.graph.get_num_vertices())

        # Define the binary variable x_ij
        self.model.x = pyo.Var(self.model.N, self.model.M, within=pyo.Binary)

        # Define the variable u_i
        self.model.u = pyo.Var(self.model.N, within=pyo.NonNegativeIntegers, bounds=(0, self.graph.get_num_vertices() - 1))

    def define_cost_matrix(self) -> np.ndarray:
        cost_matrix = np.zeros((self.graph.get_num_vertices(), self.graph.get_num_vertices()), dtype=np.float64)
        for i, vertex_a in enumerate(self.graph.vertices):
            for j, vertex_b in enumerate(self.graph.vertices):
                cost_matrix[i, j] = self.graph.get_distance(vertex_a, vertex_b)
        return cost_matrix

    def define_cost(self) -> None:
        self.cost_matrix = self.define_cost_matrix()
        self.model.c = pyo.Param(self.model.N, self.model.M, initialize=lambda model, i, j: self.cost_matrix[i-1, j-1])

    def define_objective(self) -> None:
        def objective_rule(model):
            return sum(model.c[i, j] * model.x[i, j] for i in model.N for j in model.M)
        self.model.objective = pyo.Objective(rule=objective_rule, sense=pyo.minimize)

    def define_constraints(self) -> None:
        def one_in_one_out_rule(model, M):
            return sum(model.x[i, M] for i in model.N if i != M) == 1
        self.model.one_in_one_out = pyo.Constraint(self.model.M, rule=one_in_one_out_rule)

        def one_out_one_in_rule(model, N) -> None:
            return sum(model.x[N, j] for j in model.M if j != N) == 1
        self.model.one_out_one_in = pyo.Constraint(self.model.N, rule=one_out_one_in_rule)

        def subtour_elimination_rule(model, i, j) -> None:
            if i != j:
                return model.u[i] - model.u[j] + self.graph.get_num_vertices() * model.x[i, j] <= self.graph.get_num_vertices() - 1
            else:
                return model.u[i] - model.u[j] == 0
        
        self.model.subtour_elimination = pyo.Constraint(self.model.U, self.model.N, rule=subtour_elimination_rule)


# class PathPlanner:

#     def __init__(self) -> None:
#         self.graph = Graph()
#         self.start = None
#         self.path = None

#     def set_start(self, x: float, y: float) -> None:
#         # Set the starting position of the robot as a vertex in the graph
#         self.start = Vertex(x, y)
#         self.graph.add_vertex(self.start)

#     def set_waypoints(self, waypoints: np.ndarray) -> None:
#         # Convert the waypoints (y, x yaw) as vertices, and add them to the graph
#         vertices = [Vertex(waypoint[1], waypoint[0]) for waypoint in waypoints]
#         self.construct_graph(vertices)
    
#     def construct_graph(self, vertices: list) -> None:
#         # Add the vertices to the graph
#         for vertex in vertices:
#             self.graph.add_vertex(vertex)

#     def plan(self) -> np.ndarray:
#         assert self.start is not None, 'The starting position of the robot has not been set'
#         assert self.graph.get_num_vertices() > 0, 'The graph has no vertices'

#         # # Solve the TSP problem for the graph
#         # tsp_solver = TSPSolver(self.graph)
#         # tsp_solver.solve()
#         # self.path = deque(tsp_solver.get_path(self.start)[1:]) # Skip the first waypoint which is the starting position
#         # return self.path

#         # Hardcoded path for testing
#         self.path = deque(self.graph.get_vertices())
#         return self.path

#     def get_next_waypoint(self) -> Vertex:
#         if self.path is None:
#             print('The path has not been planned.')
#             return None
        
#         if len(self.path) == 0:
#             print('The path is completed.')
#             return None

#         waypoint = self.path.popleft()
#         return {
#             'x': waypoint.x,
#             'y': waypoint.y,
#         }
    
#     def get_start(self) -> Vertex:
#         return self.start
    
#     def get_distance_between_waypoints(self, a: Vertex, b: Vertex) -> float:
#         return np.sqrt((b.x - a.x)**2 + (b.y - a.y)**2)

#     def get_orientation_between_waypoints(self, a: Vertex, b: Vertex) -> float:
#         return np.arctan2(b.y - a.y, b.x - a.x)
    
#     def get_orientation_to_waypoint(self, waypoint: Vertex) -> float:
#         return self.get_orientation_between_waypoints(self.start, waypoint)

class PathPlanner:

    def __init__(self) -> None:
        self.graph = None
        self.start = None
        self.path = None
        self.waypoints = None

    def set_waypoints(self, waypoints: np.ndarray, resolution: float) -> None:
        # Set the waypoints and create a tree for the waypoints
        self.waypoints = waypoints
        tree = KDTree(waypoints)

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
            for node in path[::5]:
                if node in visited_nodes:
                    continue
                visited_nodes.add(node)

            # Update the starting leaf node
            starting_leaf_node = next_leaf_node

        # Convert the visited nodes to array (n, 2) format
        self.path = np.array([self.waypoints[node] for node in visited_nodes])
        return path

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
    
    # def get_distance_between_waypoints(self, a: np.ndarray, b: np.ndarray) -> float:
    #     return np.linalg.norm(b - a)
    
    # def get_orientation_between_waypoints(self, a: np.ndarray, b: np.ndarray) -> float:
    #     return np.arctan2(b[1] - a[1], b[0] - a[0])
    
    # def get_orientation_to_waypoint(self, waypoint: np.ndarray) -> float:
    #     return self.get_orientation_between_waypoints(self.start, waypoint)
