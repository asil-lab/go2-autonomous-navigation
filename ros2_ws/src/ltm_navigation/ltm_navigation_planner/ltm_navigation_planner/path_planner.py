""" This script is responsible for planning the path for the robot to follow along the waypoints.
Author: Alexander James Becoy
Revision: 1.0
Date: 19-08-2024
"""

import numpy as np
import pyomo.environ as pyo

"""TODO: 
- [ ] Implement the A* algorithm.
- [ ] Visiting all of the waypoints is equivalent to solving the Traveling Salesman Problem (TSP).
    - [ ] Implement the TSP algorithm: https://www.routific.com/blog/travelling-salesman-problem. 
"""

class Vertex:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def __eq__(self, other) -> bool:
        return self.x == other.x and self.y == other.y
    
    def __str__(self) -> str:
        return f'The vertex is at ({self.x}, {self.y})'
    
    def __repr__(self) -> str:
        return f'({self.x}, {self.y})'
    
    def __hash__(self) -> int:
        return hash((self.x, self.y))


class Edge:
    def __init__(self, a: Vertex, b: Vertex, weight=1.0) -> None:
        self.vertex_a = a
        self.vertex_b = b
        self.weight = weight
        self.distance = self.calculate_distance(a, b)

    def __str__(self) -> str:
        return f'The edge is from {self.vertex_a} to {self.vertex_b} with a distance of {self.distance}'
    
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
    
    def add_vertex(self, vertex: Vertex) -> None:
        if vertex in self.vertices:
            return
        self.vertices.append(vertex)
        self.adjacency_list[vertex] = []
    
    def add_edge(self, edge: Edge) -> None:
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
        return self.vertices.index(self.get_vertex(vertex.x, vertex.y))


class TSPSolver:

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

    def get_distance(self, model, i, j) -> float:
        return self.graph.get_distance(i, j)
    
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


class PathPlanner:

    def __init__(self, graph: Graph):
        self.graph = graph

    def set_start(self, x: int, y: int) -> None:
        self.start = self.graph.get_vertex(x, y)

    
    