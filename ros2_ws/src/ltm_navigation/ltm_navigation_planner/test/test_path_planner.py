""" Test for ltm_navigation_planner/path_planner.py
Revision: 1.0
Date: 20-08-2024
"""

import pytest
import numpy as np

from ltm_navigation_planner.path_planner import Vertex, Edge, Graph, TSPSolver, PathPlanner

def test_vertex():
    vertex = Vertex(1.0, 2.0)
    assert vertex.x == 1.0
    assert vertex.y == 2.0

def test_edge():
    vertex_a = Vertex(1.0, 2.0)
    vertex_b = Vertex(3.0, 4.0)
    edge = Edge(vertex_a, vertex_b)
    assert edge.vertex_a == vertex_a
    assert edge.vertex_b == vertex_b
    assert edge.distance == np.sqrt((3.0 - 1.0)**2 + (4.0 - 2.0)**2)

def test_graph():
    # Create a graph
    graph = Graph()

    # Add vertices and edges to the graph
    vertex_a = Vertex(1.0, 2.0)
    vertex_b = Vertex(3.0, 4.0)
    vertex_c = Vertex(5.0, 6.0)
    graph.add_vertex(vertex_a)
    graph.add_vertex(vertex_b)
    graph.add_vertex(vertex_c)

    edge_ab = Edge(vertex_a, vertex_b)
    edge_bc = Edge(vertex_b, vertex_c)
    graph.add_edge(edge_ab)
    graph.add_edge(edge_bc)

    # Test the graph
    assert graph.vertices == [vertex_a, vertex_b, vertex_c]
    assert graph.edges == [edge_ab, edge_bc]
    assert graph.adjacency_list[vertex_a] == [edge_ab]
    assert graph.adjacency_list[vertex_b] == [edge_ab, edge_bc]
    assert graph.adjacency_list[vertex_c] == [edge_bc]
    assert graph.get_vertex(1.0, 2.0) == vertex_a
    assert graph.get_vertex(3.0, 4.0) == vertex_b
    assert graph.get_vertex(5.0, 6.0) == vertex_c
    assert graph.get_edge(vertex_a, vertex_b) == edge_ab
    assert graph.get_edge(vertex_b, vertex_c) == edge_bc
    assert graph.get_neighbors(vertex_a) == [edge_ab]
    assert graph.get_neighbors(vertex_b) == [edge_ab, edge_bc]
    assert graph.get_neighbors(vertex_c) == [edge_bc]
    assert graph.get_distance(vertex_a, vertex_b) == edge_ab.distance
    assert graph.get_distance(vertex_b, vertex_c) == edge_bc.distance

def test_tsp_solver():
    # Create a graph
    graph = Graph()

    # Add vertices and edges to the graph
    vertex_a = Vertex(0.0, 0.0)
    vertex_b = Vertex(0.0, 1.0)
    vertex_c = Vertex(1.0, 2.0)
    vertex_d = Vertex(0.0, 3.0)
    vertex_e = Vertex(-1.0, 2.0)
    graph.add_vertex(vertex_a)
    graph.add_vertex(vertex_b)
    graph.add_vertex(vertex_c)
    graph.add_vertex(vertex_d)
    graph.add_vertex(vertex_e)

    # # Create a TSP solver
    tsp_solver = TSPSolver(graph)
    tsp_solver.solve()

    # # Test the TSP solver
    assert tsp_solver.get_path(vertex_a) == [vertex_a, vertex_e, vertex_d, vertex_c, vertex_b]