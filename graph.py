from random import randrange
from copy import deepcopy
import sys # Library for INT_MAX


class UndirectedGraph:
    def __init__(self, n=0, m=0):
        self._vertices = set()
        self._neighbours = dict()
        self._cost = dict()
        for i in range(n):
            self.add_vertex(i)
        for j in range(m):
            vertex1 = randrange(n)
            vertex2 = randrange(n)
            while self.is_edge(vertex1, vertex2):
                vertex1 = randrange(n)
                vertex2 = randrange(n)
            self.add_edge(vertex1, vertex2, randrange(1000000))

    def vertices_iterator(self):
        """
        Returns an iterator to the set of vertices.
        """
        for vertex in self._vertices:
            yield vertex

    def neighbours_iterator(self, vertex):
        """
        Returns an iterator to the set of (outbound) neighbours of a vertex.
        """
        if not self.is_vertex(vertex):
            raise Exception("Invalid vertex.")
        for neighbour in self._neighbours[vertex]:
            yield neighbour

    def edges_iterator(self):
        """
        Returns an iterator to the set of edges.
        """
        for key, value in self._cost.items():
            yield key[0], key[1], value

    def is_vertex(self, vertex):
        """
        Returns True if vertex belongs to the graph.
        """
        return vertex in self._vertices

    def is_edge(self, vertex1, vertex2):
        """
        Returns True if the edge from vertex1 to vertex2 belongs to the graph.
        """
        if vertex1 > vertex2:
            vertex1, vertex2 = vertex2, vertex1
        return vertex1 in self._neighbours and vertex2 in self._neighbours[vertex1]

    def count_vertices(self):
        """
        Returns the number of vertices in the graph.
        """
        return len(self._vertices)

    def count_edges(self):
        """
        Returns the number of edges in the graph.
        """
        return len(self._cost)

    def degree(self, vertex):
        """
        Returns the number of edges with the start point vertex.
        """
        if vertex not in self._neighbours:
            raise Exception("The specified vertex does not exist.")
        return len(self._neighbours[vertex])

    def get_edge_cost(self, vertex1, vertex2):
        """
        Returns the cost of an edge if it exists.
        """
        if vertex1 > vertex2:
            vertex1, vertex2 = vertex2, vertex1
        if (vertex1, vertex2) not in self._cost:
            raise Exception("The specified edge does not exist.")
        return self._cost[(vertex1, vertex2)]

    def set_edge_cost(self, vertex1, vertex2, new_cost):
        """
        Sets the cost of an edge in the graph if it exists.
        """
        if vertex1 > vertex2:
            vertex1, vertex2 = vertex2, vertex1
        if (vertex1, vertex2) not in self._cost:
            return -1
        self._cost[(vertex1, vertex2)] = new_cost

    def add_vertex(self, vertex):
        """
        Adds a vertex to the graph.
        """
        if self.is_vertex(vertex):
            raise Exception("Cannot add a vertex which already exists.")
        self._vertices.add(vertex)
        self._neighbours[vertex] = set()

    def add_edge(self, vertex1, vertex2, edge_cost=0):
        """
        Adds an edge to the graph.
        """
        if vertex1 > vertex2:
            vertex1, vertex2 = vertex2, vertex1
        if self.is_edge(vertex1, vertex2):
            return -1
        if not self.is_vertex(vertex1) or not self.is_vertex(vertex2):
            return -1
        self._neighbours[vertex1].add(vertex2)
        self._neighbours[vertex2].add(vertex1)
        self._cost[(vertex1, vertex2)] = edge_cost

    def remove_edge(self, vertex1, vertex2):
        """
        Removes an edge from the graph.
        """
        if vertex1 > vertex2:
            vertex1, vertex2 = vertex2, vertex1
        if not self.is_edge(vertex1, vertex2):
            raise Exception("The specified edge does not exist.")
        del self._cost[(vertex1, vertex2)]
        self._neighbours[vertex1].remove(vertex2)
        self._neighbours[vertex2].remove(vertex1)

    def remove_vertex(self, vertex):
        """
        Removes a vertex from the graph.
        """
        if not self.is_vertex(vertex):
            raise Exception("Cannot remove a vertex which doesn't exist.")
        to_remove = []
        for node in self._neighbours[vertex]:
            to_remove.append(node)
        for node in to_remove:
            self.remove_edge(vertex, node)
        del self._neighbours[vertex]
        self._vertices.remove(vertex)

    def copy(self):
        """
        Returns a deepcopy of the graph.
        """
        return deepcopy(self)

    # A utility function to print the constructed MST stored in parent[]
    def printMST(self, parent):
        print ("Edge \tWeight")
        for i in range(1, self.V):
            print (parent[i], "-", i, "\t", self.graph[i][parent[i]])

    # A utility function to find the vertex with
    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def minKey(self, key, mstSet):

        # Initialize min value
        min = sys.maxsize

        for v in range(self.count_vertices()):
            if key[v] < min and mstSet[v] == False:
                min = key[v]
                min_index = v

        return min_index

    # Function to construct and print MST for a graph
    # represented using adjacency matrix representation
    def primMST(self):

        # Key values used to pick minimum weight edge in cut
        key = [sys.maxsize] * self.count_vertices()
        parent = [None] * self.count_vertices()  # Array to store constructed MST
        # Make key 0 so that this vertex is picked as first vertex
        key[0] = 0
        mstSet = [False] * self.count_vertices()

        parent[0] = -1  # First node is always the root of

        for cout in range(self.count_vertices()):

            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # u is always equal to src in first iteration
            u = self.minKey(key, mstSet)

            # Put the minimum distance vertex in
            # the shortest path tree
            mstSet[u] = True

            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for v in range(self.V):

                # graph[u][v] is non zero only for adjacent vertices of m
                # mstSet[v] is false for vertices not yet included in MST
                # Update the key only if graph[u][v] is smaller than key[v]
                if self.graph[u][v] > 0 and mstSet[v] == False and key[v] > self.graph[u][v]:
                    key[v] = self.graph[u][v]
                    parent[v] = u

        self.printMST(parent)


class Graph():

    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]

    def printMST(self, parent):
        print ("Edge \tWeight")
        for i in range(1, self.V):
            print (parent[i], "-", i, "\t", self.graph[i][parent[i]])

    # Find the vertex with the minimum distance value, from the set of vertices not yet included in shortest path tree
    def minKey(self, key, mstSet):
        min = sys.maxsize
        for v in range(self.V):
            if key[v] < min and mstSet[v] is False:
                min = key[v]
                min_index = v

        return min_index

    def primMST(self):
        key = [sys.maxsize] * self.V
        parent = [None] * self.V  # Array to store constructed MST
        mstSet = [False] * self.V
        # First vertex is 0
        key[0] = 0
        parent[0] = -1  # First node is the root of MST
        for c in range(self.V):
            u = self.minKey(key, mstSet)
            mstSet[u] = True
            for v in range(self.V):
                if self.graph[u][v] > 0 and mstSet[v] == False and key[v] > self.graph[u][v]:
                    key[v] = self.graph[u][v]
                    parent[v] = u

        self.printMST(parent)

