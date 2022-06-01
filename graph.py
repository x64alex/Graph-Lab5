from random import randrange
from copy import deepcopy
from queue import PriorityQueue


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

    def print_mst(self, parent):
        print("Edge \tWeight")
        for i in range(0, len(parent)):
            print(parent[i][0], "-", parent[i][1], "\t", self.get_edge_cost(parent[i][0], parent[i][1]))

    def prim_mst(self):
        edges = []
        q = PriorityQueue()
        prev = {}
        dist = {}
        vertices = {0}
        for x in self._neighbours[0]:
            dist[x] = self.get_edge_cost(x, 0)
            prev[x] = 0
            q.put((dist[x], x))

        while not q.empty():
            x = q.get()[1]
            if x not in vertices:
                edges.append([prev[x], x])
                vertices.add(x)
                for y in self._neighbours[x]:
                    if y not in dist.keys() or self.get_edge_cost(x, y) < dist[y]:
                        dist[y] = self.get_edge_cost(x, y)
                        q.put((dist[y], y))
                        prev[y] = x

        return edges

    def ham(self):
        mst = self.prim_mst()
        path = []

        # Construct a mst graph
        g = UndirectedGraph(len(self._vertices))
        for edge in mst:
            g.add_edge(edge[0], edge[1], self.get_edge_cost(edge[0], edge[1]))

        # doing a bft with preorder to construct the  path
        q = [0]
        visited = {0}
        path.append(0)
        while len(q) != 0:
            el = q.pop(0)
            for c in self._neighbours[el]:
                if c not in visited:
                    visited.add(c)
                    path.append(c)
                    q.append(c)

        path.append(0)
        return path

    def print_ham(self):
        ham = self.ham()
        s = "A Hamilton cycle is: "

        for n in ham:
            s += str(n)
            s += "-"

        s = s[:-1]

        print(s)

# Useful links: https://www.geeksforgeeks.org/travelling-salesman-problem-set-2-approximate-using-mst/
# http://www.cs.uni.edu/~fienup/cs270s04/lectures/lec29_4-27-04.htm
# https://iq.opengenus.org/approximation-algorithm-for-travelling-salesman-problem/

