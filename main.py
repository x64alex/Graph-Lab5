from graph import UndirectedGraph
from graph import Graph
from random import randrange
from queue import Queue


def connected_components(graph):
    met = set()
    ans = []
    for vertex in graph.vertices_iterator():
        if vertex not in met:
            q = Queue()
            q.put(vertex)
            met.add(vertex)
            ans.append([vertex])

            while not q.empty():
                node = q.get()
                for neighbour in graph.neighbours_iterator(node):
                    if neighbour not in met:
                        met.add(neighbour)
                        q.put(neighbour)
                        ans[-1].append(neighbour)
    return ans


def read_file(file_path):
    file = open(file_path, "r")
    n, m = map(int, file.readline().split())
    g = UndirectedGraph(n)
    for _ in range(m):
        vertex1, vertex2, edge_cost = map(int, file.readline().split())
        g.add_edge(vertex1, vertex2, edge_cost)
    file.close()
    return g


def write_file(g, file_path):
    file = open(file_path, "w")
    file.write("{0} {1}\n".format(g.count_vertices(), g.count_edges()))
    for node in g.vertices_iterator():
        for neighbour in g.neighbours_iterator(node):
            file.write("{0} {1} {2}\n".format(node, neighbour, g.get_edge_cost(node, neighbour)))
    file.close()


def random_graph(vertices_no, edges_no):
    g = UndirectedGraph()
    for i in range(vertices_no):
        g.add_vertex(i)
    for j in range(edges_no):
        vertex1 = randrange(vertices_no)
        vertex2 = randrange(vertices_no)
        while g.is_edge(vertex1, vertex2):
            vertex1 = randrange(vertices_no)
            vertex2 = randrange(vertices_no)
        g.add_edge(vertex1, vertex2, randrange(1000000))

    return g

def toMatrix(graph):
    g = [[0 for column in range(graph.count_vertices())]
             for row in range(graph.count_vertices())]

    for v in graph.vertices_iterator():
        for c in graph.neighbours_iterator(v):
            g[v][c] = graph.get_edge_cost(v,c)
            g[c][v] = graph.get_edge_cost(v,c)

    return g

class Console:

    def __init__(self):
        self.__graph = UndirectedGraph()

    def __get_number_of_vertices(self):
        self.__graph.count_vertices()

    def __print_all_vertices(self):
        print("The vertices of the graph are: ")
        for vertex in self.__graph.vertices_iterator():
            print(vertex)

    def __edge_from_x_to_y(self):
        print("Give vertices x and y:")
        start = int(input())
        end = int(input())
        print(self.__graph.is_edge(start, end))

    def __modify_cost(self):
        print("Give edge start:")
        start = int(input())
        print("Give edge end:")
        end = int(input())
        print("Give new cost:")
        cost = int(input())
        if self.__graph.set_edge_cost(start, end, cost) == -1:
            print("The edge does not exists!")

    def __add_vertex(self):
        print("Give new vertex:")
        vertex = int(input())
        if self.__graph.add_vertex(vertex) == -1:
            print("The vertex already exists!")

    def __add_edge(self):
        start = int(input("Give edge start:"))
        end = int(input("Give edge end:"))
        cost = int(input("Give edge cost:"))
        if self.__graph.add_edge(start, end, cost) == 0:
            print("The edge already exits!")

    def __remove_edge(self):
        start = int(input("Give edge start:"))
        end = int(input("Give edge end:"))
        if self.__graph.remove_edge(start, end) == -1:
            print("The edge does not exist!")

    def __remove_vertex(self):
        vertex = int(input("Give vertex you want to remove:"))
        if self.__graph.remove_vertex(vertex) == -1:
            print("The vertex does not exist!")

    def __print_graph(self):
        print("The vertices of the graph are: ")
        for vertex in self.__graph.vertices_iterator():
            print(vertex)
        print("The edges of the graph are: ")
        for triple in self.__graph.edges_iterator():
            print(triple[0], triple[1])

    def __read_from_file(self):
        file_name = input("Enter file name:")
        self.__graph = read_file(file_name)

    def __get_neighbours(self):
        graph = int(input("Enter graph number:"))
        print("The neighbours of the graph are: ")
        for vertex in self.__graph.neighbours_iterator(graph):
            print(vertex)

    def __write_to_file(self):
        file_name = input("Enter file name:")
        write_file(self.__graph, file_name)

    def __generate_graph(self):
        try:
            number_vertices = int(input("Enter the number of vertices:"))
            number_edges = int(input("Enter the number of edges:"))
            self.__graph = random_graph(number_vertices, number_edges)
        except ValueError:
            print("Invalid values for the vertices and edges")

    def __connected_components(self):
        print(connected_components(self.__graph))

    def __prims(self):
        g = Graph(self.__graph.count_vertices())
        g.graph = toMatrix(self.__graph)
        g.primMST()

    @staticmethod
    def print_menu():
        print("Options:\n")
        print("0.Exit the program")
        print("r.Read from a file")
        print("g.Generate random the graph")
        print("w.Write to a file")
        print("c.Connected components")
        print("p.Print the graph vertices and edges.")
        print("1.Get the number of vertices")
        print("2.See all vertices")
        print("3.See if there is an edge from <x> to <y>")
        print("4.Print the neighbours of a vertex")
        print("5.Modify the cost of an edge")
        print("6.Add a vertex")
        print("7.Add an edge")
        print("8.Remove a vertex")
        print("9.Remove an edge")
        print("10.Minimal spanning tree using the Prim's algorithm")

    def run(self):
        while True:
            self.print_menu()
            cmd = input("Enter option:")
            if cmd == "0":
                return
            if cmd == "r":
                self.__read_from_file()
            if cmd == "g":
                self.__generate_graph()
            if cmd == "w":
                self.__write_to_file()
            if cmd == "c":
                self.__connected_components()
            if cmd == "1":
                self.__get_number_of_vertices()
            if cmd == "2":
                self.__print_all_vertices()
            if cmd == "3":
                self.__edge_from_x_to_y()
            if cmd == "4":
                self.__get_neighbours()
            if cmd == "5":
                self.__modify_cost()
            if cmd == "6":
                self.__add_vertex()
            if cmd == "7":
                self.__add_edge()
            if cmd == "8":
                self.__remove_vertex()
            if cmd == "9":
                self.__remove_edge()
            if cmd == "p":
                self.__print_graph()
            if cmd == "10":
                self.__prims()


c = Console()
c.run()
