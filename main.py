"""
This program is to find shortest path between source and destination in undirected weighted graph
using dijkstra algorithm
"""
from collections import defaultdict
import math


class Vertex:
    def __init__(self, v_id, latitude, longitude):
        self.id = v_id
        self.latitude = latitude
        self.longitude = longitude


class Graph:
    def __init__(self):
        self.adjList = defaultdict(list)
        # key -> id | value -> vertex
        self.vertexes = {}
        # key -> tuple src and dest vertex | value -> edge traffic
        self.edges = {}

    def addEdge(self, src, dest):
        self.adjList[src].append(dest)
        self.adjList[dest].append(src)

    def edgeWeight(self, src_id, dest_id):
        src = self.vertexes[src_id]
        dest = self.vertexes[dest_id]
        length = math.dist([src.longitude, src.latitude], [dest.longitude, dest.latitude])
        traffic_factor = 0.3
        traffic = self.edges.get((src_id, dest_id), -1)
        if traffic == -1:
            traffic = self.edges[(dest_id, src_id)]
        weight = length * (1 + (traffic_factor * traffic))
        return weight

    def increaseTraffic(self, src_id, dest_id):
        traffic = self.edges.get((src_id, dest_id), -1)
        if traffic == -1:
            traffic = self.edges[(dest_id, src_id)]
            self.edges[(dest_id, src_id)] = traffic + 1
        else:
            self.edges[(src_id, dest_id)] = traffic + 1

    def decreaseTraffic(self, src_id, dest_id):
        traffic = self.edges.get((src_id, dest_id), -1)
        if traffic == -1:
            traffic = self.edges[(dest_id, src_id)]
            if traffic > 0:
                self.edges[(dest_id, src_id)] = traffic - 1
        else:
            if traffic > 0:
                self.edges[(src_id, dest_id)] = traffic - 1

    # if flag is true increase the traffic else decrease the traffic
    def updateTraffic(self, path, flag):
        for k in range(len(path)):
            if k == len(path) - 1:
                break
            if flag:
                self.increaseTraffic(path[k], path[k + 1])
            else:
                self.decreaseTraffic(path[k], path[k + 1])


class HeapNode:
    def __init__(self, vertex_id, distance):
        self.v = vertex_id
        self.dist = distance


class MinHeap:
    def __init__(self, size):
        self.arr = []
        self.size = size
        self.nodePos = {}

    def minHeapify(self, index):
        smallest = index
        left = 2 * index + 1
        right = 2 * index + 2
        if left < self.size and self.arr[left].dist < self.arr[smallest].dist:
            smallest = left

        if right < self.size and self.arr[right].dist < self.arr[smallest].dist:
            smallest = right

        if smallest != index:
            self.nodePos[self.arr[smallest].v] = index
            self.nodePos[self.arr[index].v] = smallest
            self.arr[smallest], self.arr[index] = self.arr[index], self.arr[smallest]
            self.minHeapify(smallest)

    def extractMin(self):
        if self.size == 0:
            return
        minNode = self.arr[0]
        last = self.arr[self.size - 1]
        self.arr[0] = last
        self.nodePos[last.dist] = 0
        self.nodePos.pop(minNode.v)
        self.size -= 1
        self.minHeapify(0)
        return minNode

    def updateDistance(self, vertex, newDist):
        index = self.nodePos[vertex]
        self.arr[index].dist = newDist
        while index > 0 and self.arr[index].dist < self.arr[int((index - 1) / 2)].dist:
            self.nodePos[self.arr[index].v] = int((index - 1) / 2)
            self.nodePos[self.arr[int((index - 1) / 2)].v] = index
            self.arr[index], self.arr[int((index - 1) / 2)] = self.arr[int((index - 1) / 2)], self.arr[index]
            index = int((index - 1) / 2)

    def containsVertex(self, vertex):
        return True if self.nodePos.__contains__(vertex) else False

    def isEmpty(self):
        return True if self.size == 0 else False


class Dijkstra:
    def __init__(self, graph, src, dest):
        self.graph = graph
        self.path = []
        # key -> vertex_id | value -> distance
        self.distance = {}
        self.src = src
        self.dest = dest

    def shortestPath(self):
        parent = {}
        # set vertexes first distance to infinity
        INF = float('inf')
        heap = MinHeap(len(self.graph.vertexes))
        index = 0
        # add vertexes to min heap
        for vertex in self.graph.vertexes:
            node = HeapNode(vertex, INF)
            heap.arr.append(node)
            heap.nodePos[vertex] = index
            index += 1
        # set source vertex distance to 0
        heap.updateDistance(self.src, 0)
        self.distance[self.src] = 0
        parent[self.src] = None
        # Implement dijkstra algorithm

        while not (heap.isEmpty()):
            curr = heap.extractMin()
            key = curr.v
            self.distance[key] = curr.dist
            for adjacent in list(self.graph.adjList[key]):
                if not heap.containsVertex(adjacent):
                    continue
                newDistance = self.distance[key] + self.graph.edgeWeight(key, adjacent)
                curr_dist = heap.arr[heap.nodePos[adjacent]].dist
                if newDistance < curr_dist:
                    self.distance[adjacent] = newDistance
                    heap.updateDistance(adjacent, newDistance)
                    parent[adjacent] = key

                if key == self.dest:
                    break

        # Iterate parent list backward to find a path
        temp = parent[self.dest]
        while not (temp is None):
            self.path.append(temp)
            temp = parent[temp]
        self.path.reverse()
        self.path.append(self.dest)

    def duration(self):
        pathWeight = 0
        for k in range(len(self.path)):
            if k == len(self.path) - 1:
                break
            pathWeight += self.graph.edgeWeight(self.path[k], self.path[k + 1])
        return 120 * pathWeight


myGraph = Graph()
inputs = input().split()
n = int(inputs[0])
m = int(inputs[1])
# get vertexes
for i in range(0, n):
    inputs = input().split()
    id = inputs[0]
    y = float(inputs[1])
    x = float(inputs[2])
    v = Vertex(id, y, x)
    myGraph.vertexes[id] = v

# get edges
for i in range(0, m):
    inputs = input().split()
    start_id = inputs[0]
    end_id = inputs[1]
    myGraph.addEdge(start_id, end_id)
    myGraph.edges[(start_id, end_id)] = 0

request_time = []  # List of each request (duration + sending time)
time_path = defaultdict(list)  # key -> request sending time + path duration  | value -> shortest path
# have a list to prevent adding traffic for specific request more than once
apply_traffic = []

while True:
    inputs = input().split()
    if inputs[0] == 'exit':
        break
    time = float(inputs[0])
    start_id = inputs[1]
    destination_id = inputs[2]
    for item in request_time[:]:

        if time < item:
            # adding item to apply_traffic list if we didn't increase item traffic before
            if not (item in apply_traffic):
                apply_traffic.append(item)
                myGraph.updateTraffic(time_path[item], True)
        else:
            myGraph.updateTraffic(time_path[item], False)
            request_time.remove(item)
            time_path.pop(item)
            if item in apply_traffic:
                apply_traffic.remove(item)
    dijkstra = Dijkstra(myGraph, start_id, destination_id)
    dijkstra.shortestPath()
    path_duration = dijkstra.duration()
    print('Path:', dijkstra.path)
    print('Duration:', path_duration)
    request_time.append(path_duration + time)
    time_path[path_duration + time] = dijkstra.path
