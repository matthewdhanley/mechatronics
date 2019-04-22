import numpy as np
import heapq

class Vertex:
    def __init__(self, node, location):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = 10000000000
        # Mark all nodes unvisited
        self.visited = False
        # Predecessor
        self.previous = None
        self.location = np.array(location)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.distance == other.distance
        return NotImplemented

    def __lt__(self, other):
        if isinstance(other, self.__class__):
            return self.distance < other.distance
        return NotImplemented

    def __hash__(self):
        return id(self)

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()

    def get_id(self):
        return self.id

    def get_location(self):
        return self.location

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def calc_distance(self, other):
        return np.sum(np.abs(other.location-self.location))

    def calc_distance_from_point(self, point):
        return np.sum(np.abs(point - self.location))

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])


class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node, location):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node, location)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to):
        if frm not in self.vert_dict:
            raise NameError
        if to not in self.vert_dict:
            raise NameError

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], self.get_vertex(frm).calc_distance(self.get_vertex(to)))
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], self.get_vertex(to).calc_distance(self.get_vertex(frm)))

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

    def get_nearest(self, location):
        nearest_node = None
        min_distance = 10000000
        for vertex in self.vert_dict:
            if self.get_vertex(vertex).calc_distance_from_point(location) < min_distance:
                nearest_node = vertex
                min_distance = self.get_vertex(vertex).calc_distance_from_point(location)
        return nearest_node


def shortest(v, path):
    """make shortest path from v.previous"""
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return


def dijkstra(aGraph, start):
    print('''Dijkstra's shortest path''')
    # Set the distance for the start node to zero
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(), v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        # for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)

            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
            #     print('updated : current = %s next = %s new_dist = %s' \
            #           % (current.get_id(), next.get_id(), next.get_distance()))
            # else:
            #     print('not updated : current = %s next = %s new_dist = %s' \
            #           % (current.get_id(), next.get_id(), next.get_distance()))

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(), v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)


def get_graph():
    g = Graph()
    g.add_vertex('qr1',  [14., 22.])
    g.add_vertex('qr2',  [21., 18.5])
    g.add_vertex('qr3',  [13.5, 41.5])
    g.add_vertex('qr4',  [31.5, 42.])
    g.add_vertex('qr5',  [53.5, 41.5])
    g.add_vertex('qr6',  [76., 18.])
    g.add_vertex('qr7',  [76.5, 41.5])
    g.add_vertex('qr8',  [76.5, 62.])
    g.add_vertex('qr9',  [80., 81.])
    g.add_vertex('qr10', [80., 104.5])
    g.add_vertex('qr11', [54., 104.5])
    g.add_vertex('qr12', [31.5, 104.5])
    g.add_vertex('qr13', [14.5, 104.5])
    g.add_vertex('qr14', [14., 118.5])
    g.add_vertex('qr15', [21., 127.5])
    g.add_vertex('qr16', [80., 127.5])
    g.add_vertex('qr17', [80., 99.])
    g.add_vertex('qr18', [76.5, 49.5])
    # g.add_vertex('qr19', [80., 99.])
    g.add_vertex('qr20', [66., 18.5])
    g.add_vertex('qr21', [50.5, 18.5])
    g.add_vertex('qr22', [46., 18.5])
    g.add_vertex('qr23', [50.5, 41.5])
    g.add_vertex('qr24', [66., 41.5])
    g.add_vertex('qr25', [61., 104.5])
    g.add_vertex('qr26', [41., 104.5])
    g.add_vertex('qr27', [14.5, 98.])
    g.add_vertex('qr28', [14., 78.])
    g.add_vertex('qr29', [14., 67.5])
    g.add_vertex('qr30', [14., 47.5])
    g.add_vertex('qr31', [31.5, 60.5])
    g.add_vertex('qr32', [31.5, 80.5])
    g.add_vertex('qr33', [54., 80.5])
    g.add_vertex('qr34', [54., 60.5])
    g.add_vertex('qr35', [80., 119.])
    g.add_vertex('qr36', [61., 127.5])
    g.add_vertex('qr37', [41., 127.])
    # g.add_vertex('qr16', [80., 127.5])

    g.add_edge('qr16', 'qr36')
    g.add_edge('qr37', 'qr36')
    g.add_edge('qr37', 'qr15')
    g.add_edge('qr13', 'qr14')
    g.add_edge('qr13', 'qr27')
    g.add_edge('qr28', 'qr27')
    g.add_edge('qr28', 'qr29')
    g.add_edge('qr30', 'qr29')
    g.add_edge('qr30', 'qr3')
    g.add_edge('qr1', 'qr3')
    g.add_edge('qr4', 'qr3')
    g.add_edge('qr4', 'qr31')
    g.add_edge('qr32', 'qr31')
    g.add_edge('qr32', 'qr12')
    g.add_edge('qr13', 'qr12')
    g.add_edge('qr26', 'qr12')
    g.add_edge('qr26', 'qr11')
    g.add_edge('qr25', 'qr11')
    g.add_edge('qr25', 'qr10')
    g.add_edge('qr35', 'qr10')
    g.add_edge('qr35', 'qr16')
    g.add_edge('qr10', 'qr17')
    g.add_edge('qr9', 'qr17')
    g.add_edge('qr8', 'qr18')
    g.add_edge('qr7', 'qr18')
    g.add_edge('qr7', 'qr24')
    g.add_edge('qr7', 'qr24')
    g.add_edge('qr5', 'qr24')
    g.add_edge('qr5', 'qr34')
    g.add_edge('qr33', 'qr34')
    g.add_edge('qr33', 'qr11')
    g.add_edge('qr5', 'qr23')
    g.add_edge('qr4', 'qr23')
    g.add_edge('qr7', 'qr29')
    g.add_edge('qr6', 'qr29')
    g.add_edge('qr6', 'qr21')
    g.add_edge('qr22', 'qr21')
    g.add_edge('qr22', 'qr2')

    return g


if __name__ == '__main__':

    g = Graph()

    g.add_vertex('qr1',  [14., 22.])
    g.add_vertex('qr2',  [21., 18.5])
    g.add_vertex('qr3',  [13.5, 41.5])
    g.add_vertex('qr4',  [31.5, 42.])
    g.add_vertex('qr5',  [53.5, 41.5])
    g.add_vertex('qr6',  [76., 18.])
    g.add_vertex('qr7',  [76.5, 41.5])
    g.add_vertex('qr8',  [76.5, 62.])
    g.add_vertex('qr9',  [80., 81.])
    g.add_vertex('qr10', [80., 104.5])
    g.add_vertex('qr11', [54., 104.5])
    g.add_vertex('qr12', [31.5, 104.5])
    g.add_vertex('qr13', [14.5, 104.5])
    g.add_vertex('qr14', [14., 118.5])
    g.add_vertex('qr15', [21., 127.5])
    g.add_vertex('qr16', [80., 127.5])

    # g.add_edge('qr1', 'qr2')
    g.add_edge('qr1', 'qr3')
    g.add_edge('qr2', 'qr6')
    g.add_edge('qr6', 'qr7')
    g.add_edge('qr5', 'qr7')
    g.add_edge('qr4', 'qr5')
    g.add_edge('qr3', 'qr4')
    g.add_edge('qr3', 'qr4')
    g.add_edge('qr3', 'qr13')
    g.add_edge('qr13', 'qr14')
    # g.add_edge('qr14', 'qr15')
    g.add_edge('qr15', 'qr16')
    g.add_edge('qr10', 'qr16')
    g.add_edge('qr10', 'qr11')
    g.add_edge('qr10', 'qr9')
    g.add_edge('qr8', 'qr9')
    g.add_edge('qr8', 'qr7')
    g.add_edge('qr11', 'qr5')
    g.add_edge('qr12', 'qr4')
    g.add_edge('qr12', 'qr13')

    print('Graph data:')
    for v in g:
        for w in v.get_connections():
            vid = v.get_id()
            wid = w.get_id()
            print('( %s , %s, %3d)' % (vid, wid, v.get_weight(w)))

    dijkstra(g, g.get_vertex('qr1'))

    target = g.get_vertex('qr16')
    path = [target.get_id()]
    shortest(target, path)
    print('The shortest path : %s' % (path[::-1]))

    #class Rack:
    #    direction = np.array([]) # direction bot must face to get pallet
    #    qr1 = '' # qr code from which bot drives forward
    #    qr2 = '' # qr code from which bot drives backward

    #Rack racks[12]
    #racks[0].direction = np.array([1,0])
        # rack_loc = self.G.get_vertex(rack.qr1).get_location()
    # get path length
    # rack_loc = self.G.get_vertex(rack.qr2).get_location()
    # get path length
    # choose shorter path
    # once there allign bot direction to rack.direction and drive forward for rack.qr1 or back for rack.qr2
