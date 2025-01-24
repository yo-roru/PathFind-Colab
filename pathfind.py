
# -*- coding: utf-8 -*-

from collections import UserList
from itertools import chain

class Edge(object):
    """An edge is a unidirectional link between two nodes in a graph."""
    def __init__(self, source, destination, ref=None, weight=1):
        self.source = source
        self.destination = destination
        self.ref = ref
        self.weight = weight
    def __str__(self):
        if self.weight == 1:
            return "%s(%s, %s, %r)" % (self.__class__.__name__, \
                str(self.source), str(self.destination), self.ref)
        else:
            return "%s(%s, %s, %r, %r)" % (self.__class__.__name__, \
                str(self.source), str(self.destination), self.ref, self.weight)
    __repr__ = __str__


class Node(object):
    """A node (or vertex) in a graph."""
    def __init__(self, ref=None, weight=0, edges=[]):
        self.ref = ref
        self.weight = weight
        self.edges = edges[:]
    def add_edge(self, neighbour, ref=None, weight=1):
        edge = Edge(self, neighbour, ref, weight)
        self.edges.append(edge)
        return edge
    def add_bidirectional_edge(self, neighbour, ref=None, weight=1):
        edge1 = self.add_edge(neighbour, ref, weight)
        edge2 = neighbour.add_edge(self, ref, weight)
        return (edge1,edge2)
    def neigbours(self):
        return [edge.destination for edge in self.edges]
    #我改的 
    def update_edge_weight(self, neighbour, new_weight):
        
        for edge in self.edges:
            if edge.destination == neighbour:
                edge.weight = new_weight
        for edge in neighbour.edges:
            if edge.destination == self:
                edge.weight = new_weight
        

    def __str__(self):
        if self.weight:
            return "%s(%r, %r)" % (self.__class__.__name__, self.ref, self.weight)
        else:
            return "%s(%r)" % (self.__class__.__name__, self.ref)
    __repr__ = __str__

class Graph(UserList):
    """A directed graph is a list of nodes, which have associated edges."""
    def __init__(self):
        UserList.__init__(self)
        self._noderefs = {}
    def add_node(self, ref=None, weight=0):
        n = Node(ref=ref, weight=weight)
        if ref != None:
            self._noderefs[ref] = n
        self.append(n)
        return n
    def get_node(self, ref):
        return self._noderefs[ref]
    # __getitem__ = get_node
    def nodes(self):
        return self
    def edges(self):
        edges = []
        for node in self:
            edges.extend(node.edges)
        return edges
    def copy(self):
        """Return a copy of the graph, with other node and edge objects."""
        g = Graph()
        nodes = {node: g.add_node(node.ref, node.weight) for node in self}
        for oldnode, newnode in nodes.items():
            for oldedge in oldnode.edges:
                newneighbour = nodes[oldedge.destination]
                newnode.add_edge(newneighbour, oldedge.ref, oldedge.weight)
        return g

class NoPath(Exception):
    """Raised if not path exists between a given source and destination"""
    def __init__(self, source, destination, k=None):
        self.source = source
        self.destination = destination
        self.k = k
    def __str__(self):
        if self.k:
            return "No %d paths exist between %s and %s." %(self.k, self.source, self.destination)
        else:
            return "No path exists between %s and %s." %(self.source, self.destination)

class _InverseEdge(Edge):
    """An inverse edge annuls a forward edge from s to d, and behaves as a 
    regular edge from d to s with negative weight. It is used for Bhandari's
    and Suurballe's algorithms."""
    def __init__(self, edge):
        self.edge = edge
        self.source = edge.destination
        self.destination = edge.source
        self.ref = edge.ref
        self.weight = -edge.weight
    def __str__(self):
        return "%s(%s, %s, %r, %r)" % (self.__class__.__name__, \
            str(self.source), str(self.destination), self.ref, self.weight)

class _DoubleEdge(Edge):
    """A double edge behaves as tow consecutive edges.
    It is used for Suurballe's algorithm."""
    def __init__(self, edge1, edge2):
        self.edge1 = edge1
        self.edge2 = edge2
        assert edge1.destination == edge2.source
        self.source = edge1.source
        self.destination = edge2.destination
        self.ref = edge1.ref
        self.weight = edge1.weight + edge2.weight
    def __str__(self):
        return "%s(%s, %s, %r, %r)" % (self.__class__.__name__, \
            str(self.source), str(self.destination), self.ref, self.weight)

class PathFinder(object):
    """Implmentation of the A* shortest path finding algorithm. 
    This is a variant of Edsgar W. Dijkstra's shortest path algorithm, with 
    a heuristic based on the mimimum distance between two points."""
    debug = 0  # 0...5
    LIFO = -1  # constant: last in first out
    FIFO = 0   # constant: first in first out
    queue_type = LIFO
    stop_first_answer = True # Set to False for negative weights
    sort_queue = True # Set to False if the tree can be looped in any order
    def __init__(self, graph, start=None, destination=None, \
                 heurist_lower_boundary=None, debug=0):
        self.graph = graph      # list of all nodes
        self.path = None        # list of edges in the shortest path
        self.hops = None        # list of node in the shortest path
        self.cost = None        # cost of the shortest path
        self.iterations = 0
        if heurist_lower_boundary:
            self.heurist_lower_boundary = heurist_lower_boundary
        self.debug = debug
        if start and destination:
            self.solve(start, destination)
    def heurist_lower_boundary(self, node1, node2):
        """heurist_lower_boundary returns a lower boundary of the distance
        between node1 and node2. In case this function always returns 0, the 
        A* algorithm is the same as the Dijkstra algorithm.
        """
        return 0
    def solve(self, start, destination):
        """Calculate a shortest path"""
        self.create_tree(start, destination)
        if destination not in self._prev:
            raise NoPath(start, destination)
        self.trace_path(destination)
    def initialise_path_find(self, start, destination=None):
        self._prev = {start: None}
        """self._prev[node]: previous node for minimum cost path"""
        self._mincost = {start: start.weight}
        """self._mincost[node]: minumum cost from start to 'node'"""
        self._minpathcost = {start: start.weight + \
                self.heurist_lower_boundary(start, destination)}
    def _get_edges(self, node):
        return node.edges
    def _add_to_queue(self, queue, node, cost, trace, destination):
        """Add node to the queue with given <cost> from start to the node.
        trace is the edge that has been followed"""
        self._mincost[node] = cost
        self._minpathcost[node] = cost + \
                 self.heurist_lower_boundary(node, destination)
        self._prev[node] = trace
        if self.debug > 2:
            print("  append %s with cost: %d" % (node, cost))
        # Do not reinsert if it is already in the queue!
        if node not in queue:
            queue.append(node)
    def _better_path(self, node, cost):
        try:
            curcost = self._mincost[node]
        except KeyError:
            return True
        if self.debug > 4 and cost >= curcost:
            print("  skip %s (cost %d >= %d)" % (node, cost, curcost))
        return cost < curcost
    def create_tree(self, start, destination=None):
        """Calculate a shortest path"""
        if self.debug > 0:
            if destination:
                print("Find a path", start, "-->", destination)
            else:
                print("Find a tree from", start)
        self.initialise_path_find(start, destination)
        queue = [start]
        while queue:
            self.iterations += 1
            if self.sort_queue:
                queue.sort(key = lambda n: self._minpathcost[n], reverse=True)
            if self.debug > 3:
                print("queue:",queue)
            try:
                node = queue.pop(self.queue_type)
            except IndexError: # queue is empty
                break
            curcost = self._mincost[node]
            if self.debug > 1:
                print("iteration %d: pop node %s cost: %s" % \
                        (self.iterations, node, curcost))
            # Original Dijkstra algorithm stops at the first answer.
            # Continue to support negative weights
            if self.stop_first_answer and node == destination:
                break
            # assert curcost > -5
            
            for edge in self._get_edges(node):
                neighbour = edge.destination
                totalcost = curcost + edge.weight + neighbour.weight
                if not self._better_path(neighbour, totalcost):
                    continue
                self._add_to_queue(queue, neighbour, totalcost, edge, destination)
    def trace_path(self, destination):
        # path is a list of edges
        # hops is a list of nodes
        try:
            self.cost = self._mincost[destination]
            edge = self._prev[destination]
        except KeyError:
            raise NoPath(None, destination)
        self.path = []
        self.hops = [destination]
        while edge != None:
            self.path = [edge] + self.path
            self.hops = [edge.source] + self.hops
            edge = self._prev[edge.source]

class Dijkstra(PathFinder):
    stop_first_answer = True
    sort_queue = True

class Bhandari(PathFinder):
    """Implmentation of the Bhandari edge disjoint path finding algorithm. 
    This algorithm finds <k> different, edge disjoint path."""
    stop_first_answer = False
    def __init__(self, graph, start=None, destination=None, k=2, \
                 heurist_lower_boundary=None, debug=0):
        self.graph = graph      # list of nodes
        self.path = []          # list of all found shortest paths
        self.hops = []          # list of hops in each shortest path
        self.cost = []          # list of the cost of each shortest path
        self.iterations = 0
        if heurist_lower_boundary:
            self.heurist_lower_boundary = heurist_lower_boundary
        self.debug = debug
        if start and destination:
            self.solve(start, destination, k)
    def solve(self, start, destination, k=2):
        """Calculate a shortest path"""
        _orig_stop_first_answer = self.stop_first_answer
        for c in range(k):
            self.create_tree(start, destination)
            #print(self.graph.__dict__)
            if destination not in self._prev:    
                self.stop_first_answer = _orig_stop_first_answer # reset to original value
                break
                #raise NoPath(start, destination, c+1)
            self.trace_path(destination)
            self.untangle_paths()
            self.stop_first_answer = False # required for negative weights
        self.stop_first_answer = _orig_stop_first_answer # reset to original value
    def initialise_path_find(self, start, destination=None):
        self._prev = {start: None}
        """self._prev[node]: previous node for minimum cost path"""
        self._mincost = {start: start.weight}
        """self._mincost[node]: minumum cost from start to 'node'"""
        self._minpathcost = {start: start.weight + \
                self.heurist_lower_boundary(start, destination)}
        """self._minpathcost[node]: lower boundary of cost from start via 
        'node' to destination"""
        self._forward_edges = set(chain(*self.path))
        """Edges used in previous found shortest paths"""
        self._inverse_edges = {}
        """Inverse of the edges in _forward_edges"""
        for k, path in enumerate(self.path):
            for i, edge in enumerate(path):
                hop = edge.destination
                if hop == destination:
                    # Suurballe prevents hops from occuring in multiple paths.
                    # However, the destination (and source) are exempt from
                    # this requirement (they obviously occur in all paths.)
                    # So don't include the destination in _inverse_edges.
                    continue
                assert hop not in self._inverse_edges
                self._inverse_edges[hop] = _InverseEdge(edge)
    def _get_edges(self, node):
        # Both:
        # remove edges in self.path
        # add inverse edges for edges in self.path
        # TODO: shouldn't the cost of the node also be taken into account?
        for edge in node.edges:
            if edge not in self._forward_edges:
                yield edge
            elif self.debug > 4:
                print("  skip edge already in use", edge)
        if self.debug > 4 and node in self._inverse_edges:
            print("  add inverse edge", self._inverse_edges[node])
        try:
            yield self._inverse_edges[node]
        except KeyError:
            pass
    def _path_edge_index(self, edge):
        """Given an edge, return the indexes (k, i) for the path and location 
        in that path where this edge is located. 
        Raises ValueError if the edge can't be found."""
        for k, path in enumerate(self.path):
            try:
                i = path.index(edge)
            except ValueError:
                continue
            else:
                return k,i
        raise ValueError("edge %s not part of any path" % edge)
    def _update_hops(self, k):
        self.hops[k] = [self.path[k][0].source] + \
                       [e.destination for e in self.path[k]]
        self.cost[k] = sum(e.weight for e in self.path[k]) + \
                       sum(n.weight for n in self.hops[k])
    def trace_path(self, destination):
        """Trace a path from the destination back to start."""
        # path is a list of edges
        # hops is a list of nodes
        try:
            edge = self._prev[destination]
        except KeyError:
            raise NoPath(None, destination)
        path = []
        while edge != None:
            assert edge not in path
            if isinstance(edge, _DoubleEdge):
                path = [edge.edge1, edge.edge2] + path
            else:
                path = [edge] + path
            edge = self._prev[edge.source]
        self.path.append(path)
        self.hops.append(None)
        self.cost.append(None)
        if self.debug > 3:
            self._update_hops(-1)
            print("Found (raw) path:", self.hops[-1])
    def untangle_paths(self):
        """Untangle path by eliminating any occurance of _InverseEdge()
        in the last found path. This may change multiple paths."""
        k = len(self.path)-1 # path to check for InverseEdges..
        dirty_paths = set([k])
        path = self.path[k]
        for i in range(len(path)-1, -1, -1):
            edge = path[i]
            if isinstance(edge, _InverseEdge):
                # Don't store in which path the forward edge is located
                # in the _InverseEdge(), since it changes with each untangle.
                l,j = self._path_edge_index(edge.edge)
                if self.debug > 3:
                    print("Untangle paths %d and %d at %s" % \
                            (l, k, edge.edge))
                dirty_paths.add(l)
                # somehow the trick a,b = b,a did not work.
                # Note: this also works if k == l.
                _path = self.path[k][:i] + self.path[l][j+1:]
                self.path[l] = self.path[l][:j] + self.path[k][i+1:]
                self.path[k] = _path
        for l in dirty_paths:
            self._update_hops(l)
        if self.debug > 0:
            for l in sorted(dirty_paths - {k}):
                print("Reroute path %d: %s" % (l, self.hops[l]))
            print("Found path %d: %s" % (k, self.hops[k]))

class AdjustWeights():
    pass


if __name__ == '__main__':
    #import unittest
    def two_paths():
        graph = Graph()
        # create nodes
        s1 = graph.add_node('S1')
        s2 = graph.add_node('S2')
        s3 = graph.add_node('S3')
        s4 = graph.add_node('S4')
        s5 = graph.add_node('S5')
        s6 = graph.add_node('S6')
        s7 = graph.add_node('S7')
        #h = graph.add_node('H')
        # create edges
        s1.add_bidirectional_edge(s2, 'S1S2', weight=1)
        s1.add_bidirectional_edge(s5, 'S1S5', weight=1)
        #s1.add_bidirectional_edge(s3, 'S1S3', weight=1)

        s2.add_bidirectional_edge(s5, 'S2S5', weight=1)
        s3.add_bidirectional_edge(s4, 'S3S4', weight=1)
        s4.add_bidirectional_edge(s5, 'S4S5', weight=1)

        s1.add_bidirectional_edge(s6, 'S1S6', weight=1)
        s2.add_bidirectional_edge(s6, 'S2S6', weight=1)

        s7.add_bidirectional_edge(s6, 'S6S7', weight=1)
        s7.add_bidirectional_edge(s3, 'S3S7', weight=1)
        s7.add_bidirectional_edge(s5, 'S5S7', weight=1)
        
        '''

        a = graph.add_node('A')
        b = graph.add_node('B')
        c = graph.add_node('C')
        d = graph.add_node('D')
        e = graph.add_node('E')
        f = graph.add_node('F')
        a.add_bidirectional_edge(b, 'AB', weight=1)
        a.add_bidirectional_edge(c, 'AC', weight=1)
        b.add_bidirectional_edge(e, 'BE', weight=1)
        c.add_bidirectional_edge(d, 'CD', weight=1)
        d.add_bidirectional_edge(b, 'DB', weight=1)
        d.add_bidirectional_edge(f, 'DF', weight=1)
        e.add_bidirectional_edge(f, 'EF', weight=1)      
        
        # 添加節點
        n1 = graph.add_node('N1')
        n2 = graph.add_node('N2')
        n3 = graph.add_node('N3')
        n4 = graph.add_node('N4')
        n5 = graph.add_node('N5')
        n6 = graph.add_node('N6')
        n7 = graph.add_node('N7')
        n8 = graph.add_node('N8')
        n9 = graph.add_node('N9')
        n10 = graph.add_node('N10')

        # 添加雙向邊 (權重為 1~10)
        n1.add_bidirectional_edge(n2, 'N1N2', weight=1)
        #n1.add_bidirectional_edge(n3, 'N1N3', weight=1)
        n2.add_bidirectional_edge(n3, 'N2N3', weight=1)
        n2.add_bidirectional_edge(n5, 'N2N5', weight=1)
        n3.add_bidirectional_edge(n4, 'N3N4', weight=1)
        n3.add_bidirectional_edge(n6, 'N3N6', weight=1)
        n4.add_bidirectional_edge(n5, 'N4N5', weight=1)
        n4.add_bidirectional_edge(n7, 'N4N7', weight=1)
        n5.add_bidirectional_edge(n8, 'N5N8', weight=1)
        n6.add_bidirectional_edge(n8, 'N6N8', weight=1)
        n6.add_bidirectional_edge(n9, 'N6N9', weight=1)
        n7.add_bidirectional_edge(n9, 'N7N9', weight=1)
        n7.add_bidirectional_edge(n10, 'N7N10', weight=1)
        n8.add_bidirectional_edge(n10, 'N8N10', weight=1)
        n9.add_bidirectional_edge(n10, 'N9N10', weight=1)
        n1.add_bidirectional_edge(n4, 'N1N4', weight=1)
        n2.add_bidirectional_edge(n6, 'N2N6', weight=1)
        n3.add_bidirectional_edge(n7, 'N3N7', weight=1)
        n5.add_bidirectional_edge(n9, 'N5N9', weight=1)
        n6.add_bidirectional_edge(n10, 'N6N10', weight=1)
        '''
        #print(graph)
        return graph, s3, s1
    
    import pprint

    g, s, d = two_paths() 
    num_paths = 3 
    disjoint_path = Bhandari(g, s, d, num_paths)
    #pprint.pprint(disjoint_path.__dict__['path'])
    paths=disjoint_path.__dict__['path']
    
    adjust_weight=4
    #print(paths)
    print("Find the disjoint paths:")
    for path in paths:
        print(end='   ')
        for edge in path:
            print(edge.ref,'', end='')
        print()
    num_disjoint_paths = len(paths)

    for idx in range(len(paths),num_paths):
        for path in paths:
            for edge in path:
                #print(edge.source.__dict__,edge.destination)
                edge.source.update_edge_weight(edge.destination,edge.weight*adjust_weight)
        new_path = Dijkstra(g, s, d)
        paths.append(new_path.__dict__['path'])
    #print(paths)

    print(f"Others: (adjust weight: {adjust_weight})")
    for path in paths[num_disjoint_paths:]:
        print(end='   ')
        for edge in path:
            print(edge.ref,'', end='')
        print()
    
    #s.update_edge_weight(g.get_node('E'),3)
    #pprint.pprint(s.edges)
    #pprint.pprint(g.get_node('E').edges)
    
    #pprint.pprint(b.__dict__)

