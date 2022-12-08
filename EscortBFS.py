from skgeom import *
from skgeom.draw import *
from matplotlib import pyplot as plt
from general_polygon import GeneralPolygon
from scikit_utils import *
import time
from collections import deque
from transition import State
from visibility import VisPoly
from conservative_regions import get_adj_list_of_conservative_centroid_nodes, coords_from_json

# Documentation and Demos used in this code:
# https://matplotlib.org/stable/gallery/event_handling/coords_demo.html
# https://scikit-geometry.github.io/scikit-geometry/introduction.html
# general_polygon.py

class EscortProblem:
    def __init__(self, env_file_path, root, goal):
        '''
        env_file_path - string to json file containing coordinates of environment
        root - value of starting point in graph, must correspond to a node in the adjacency list. Example: (3,8)
        goal - goal point in the environment, does not have to be a node in adjacency list. Example: (4,16)
        '''
        self.root = root 
        self.goal = goal
        coords = coords_from_json(env_file_path)
        self.coordinate_adj_list = get_adj_list_of_conservative_centroid_nodes(coords)
        gp = GeneralPolygon.load_from_json(env_file_path, verbose=False)
        gp = GeneralPolygon.load_from_json("Envs/rooms.json", verbose=True)

        gp.build_arrangement(verbose=True)
        np_half = np.array([])
        for ha in gp.arrangement.halfedges:
            # draw(ha.curve())
            np_half = np.append(np_half, Segment2(ha.source().point(), ha.target().point()))
        environment = State(gp, np_half, 1)


    # BFS algorithm
    def bfs(self):
        
        path = []
        visited, queue = set(), deque([self.root])
        
        visited.add(self.root)

        while queue:

            # Dequeue a vertex from queue
            vertex = queue.popleft()
            print(str(vertex) + " ", end="")

            # If not visited, mark it as visited, and
            # enqueue it
            for neighbour in self.adjacency_list[vertex]:
                if neighbour not in visited: # Check if safe_zone already appeared at this point
                    visited.add(neighbour)
                    queue.append(neighbour)


