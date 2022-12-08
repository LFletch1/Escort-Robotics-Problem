import skgeom as sg
from skgeom.draw import *
from matplotlib import pyplot as plt
from general_polygon import GeneralPolygon
from scikit_utils import *
import time
from collections import deque
from transition import State
# from visibility import VisPoly
from conservative_regions import get_adj_list_of_conservative_centroid_nodes, coords_from_json
from Environment import *

# Documentation and Demos used in this code:
# https://matplotlib.org/stable/gallery/event_handling/coords_demo.html
# https://scikit-geometry.github.io/scikit-geometry/introduction.html
# general_polygon.py

class EscortProblem:
    def __init__(self, env_file_path, root, goal):
        '''
        env_file_path - string to json file containing coordinates of environment
        root - value of starting point in graph, must correspond to a node in the adjacency list. Example: (3,8)
        goal - goal point in the environment, does not have to be a node in adjacency list. Represents, VIP destination. Example: (4,16)
        '''
        self.root = root 
        self.goal = goal

        coords = coords_from_json(env_file_path)
        self.coordinate_adj_list = get_adj_list_of_conservative_centroid_nodes(coords)
        gp = GeneralPolygon.load_from_json(env_file_path, verbose=False)
        gp.build_arrangement(verbose=True)
        np_half = np.array([])
        for ha in gp.arrangement.halfedges:
            np_half = np.append(np_half, sg.Segment2(ha.source().point(), ha.target().point()))
        self.environment = Environment(gp, np_half, self.coordinate_adj_list)

    # BFS algorithm
    def bfs(self): 
        state  = self.environment.get_starting_state(list(self.coordinate_adj_list.keys())[0]) 
        state2 = self.environment.get_starting_state(list(self.coordinate_adj_list.keys())[0]) 
        print(state == state2)

        for key1 in self.coordinate_adj_list.keys():
            for key2 in self.coordinate_adj_list.keys():
                print(key1, "==", key2, end=" ")
                s1 = self.environment.get_starting_state(key1)
                s2 = self.environment.get_starting_state(key2)
                print(s1 == s2)
        # path = []
        # visited, queue = set(), deque([self.root]) 
        # visited.add(self.root)
        # while queue:
        #     # Dequeue a vertex from queue
        #     vertex = queue.popleft()
        #     print(str(vertex) + " ", end="")

        #     # If not visited, mark it as visited, and
        #     # enqueue it
        #     for neighbour in self.adjacency_list[vertex]:
        #         if neighbour not in visited: # Check if safe_zone already appeared at this point
        #             visited.add(neighbour)
        #             queue.append(neighbour)

        # return path 

    def show_path(self, path):
        # path is a list of tuples that represent ordered pairs of coordinates to transition to in the environment
        # [(2,11), (1,9), (2,6), (8,6)]
        state = self.environment.get_starting_state(path[0]) 
        self.environment.draw_state(state)
        plt.show()
        for next_pos in path[1:]:
            plt.clf()
            state = self.environment.transition_blackbox(state, next_pos)
            self.environment.draw_state(state)
            plt.show()
            
        pass


def main():
    escort_prob = EscortProblem("Envs/tetris_env.json", (2,10.5), (1.333, 8.667))
    # Should be able to call path = escort_prob.bfs() and will return a successful path
    escort_prob.bfs()

    # Tetris test path
    path = [(2,10.5), (1.333, 8.667), (2.0, 6.0), (1.333, 8.667), (2.0, 6.0), (8.0, 6.0), (14.0, 6.0)]

    # Rooms test path
    # escort_prob.coordinate_adj_list[(60.0, 66.667)] = []
    # escort_prob.environment.adj_list[(60.0, 66.667)] = []
    # path = [(10.0, 16.667), (6.667, 31.556), (6.667, 33.333), (7.5, 36.75),(16.667, 41.667),(30.0, 43.75),(43.333, 50.0),
    #         (60.0, 50.0),(60.0, 35.0),(60.0, 20.0),(60.0, 35.0),(60.0, 50.0),(63.75, 65.0),(55.0, 66.667),(60.0, 66.667)] 

    escort_prob.show_path(path)


if __name__ == "__main__":
    main()