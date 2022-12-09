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
    def __init__(self, env_file_path, start_pos, vip_goal_pos):
        '''
        env_file_path - string to json file containing coordinates of environment
        root - value of starting point in graph, must correspond to a node in the adjacency list. Example: (3,8)
        goal - goal point in the environment, does not have to be a node in adjacency list. Represents, VIP destination. Example: (4,16)
        '''
        self.start_pos = start_pos
        self.vip_goal_pos = vip_goal_pos

        coords = coords_from_json(env_file_path)
        self.coordinate_adj_list = get_adj_list_of_conservative_centroid_nodes(coords)
        gp = GeneralPolygon.load_from_json(env_file_path, verbose=False)
        gp.build_arrangement(verbose=True)
        np_half = np.array([])
        for ha in gp.arrangement.halfedges:
            np_half = np.append(np_half, sg.Segment2(ha.source().point(), ha.target().point()))
        self.environment = Environment(gp, np_half, self.coordinate_adj_list)

    # BFS algorithm
    def bfs_safe_path(self): 
        print("Searching for Safe Path")
        start_state = self.environment.get_starting_state(self.start_pos)
        visited = {}
        visited[start_state] = True
        queue = deque([start_state]) 
        path = [] 
        end_state = None
        while queue:
            # print(queue)
            # Dequeue a vertex from queue
            current_state = queue.popleft()

            # Check to see if VIP goal position is inside a safezone. If yes, then path has been found
            goal_point = sg.Point2(self.vip_goal_pos[0], self.vip_goal_pos[1])
            if current_state.safezones.locate(goal_point):
                # print("Safe Path Found!")
                # self.environment.draw_state(current_state)
                draw(goal_point, color="yellow")
                end_state = current_state
                break

            # Prune this path if no safezones exists
            elif not current_state.safezones:
                continue

            # enqueue it
            for n_pos in current_state.neighbors_pos:
                neighbor = self.environment.transition_blackbox(current_state, n_pos)
                # if not visited[False]
                if neighbor not in visited: # Check if state has already been visited
                    # visited.add(neighbour)
                    visited[neighbor] = True
                    queue.append(neighbor)
        
        if end_state != None:
            path = []
            previous_state = end_state
            # path.insert(0, end_state.pos)
            while previous_state != None:
                path.insert(0,previous_state.pos)
                previous_state = previous_state.parent
            return path
        else:
            return None




    def show_path(self, path):
        # path is a list of tuples that represent ordered pairs of coordinates to transition to in the environment
        # [(2,11), (1,9), (2,6), (8,6)]
        state = self.environment.get_starting_state(path[0]) 
        self.environment.draw_state(state)
        goal_point = sg.Point2(self.vip_goal_pos[0], self.vip_goal_pos[1])
        draw(goal_point, color="green")
        plt.show()
        for next_pos in path[1:]:
            plt.clf()
            state = self.environment.transition_blackbox(state, next_pos)
            self.environment.draw_state(state)
            draw(goal_point, color="green")
            plt.show()
            
        pass


def main():
    # escort_prob = EscortProblem("Envs/tetris_env.json", (2,10.5), (12.5, 1))
    # escort_prob = EscortProblem("Envs/tetris_env.json", (8.0, 6.0), (12.5, 1)) # No safe path exists
    # escort_prob = EscortProblem("Envs/rooms.json", (10.0, 16.667), (100, 45))
    # escort_prob = EscortProblem("Envs/rooms2.json", (2.333, 8.667), (8.5, 3))
    escort_prob = EscortProblem("Envs/rooms3.json", (3.1, 5.3), (8.4, 5))
    # escort_prob = EscortProblem("Envs/rooms4.json", (8.5, 1.5), (9, 9.5))
    # print(escort_prob.coordinate_adj_list.keys())
    # Should be able to call path = escort_prob.bfs_safe_path() and will return a successful path
    path = escort_prob.bfs_safe_path()
    print("Done")
    # print(path)

    # Tetris test path
    # path = [(2,10.5), (1.333, 8.667), (2.0, 6.0), (1.333, 8.667), (2.0, 6.0), (8.0, 6.0), (14.0, 6.0)]

    # Rooms test path
    # escort_prob.coordinate_adj_list[(60.0, 66.667)] = []
    # escort_prob.environment.adj_list[(60.0, 66.667)] = []
    # path = [(10.0, 16.667), (6.667, 31.556), (6.667, 33.333), (7.5, 36.75),(16.667, 41.667),(30.0, 43.75),(43.333, 50.0),
    #         (60.0, 50.0),(60.0, 35.0),(60.0, 20.0),(60.0, 35.0),(60.0, 50.0),(63.75, 65.0),(55.0, 66.667),(60.0, 66.667)]
    if path != None:
        escort_prob.show_path(path)
    else:
        print("No safe path exists")


if __name__ == "__main__":
    main()