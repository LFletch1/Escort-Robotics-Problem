import skgeom as sg
from skgeom.draw import *
from matplotlib import pyplot as plt
from general_polygon import GeneralPolygon
from scikit_utils import *
from collections import deque
# from visibility import VisPoly
from conservative_regions import get_adj_list_of_conservative_centroid_nodes, coords_from_json
from Environment import *
import time

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


    def bfs_safe_path(self): 
        '''Breadth first search of graph to find path which makes a VIP safezones contain the goal point'''
        print("Searching for Safe Path")
        start_state = self.environment.get_starting_state(self.start_pos)
        visited = {}
        visited[start_state] = True
        queue = deque([start_state]) 
        end_state = None
        while queue:
            # Dequeue a vertex from queue
            current_state = queue.popleft()

            # Check to see if VIP goal position is inside a safezone. If yes, then path has been found
            goal_point = sg.Point2(self.vip_goal_pos[0], self.vip_goal_pos[1])
            if current_state.safezones.locate(goal_point):
                draw(goal_point, color="green")
                end_state = current_state
                break

            # Prune this path if no safezones exists
            elif not current_state.safezones:
                continue

            # Enqueue it
            for n_pos in current_state.neighbors_pos:
                neighbor = self.environment.transition_blackbox(current_state, n_pos)
                if neighbor not in visited: # Check if state has already been visited
                    visited[neighbor] = True
                    queue.append(neighbor)
        
        if end_state != None:
            path = []
            previous_state = end_state
            while previous_state != None:
                path.insert(0,previous_state.pos)
                previous_state = previous_state.parent
            return path
        else:
            return None


    def show_path(self, path):
        # Path is a list of tuples that represent pairs of coordinates to transition to in the environment
        # Example: [(2,11), (1,9), (2,6), (8,6)]
        state = self.environment.get_starting_state(path[0]) 
        self.environment.draw_state(state)

        goal_point = sg.Point2(self.vip_goal_pos[0], self.vip_goal_pos[1])
        draw(goal_point, color="green")
        # plt.show()
        for next_pos in path[1:]:
            plt.clf()
            state = self.environment.transition_blackbox(state, next_pos)
            # self.environment.draw_state(state)
            draw(goal_point, color="green")
            # plt.show()
        return state
    


def main():
    # escort_prob = EscortProblem("Envs/tetris_env.json", (2,10.5), (12.5, 1))
    # escort_prob = EscortProblem("Envs/tetris_env.json", (8.0, 6.0), (12.5, 1)) # No safe path exists
    # escort_prob = EscortProblem("Envs/rooms.json", (1, 1.667), (10, 4.5))
    # escort_prob = EscortProblem("Envs/rooms.json", (1, 1.667), (1.5, 3))
    escort_prob = EscortProblem("Envs/rooms2.json", (2.5, 8.5), (8.5, 3))
    # escort_prob = EscortProblem("Envs/rooms3.json", (3.1, 5.3), (8.4, 5))
    # escort_prob = EscortProblem("Envs/new_env.json", (1, 1.5), (3.4, 7.4))
    # escort_prob = EscortProblem("Envs/new_env2.json", (1.5, 8.5), (7.5, 2.5))
    # escort_prob = EscortProblem("Envs/new_env3.json", (9.667, 2.889), (2.5, 7.5))
    t1 = time.time()
    path = escort_prob.bfs_safe_path()
    t2 = time.time()
    print(t2-t1, "seconds")
    if path != None:
        print("Path Found!")
        state = escort_prob.show_path(path)
    else:
        print("No safe path found :(")
    escort_prob.environment.draw_state(state)

    i = 1
    while i < len(path):
        prev_point = sg.Point2(path[i-1][0],path[i-1][1])
        curr_point = sg.Point2(path[i][0],path[i][1])
        draw(sg.Segment2(prev_point, curr_point), color="black")
        i += 1
    draw(sg.Point2(path[0][0], path[0][1]), color="blue")
    draw(sg.Point2(path[-1][0], path[-1][1]), color="red")
    draw(sg.Point2(8.5, 3), color="green")
    plt.savefig("rooms2_solution.png")


if __name__ == "__main__":
    main()