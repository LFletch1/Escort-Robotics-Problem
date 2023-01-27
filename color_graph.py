# Goal to visually view the safe zones within the enviornment
from Environment import *
from EscortProblem import *

def get_grid_point(start_pos, interval, gp):
    # Plan:
    # get all neighbors of single point
    # check if they are within the enviornment, if not remove
    # for each remaining neighbor, repeat the same process
    # Return when the entire list traversed
    neighbors = [
                (start_pos[0]- interval, start_pos[1]),
                (start_pos[0]+ interval, start_pos[1]),
                (start_pos[0], start_pos[1]-interval),
                (start_pos[0], start_pos[1]+interval)
                ]

    for n in neighbors:
        if not gp.contains(n):
            neighbors.remove(n)

    return neighbors

def count_safe_zones(envstate):
    safezones = envstate.safezones
    num = 0
    for poly in safezones.polygons:
        num += 1
    return num


escort_prob = EscortProblem("Envs/rooms.json", (10.0, 16.667), (100, 45))

state = escort_prob.environment.get_starting_state(escort_prob.start_pos)
number = count_safe_zones(state)
escort_prob.environment.draw_state(state)
plt.show()
