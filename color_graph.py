# Goal to visually view the safe zones within the enviornment
from Environment import *
from EscortProblem import *


def count_safe_zones(envstate):
    safezones = envstate.safezones
    num = 0
    for poly in safezones.polygons:
        num += 1
    return num


escort_prob = EscortProblem("Envs/rooms.json", (10.0, 16.667), (100, 45))
state = escort_prob.environment.get_starting_state(escort_prob.start_pos)
number = count_safe_zones(state)
print(number)
escort_prob.environment.draw_state(state)
plt.show()
