# Goal to visually view the safe zones within the enviornment
from Environment import *
from EscortProblem import *




escort_prob = EscortProblem("Envs/rooms.json", (10.0, 16.667), (100, 45))
print("HERE")

state = escort_prob.environment.get_starting_state(escort_prob.start_pos)
safezones = state.safezones
num = 0
for poly in safezones.polygons:
    num += 1

escort_prob.environment.draw_state(state)
plt.show()
print("Here")
print(num)
