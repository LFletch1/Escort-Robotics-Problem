from EscortProblem import EscortProblem
import matplotlib.pyplot as plt
import skgeom as sg
from skgeom.draw import draw
import numpy as np


def main():
    
    # ep = EscortProblem("Envs/tetris_env.json", (2,10.5), (12.5, 1))
    ep = EscortProblem("Envs/rooms.json", (10.0, 16.667), (100, 45))
    # ep = EscortProblem("Envs/rooms2.json", (2.333, 8.667), (8.5, 3))
    # ep = EscortProblem("Envs/rooms3.json", (3.1, 5.3), (8.4, 5))
    # ep = EscortProblem("Envs/new_env.json", (1, 1.5), (3.4, 7.4))
    # ep = EscortProblem("Envs/new_env2.json", (1.5, 8.5), (7.5, 2.5))
    # ep = EscortProblem("Envs/new_env3.json", (9.667, 2.889), (2.5, 7.5))
    xs = np.arange(0,12,0.1)
    ys = np.arange(0,10,0.1)
    draw(ep.environment.gp)
    colors = {0:"red", 1:"purple", 2:"blue", 3:"seagreen", 4:"green", 5:"darkgreen"}
    for x_coord in xs:
        x_coord -= 0.01
        for y_coord in ys:
            y_coord -= 0.01
            coord = sg.Point2(float(x_coord),float(y_coord))
            try:
                if ep.environment.gp.locate(coord):
                    print(f"Good coord ({x_coord},{y_coord})")
                    state = ep.environment.get_starting_state((float(x_coord),float(y_coord)), has_adj_list=False)
                    num_of_safe_zones = len(state.safezones.polygons)    
                    draw(coord, color=colors[num_of_safe_zones])
                else:
                    print(f"Bad coord ({x_coord},{y_coord})")
            except:
                print("BUG")
    plt.show()
    

if __name__ == "__main__":
    main()