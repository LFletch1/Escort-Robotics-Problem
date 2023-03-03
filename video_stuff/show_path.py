from rrt_star import RRTStar
import skgeom as sg
from skgeom.draw import *
import sys
import math

sys.path.append("../")

from EscortProblem import EscortProblem


def interpolate_points_with_fixed_distance(point1, point2, d):
    # Calculate the distance between the two points
    distance = ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5
    
    # Calculate the number of intervals required based on the fixed distance
    num_intervals = math.floor(distance / d)
    
    if num_intervals == 0:
        return [point1, point2]
    
    # Calculate the interval length based on the number of intervals
    dx = (point2[0] - point1[0]) / num_intervals
    dy = (point2[1] - point1[1]) / num_intervals
    
    # Generate the list of interpolated points with the fixed distance
    points = [(point1[0] + i * dx, point1[1] + i * dy) for i in range(1, num_intervals)]
    
    points[0] = point1
    points[-1] = point2
    # Return the list of interpolated points
    return points



def interpolate_path(path, speed):
    print(path)
    interpolated_path = [path[0]] # Keep starting point, then don't include starting point interpolations
    for i in range(1,len(path)):
        p1 = path[i-1]
        p2 = path[i]
        points = interpolate_points_with_fixed_distance(p1, p2, speed)
        interpolated_path += points[1:]
        print(f"From point 1 {p1} to point 2 {p2}: {points}")
        for point in points:
            p = sg.Point2(point[0], point[1])
            draw(p)
    plt.show()



def main():
    env_files = ["../Envs/tetris_env.json",
                "../Envs/rooms.json",
                "../Envs/rooms2.json",
                "../Envs/rooms3.json",
                "../Envs/new_env.json",
                "../Envs/interesting.json",
                "../Envs/new_env3.json",
                "../Envs/pursuit_fail.json"]
    env_files = ["../Envs/tetris_env.json"]
    for env_file in env_files:
        path_file = env_file.replace("Envs","solution_paths").replace(".json", "_path.txt")
        file = open(path_file, 'r')
        file.readline()
        escort_start_x, escort_start_y = file.readline().strip().replace(" ","").replace("(","").replace(")","").split(":")[1].split(",")
        escort_start_pos = (float(escort_start_x), float(escort_start_y))
        file.readline()
        vip_goal_x, vip_goal_y = file.readline().strip().replace(" ","").replace("(","").replace(")","").split(":")[1].split(",")
        VIP_goal_pos = (float(vip_goal_x), float(vip_goal_y))

        lines = file.readlines()
        path = []
        for line in lines:
            x, y = line.strip().split(",")
            path.append((float(x),float(y)))
        
        prob = EscortProblem(env_file, escort_start_pos, VIP_goal_pos)
        draw(prob.environment.gp)
        interpolate_path(path, 0.1)

        prob = EscortProblem(env_file, escort_start_pos, VIP_goal_pos)
    #     # prob.show_path(path)
    #     prob.show_path(path, save_fig=True, filename=env_file.replace("Envs","solution_svgs").replace(".json", "_solution.svg"))




if __name__ == "__main__":
    main()