import skgeom as sg
from skgeom.draw import *
import sys
import math
import cv2
import os
from extremitypathfinder import PolygonEnvironment


sys.path.append("../")

from EscortProblem import EscortProblem


def interpolate_points_with_fixed_distance(point1, point2, d):
    # Calculate the distance between the two points
    distance = ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5
    
    # Calculate the number of intervals required based on the fixed distance
    num_intervals = math.floor(distance / d)
    
    if num_intervals <= 1:
        return [point1, point2]
    
    # Calculate the interval length based on the number of intervals
    dx = (point2[0] - point1[0]) / num_intervals
    dy = (point2[1] - point1[1]) / num_intervals
    
    # Generate the list of interpolated points with the fixed distance
    points = [(point1[0] + i * dx, point1[1] + i * dy) for i in range(1, num_intervals+1)]
    
    points[0] = point1
    points[-1] = point2
    # Return the list of interpolated points
    return points



def interpolate_path(path, speed):
    interpolated_path = [path[0]] # Keep starting point, then don't include starting point interpolations
    for i in range(1,len(path)):
        p1 = path[i-1]
        p2 = path[i]
        points = interpolate_points_with_fixed_distance(p1, p2, speed)
        interpolated_path += points[1:]
    return interpolated_path



def images_to_video(input_path, output_path, fps):
    # Get a list of all the image file names in the input directory
    file_names = sorted([f for f in os.listdir(input_path) if f.endswith('.png')])

    # Get the dimensions of the first image
    img = cv2.imread(os.path.join(input_path, file_names[0]))
    height, width, _ = img.shape

    # Initialize the video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    # Iterate over the images, adding them to the video
    for file_name in file_names:
        img_path = os.path.join(input_path, file_name)
        img = cv2.imread(img_path)
        out.write(img)

    # Release the video writer and destroy any open windows
    out.release()
    cv2.destroyAllWindows()

def make_escort_problem_video(env_file, video_name):

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

    speed = 0.07

    pic_file_dir = video_name.replace(".mp4", "/")
    if not os.path.exists(pic_file_dir):
        os.mkdir(pic_file_dir)
    prob = EscortProblem(env_file, escort_start_pos, VIP_goal_pos)

    curr_state, vip_start = prob.environment.get_starting_state(path[0], showing=True)
    prev_state = None
    vip_end_point = sg.Point2(VIP_goal_pos[0], VIP_goal_pos[1])
    curr_vip_pos = vip_start
    # vip_escort_positions = [] # List of pairs of positions, one representing the escort position that corresponds to when the VIP needs to move
    path_states = [curr_state]
    points = []
    for pos in path:
        curr_state = prob.environment.transition_blackbox(curr_state, pos, showing=True)
        path_states.append(curr_state)

    # Need to calculate VIP-reachable-solvable safezones. These represent safe-zones that are reachable and that are apart of the safe-zones that 
    # can be traced to the solution
    # Need to do this by backtracking from the solution state to each state
    prev_state = path_states[-1]
    solvable_safe_zones = [[poly.outer_boundary() for poly in prev_state.safezones.polygons]]
    print(solvable_safe_zones)
    for curr_state in path_states[-2::-1]:
        for poly in curr_state.safezones.polygons
        prev_state = curr_state
        


    #     for poly in curr_state.safezones.polygons:
    #         points.append(sg.centroid(poly.outer_boundary()))
    #     # print(sg.centroid(curr_state.safezones.polygons[0].outer_boundary()))
    #     curr_state = prob.environment.transition_blackbox(curr_state, pos, showing=True)
    #     # path_states.append(curr_state)
    # draw(prob.environment.gp)
    # for point in points:
    #     draw(point)
    # plt.show()
    # Go through the states in reverse order, if a state is found where the safezone is disconnected from the VIP position,
    # then we know we need to move the VIP to that position as soon as it becomes available
    # reversed_path_states = path_states[::-1]
    # for state in reversed_path_states:
    #     if state.safezones.polygon[]

    # print(vip_escort_positions)
    # print(path_states)
    exit()

    interpolated_path = interpolate_path(path, speed)
    curr_state, vip_start = prob.environment.get_starting_state(interpolated_path[0], showing=True)
    vip_end_point = sg.Point2(VIP_goal_pos[0], VIP_goal_pos[1])
    draw(vip_start, color="#e97419")
    prob.environment.draw_state(curr_state)
    plt.savefig(pic_file_dir + "00000", bbox_inches='tight')
    f = 0
    for pos in interpolated_path[1:]:
        curr_state = prob.environment.transition_blackbox(curr_state, pos,showing=True)
        prob.environment.draw_state(curr_state)
        draw(vip_start, color="#e97419")
        draw(vip_end_point, color="#a600ff")
        plt.axis("off")
        plt.savefig(pic_file_dir + str(f).zfill(5), bbox_inches='tight')
        plt.clf()
        f += 1 

    sz_environment = PolygonEnvironment()
    safe_zone = curr_state.safezones.polygons[0].outer_boundary()
    poly_vertices = []
    list_of_holes = [[(100,100),(100,101),(101,101),(101,100)]] # Library I'm using doesn't work if no holes are included in the environment
    for v in safe_zone.vertices:
        poly_vertices.append((v.x(), v.y()))
    sz_environment.store(poly_vertices, list_of_holes, validate=False)
    sz_environment.prepare()
    start_coords = (vip_start.x(), vip_start.y())
    vip_path, vip_path_length = sz_environment.find_shortest_path(start_coords, VIP_goal_pos)
    
    vip_interpolated_path = interpolate_path(vip_path, speed)
    for vip_pos in vip_interpolated_path:
        prob.environment.draw_state(curr_state)
        draw(vip_end_point, color="#a600ff")
        draw(sg.Point2(vip_pos[0], vip_pos[1]), color="#e97419")
        plt.axis("off")
        plt.savefig(pic_file_dir + str(f).zfill(5), bbox_inches='tight')
        plt.clf()
        f += 1
    
    images_to_video(pic_file_dir, video_name, 30)

def main():
    env_files = ["../Envs/rooms.json",
                "../Envs/rooms2.json",
                "../Envs/interesting.json",
                "../Envs/pursuit_fail.json",
                "../Envs/new_env.json",
                "../Envs/new_env3.json"]
    # env_files = ["../Envs/tetris_env.json"]
                # "../Envs/rooms3.json",
    env_files = ["../Envs/rooms3.json"]
    # env_files = ["../Envs/tetris_env.json"]
    # env_files = ["../Envs/interesting.json"]
    for env_file in env_files:
        video_name = env_file.replace("../Envs/", "").replace(".json", "_video.mp4")
        make_escort_problem_video(env_file, video_name)


if __name__ == "__main__":
    main()