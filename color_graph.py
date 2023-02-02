# Goal to visually view the safe zones within the enviornment
from Environment import *
from EscortProblem import *

def get_grid_point(start_pos, interval, gp, coords, halves):
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
        temp_point = Point2(n[0], n[1])
        if not gp.contains(n[0], n[1]):
            neighbors.remove(n)
        
        else:
            if isinstance(halves.find(temp_point), sg.arrangement.Vertex) or isinstance(halves.find(temp_point), sg.arrangement.Halfedge):
                print("Outer")
            else:
                if n not in coords:
                    coords.append(n)
            #else:
            #    print("Duplicate")
        

    return coords

def count_safe_zones(envstate):
    safezones = envstate.safezones
    num = 0
    for poly in safezones.polygons:
        num += 1
    return num


escort_prob = EscortProblem("Envs/rooms.json", (10.0, 16.667), (100, 45))

state = escort_prob.environment.get_starting_state(escort_prob.start_pos)
number = count_safe_zones(state)

grid_points = [escort_prob.start_pos]

for point in grid_points:
    #grid_points = get_grid_point(escort_prob.start_pos, 1, escort_prob.environment.gp, grid_points)
    grid_points = get_grid_point(point, 5, escort_prob.environment.gp, grid_points, escort_prob.environment.arran)
    #print("Start Pos: ", escort_prob.start_pos)
    #print("Neighbors:" , grid_points)



escort_prob.environment.draw_state(state)
for point in grid_points:
    try: 
        #escort_prob = EscortProblem("Envs/rooms.json", point, (100, 45))
        state = escort_prob.environment.get_starting_state(point)
        number = count_safe_zones(state)
        new_point = Point2(point[0], point[1])
        if number == 0:
            draw(new_point, color = "black")
        elif number == 1:
            draw(new_point, color = "lightpink")
        elif number == 2:
            draw(new_point, color = "lightgreen")
        elif number == 3:
            draw(new_point, color = "lightblue")
        elif number == 4:
            draw(new_point, color = "lightpurple")
        else:
            print("More safe zones!")
    except:
        print("Key Error -- Happened")
    
    

plt.show()
