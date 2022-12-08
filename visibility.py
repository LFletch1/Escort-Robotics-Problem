from skgeom import *
from skgeom.draw import *
from matplotlib import pyplot as plt
from general_polygon import GeneralPolygon
from scikit_utils import *
import time
from collections import deque
from transition import State

# Documentation and Demos used in this code:
# https://matplotlib.org/stable/gallery/event_handling/coords_demo.html
# https://scikit-geometry.github.io/scikit-geometry/introduction.html
# general_polygon.py

class VisPoly:
    def __init__(self, line, gp, halve, interval):
        self.gp = gp
        self.line = line
        self.halves = halve
        self.arran = gp.arrangement
        self.full_polyset = build_polygon_set_from_arrangement(gp.arrangement)
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self.click_handle)
        self.cid = line.figure.canvas.mpl_connect('key_press_event', self.key_handle)
        self.shadow_polyset = PolygonSet()
        self.unsafe_polyset = PolygonSet()
        self.shadows = {}
        self.safe_zones = {}
        self.interval = interval
        self(( int(line.get_xdata() ) , int (line.get_ydata()) ))
        # self(( 5 , 5 ))

    # ---------------------------------
    # state is represented by:
    # - where the escort is 
    # - set of shadows and contamination status
    # - set of safe zones and VIP contamination status
    # ---------------------------------
    def bfs(self):

        # class Node:
        #     def __init__(self, coords, shadows, safe_zones, parent):
        #         self.coords = coords
        #         self.shadows = shadows
        #         self.safe_zones = safe_zones
        #         self.parent = parent


        path = []
        startNode = State(gp, Point2(self.xs, self.ys), np_half, 1)
        print(startNode.position)


        # print (type(self.xs))
        # goal = (35,85)
        # print(goal)

        q = deque()

        # q.append((startNode.x, startNode.y))
        q.append(startNode)
        visitedNodes = []

        while q:
            current = q.popleft()
            # print("Current: ", current.x, current.y)

            # print (current.safe_zones.keys() ) 

            # if current.coords == goal:
            #     print("Found goal")

            #     while current.parent:
            #         path.append( current.coords ) 
            #         current = current.parent
                
            #     print("path: ", path)
            #     for i in path:
            #         plt.plot(i[0], i[1], '.')
            #     self.line.figure.canvas.draw()
            #     return
            # for zone in current.safe_zones:
                # print("Zone: ", zone)
                # if zone.contains(current.x, current.y): 
                # if zone.oriented_side(sg.Point2(current.x, current.y)):
                #     print("FOUND ANSWER")
                #     return
                # print (zone.area() )

            neighbors = [
                        (current.position.x()-self.interval, current.position.y()),
                        (current.position.x()+self.interval, current.position.y()),
                        (current.position.x(), current.position.y()-self.interval),
                        (current.position.x(), current.position.y()+self.interval)
                        ]
                        
            for n in neighbors:
                if self.gp.contains(n[0], n[1]): 
                    print("Checking in ", n , " in there->")
                    if not n in visitedNodes:
                        newstate = current.new_state(Point2(n[0], n[1]))
                        q.append( newstate )
                        print("Adding node at ", )

                        visitedNodes.append( n )
                    else:
                        print("ALready in there")


        print(visitedNodes)
        print("Exited")
        










                
    def key_handle(self, event):
        # print(event.key)
        if event.key == 'up':
            self((self.xs, self.ys+self.interval))
        elif event.key == 'down':
            self((self.xs, self.ys-self.interval))
        elif event.key == 'right':
            self((self.xs+self.interval, self.ys))
        elif event.key == 'left':
            self((self.xs-self.interval, self.ys))
        elif event.key == 'B':
            self.bfs()

    def click_handle(self, event):
        # self.env_reset()
        self((event.xdata, event.ydata))

    def __call__(self, coords):
        print(coords[0], coords[1])
        if not self.gp.contains(coords[0], coords[1]): 
            print("NOT IN BOUNDS")
            return
        # update position of x,y point on screen
        self.xs = coords[0] 
        self.ys = coords[1]
        # clear environment
        self.env_res()
        # vis_p = visibility polygon of escort
        # vis_shadow = visibility polygon of shadows (2nd level)
        vis_p, vis_shadow = self.compute_visib_pursue()
        self.compute_shadow_vis(vis_shadow)
        self.compute_shadows(vis_p)
        self.compute_safezones(vis_p, vis_shadow)
        self.line.figure.canvas.draw()

    def env_res(self):
        plt.cla()
        ax.set_title('click to build line segments')
        for ha in self.arran.halfedges:
            draw(ha.curve())
        plt.plot(self.xs, self.ys, '.')


    def draw_poly(self, polygon):
        plt.cla()
        for poly in polygon.polygons:
            draw(poly, facecolor = "lightgreen")
        
    # ---------------------------------
    # compute the visibility of the pursuer
    # returns visibility region of pursuer AND shadows
    # ---------------------------------
    def is_occlusion_ray(self,j,p):
        # Creates segment and sees if it overlaps
        seg = Segment2(j.source().point(), j.target().point())

        for edge in self.halves:
            if isinstance(intersection(seg, edge), Segment2):
                return False
        return True

    # ---------------------------------
    # compute the visibility of the pursuer
    # returns visibility region of pursuer AND shadows
    # ---------------------------------
    def compute_visib_pursue(self):
        vs = RotationalSweepVisibility(self.arran)
        p = Point2(self.xs, self.ys)
        face_p = arr.find(p)
        vis_p = vs.compute_visibility(p, face_p)
        unsafe_zones = np.array([])

        for j in vis_p.halfedges:
            # if segment is occlusion ray
            if ( self.is_occlusion_ray(j,p) ): 
                draw(j.curve(), color='red', visible_point = False)
                unsafe = self.compute_unsafe_zone(j, vs)
                unsafe_zones = np.append(unsafe_zones, unsafe)
            else:
                draw(j.curve(), color='black', visible_point = False)
        return vis_p, unsafe_zones

    # ---------------------------------
    # divide contaminated shadow edges into points
    # calculate unsafe zones from those points
    # ---------------------------------
    def compute_unsafe_zone(self,j, visibility):
        # divide segment in to 10 points for now
        pt1 = np.array( (j.curve()[0].x(), j.curve()[0].y()) )
        pt2 = np.array( (j.curve()[1].x(), j.curve()[1].y()) )
        # replace 10 with number proportional to length of segment using np.linalg.norm
        points = np.linspace( pt1, pt2 , 10)[1:-1]
        visibile_edges = np.array([])
        for point in points:
            current_point = Point2(point[0], point[1])
            current_face = arr.find(current_point)
            vs_current = visibility.compute_visibility(current_point, current_face)
            visibile_edges = np.append(visibile_edges, vs_current)

        return visibile_edges

    def compute_shadow_vis(self, shadow_vis_edges):
        self.unsafe_polyset = PolygonSet()
        for vis in shadow_vis_edges:
            polyset = build_polygon_set_from_arrangement(vis)
            self.unsafe_polyset = self.unsafe_polyset.union(polyset)
        # these are the 'unsafe' regions that are in sight of 
        # contaminated shadows
        for poly in self.unsafe_polyset.polygons:
            draw(poly, facecolor = "orange")

    def compute_shadows(self, visible_arr):
        # update shadow polyset
        # is difference between full polyset and visible region
        x_polyset = build_polygon_set_from_arrangement(visible_arr)
        self.shadow_polyset = self.full_polyset.difference(x_polyset)
        for pol in self.shadow_polyset.polygons:
            draw(pol, facecolor="darkblue")
            # add shadow to shadow dict with false
            self.shadows[pol] = False 

    def compute_safezones(self, vis_p, vis_shadow):
        # update shadow polyset
        # is difference between full polyset and visible region
        # x_polyset = build_polygon_set_from_arrangement(visible_arr)
        self.safe_polyset = Polygon()
        self.safe_polyset = self.full_polyset.difference(self.shadow_polyset)
        self.safe_polyset = self.safe_polyset.difference(self.unsafe_polyset)
        for pol in self.safe_polyset.polygons:
            draw(pol, facecolor="lightgreen")
            # add shadow to shadow dict with false
            self.safe_zones[pol] = False 


if __name__ == '__main__':
    fig, ax = plt.subplots()

    gp = GeneralPolygon.load_from_json("Envs/rooms.json", verbose=True)
    gp.build_arrangement(verbose=True)

    np_half = np.array([])

    # Draw the arrangement
    for he in gp.arrangement.halfedges:
        np_half = np.append(np_half, Segment2(he.source().point(), he.target().point()))
        draw(he.curve(), visible_point=False)
        
    arr = gp.arrangement

    interval = 10
    starting_pos = (5,5)

    line, = ax.plot(starting_pos[0], starting_pos[1])  # empty line
    vis_poly = VisPoly(line, gp, np_half, interval)

    plt.show()