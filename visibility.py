from skgeom import *
from skgeom.draw import *
from matplotlib import pyplot as plt
from general_polygon import GeneralPolygon
from scikit_utils import *
import time

# Documentation and Demos used in this code:
# https://matplotlib.org/stable/gallery/event_handling/coords_demo.html
# https://scikit-geometry.github.io/scikit-geometry/introduction.html
# general_polygon.py

class VisPoly:
    def __init__(self, line, gp, halve):
        self.gp = gp
        self.line = line
        self.halves = halve
        self.xs = line.get_xdata()
        self.ys = line.get_ydata()
        self.arran = gp.arrangement
        self.full_polyset = self.remove_holes()
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)
        # self.cid = line.figure.canvas.mpl_connect('motion_notify_event', self)
        self.env_res()

    def __call__(self, event):
        # print('click', event)
        # check if click is inside of line axes
        if event.inaxes!=self.line.axes or not self.gp.contains(event.xdata, event.ydata): 
            print("NOT IN BOUNDS")
            return
        # update position of x,y point on screen
        self.xs = event.xdata 
        self.ys = event.ydata
        self.line.set_data(self.xs, self.ys)
        # clear environment
        plt.cla()
        self.env_res()
        # plot point of escort
        plt.plot(event.xdata, event.ydata, '.')
        # compute viz of escort
        vis_p, vis_shadow = self.compute_visib_pursue()
        self.compute_shadow_vis(vis_shadow)
        self.compute_shadows(vis_p)
        self.line.figure.canvas.draw()

    def env_res(self):
        ax.set_title('click to build line segments')
        for ha in self.arran.halfedges:
            draw(ha.curve())

    def remove_holes(self):
        # this function is used to calculate full polyset without holes
        # for use in shadow drawing function

        holeArrangement = arrangement.Arrangement()
        for i, poly in enumerate(self.gp.polygons):
            for j, hole in enumerate(poly.holes):
                for e in hole.edges:
                    holeArrangement.insert(e)
        y_polyset = build_polygon_set_from_arrangement(self.arran)
        hole_polyset = build_polygon_set_from_arrangement(holeArrangement)
        return y_polyset.difference(hole_polyset)

    def compute_occlusion(self,j,p):
        # Creates segment and sees if it overlaps
        seg = Segment2(j.source().point(), j.target().point())

        for edge in self.halves:
            if isinstance(intersection(seg, edge), Segment2):
                return False
        return True

    def compute_unsafe_zone(self,j, visibility):
        # divide segment in to 10 points for now
        pt1 = np.array( (j.curve()[0].x(), j.curve()[0].y()) )
        pt2 = np.array( (j.curve()[1].x(), j.curve()[1].y()) )
        # replace 10 with number proportional to length of segment using np.linalg.norm
        points = np.linspace( pt1, pt2 , 10)[1:-1]

        #print("POINTS:", points)

        visibile_edges = np.array([])

        for point in points:

            current_point = Point2(point[0], point[1])
            current_face = arr.find(current_point)
            vs_current = visibility.compute_visibility(current_point, current_face)

            visibile_edges = np.append(visibile_edges, vs_current)

        # for p in points:
        #     draw(p, color='blue', visible_point = True)

        return visibile_edges
        

    def compute_visib_pursue(self):
        vs = RotationalSweepVisibility(self.arran)
        p = Point2(self.xs, self.ys)
        face_p = arr.find(p)
        vs_p = vs.compute_visibility(p, face_p)


        unsafe_zones = np.array([])

        for j in vs_p.halfedges:
            # if segment is occlusion ray
            if ( self.compute_occlusion(j,p) ): 
                draw(j.curve(), color='red', visible_point = False)
                unsafe = self.compute_unsafe_zone(j, vs)

                unsafe_zones = np.append(unsafe_zones, unsafe)
            else:
                draw(j.curve(), color='black', visible_point = False)
        return vs_p, unsafe_zones


    def compute_shadow_vis(self, shadow_vis_edges):
        total_polyset = PolygonSet()
        for vis in shadow_vis_edges:
            polyset = build_polygon_set_from_arrangement(vis)
            total_polyset = total_polyset.union(polyset)

            # for poly in polyset.polygons:
            #     draw(poly, facecolor = "lightgreen")
        for poly in total_polyset.polygons:
            draw(poly, facecolor = "lightgreen")


    def compute_shadows(self, visible_arr):
        # shadows = self.arran.difference(visible_arr)
        #print("computing shadows")
        x_polyset = build_polygon_set_from_arrangement(visible_arr)

        local = self.full_polyset.difference(x_polyset)
        for pol in local.polygons:
            draw(pol, facecolor="lightblue")
        #gps = [GeneralPolygon([x]) for x in local.polygons]
        # for j in shadows.halfedges:
        #    draw(j.curve(), color="green", visible_arr = False)


def env_setup():
    # Set up environment
    M = 50
    boundary = [
        Segment2(Point2(-M, -M), Point2(-M, M)), Segment2(Point2(-M, M), Point2(M, M)),
        Segment2(Point2(M, M), Point2(M, -M)), Segment2(Point2(M, -M), Point2(-M, -M))
    ]
    box = [
        Segment2(Point2(30, -30), Point2(-30, 30)), #Segment2(Point2(-30, 30), Point2(30, 30)),
        Segment2(Point2(30, 30), Point2(30, -30)), Segment2(Point2(30, -30), Point2(-30, -30))
    ]
    arr = arrangement.Arrangement()
    for s in boundary:
        arr.insert(s)
    for s in box:
        arr.insert(s)
    for ha in arr.halfedges:
        draw(ha.curve())
    return arr 

if __name__ == '__main__':
    fig, ax = plt.subplots()

    gp = GeneralPolygon.load_from_json("Envs/rooms.json", verbose=True)
    gp.build_arrangement(verbose=True)

    np_half = np.array([])

    # Draw the arrangement
    for he in gp.arrangement.halfedges:
        #np_half = np.append(np_half, (he.source().point().x(), he.source().point().y(), he.target().point().x(), he.target().point().y()))
        np_half = np.append(np_half, Segment2(he.source().point(), he.target().point()))
        draw(he.curve(), visible_point=False)
        
    #np_half = np_half.reshape([-1, 4])
    # arr = env_setup()
    arr = gp.arrangement

    line, = ax.plot(0, 0)  # empty line
    vis_poly = VisPoly(line, gp, np_half)


    plt.show()