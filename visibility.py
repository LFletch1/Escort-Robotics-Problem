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
    def __init__(self, line, gp):
        self.gp = gp
        self.line = line
        self.xs = line.get_xdata()
        self.ys = line.get_ydata()
        self.arran = gp.arrangement
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)
        # self.cid = line.figure.canvas.mpl_connect('motion_notify_event', self)
        self.full_polyset = self.remove_holes()

    def __call__(self, event):
        # print('click', event)
        # check if click is inside of line axes
        if event.inaxes!=self.line.axes: return
        if not self.gp.contains(event.xdata, event.ydata): 
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
        vis_p = self.compute_visib_pursue()
        self.compute_shadows(vis_p)
        self.line.figure.canvas.draw()

    def env_res(self):
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

    def occlusion(self,j,p):
        # calculate if given half edge is occlusion ray
        # if j is NOT in the arrangement, color it red
        # still not working
        # right now it only returns if the halfedge has a point that is NOT a vertex
        return (not isinstance(self.gp.arrangement.find(j.curve()[0]) , arrangement.Vertex)
        or not isinstance( self.gp.arrangement.find(j.curve()[1]) , arrangement.Vertex) ) 

    def compute_visib_pursue(self):
        vs = RotationalSweepVisibility(self.arran)
        p = Point2(self.xs, self.ys)
        face_p = arr.find(p)
        vs_p = vs.compute_visibility(p, face_p)

        for j in vs_p.halfedges:
            # if segment is occlusion ray 
            if ( self.occlusion(j,p) ): 
                draw(j.curve(), color='red', visible_point = False)
            else:
                draw(j.curve(), color='black', visible_point = False)
        return vs_p

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

    gp = GeneralPolygon.load_from_json("Envs/octbrick_env.json", verbose=True)
    gp.build_arrangement(verbose=True)

    # Draw the arrangement
    for he in gp.arrangement.halfedges:
        draw(he.curve(), visible_point=False)

    # arr = env_setup()
    arr = gp.arrangement

    # ax.set_title('click to build line segments')
    line, = ax.plot(0, 0)  # empty line
    vis_poly = VisPoly(line, gp)

    plt.show()