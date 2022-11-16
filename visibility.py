from skgeom import *
from skgeom.draw import *
from matplotlib import pyplot as plt
from general_polygon import GeneralPolygon

# Documentation and Demos used in this code:
# https://matplotlib.org/stable/gallery/event_handling/coords_demo.html
# https://scikit-geometry.github.io/scikit-geometry/introduction.html
# general_polygon.py

class VisPoly:
    def __init__(self, line, gp):
        self.line = line
        self.xs = line.get_xdata()
        print("Setting self xs to ", self.xs)
        self.ys = line.get_ydata()
        print("Setting self yx to ", self.ys)
        self.ys = line.get_ydata()
        self.arran = gp.arrangement
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)
        # self.cid = line.figure.canvas.mpl_connect('motion_notify_event', self)

    def __call__(self, event):
        print('click', event)
        # check if click is inside of line axes
        if event.inaxes!=self.line.axes: return
        if not gp.contains(event.xdata, event.ydata): 
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
        # self.compute_shadows(vis_p)
        self.line.figure.canvas.draw()

    def env_res(self):
        for ha in self.arran.halfedges:
            draw(ha.curve())


    def compute_visib_pursue(self):
        vs = RotationalSweepVisibility(self.arran)

        p = Point2(self.xs, self.ys)
        face_p = arr.find(p)
        vs_p = vs.compute_visibility(p, face_p)


        for j in vs_p.halfedges:

            # compute midpoint of newly added point
            # if midpoint is in floating space, it is an occlusion ray
            # this method is not working properly yet
            # i will also move this to another function
            midpoint = Point2(  ((j.curve()[0].x() + j.curve()[0].y()) / 2),((j.curve()[1].x() + j.curve()[1].y()) / 2) ) 


            # if segment is occlusion ray 
            if ( type(self.arran.find(midpoint)) is skgeom._skgeom.arrangement.Face ): 
                draw(j.curve(), color='black', visible_point = False)
            else:
                draw(j.curve(), color='red', visible_point = False)
        
        return vs_p

    def build_polygon_set_from_arrangement(self, arr):
        polys = []
        for f in arr.faces:
            if f.is_unbounded():
                continue
            
            if f.has_outer_ccb():
                poly_pts= []
                outer_ccb_circulator = f.outer_ccb
                first = next(outer_ccb_circulator)
                circ = next(outer_ccb_circulator)
                while circ != first:
                    poly_pts.append(circ.source().point())
                    circ = next(outer_ccb_circulator)
                poly_pts.append(circ.source().point())
                polys.append(skgeom.Polygon(poly_pts))

        return skgeom.PolygonSet(polys)  


    def compute_shadows(self, visible_arr):
        #shadows = self.arran.difference(visible_arr)
        #print("computing shadows")
        x_polyset = self.build_polygon_set_from_arrangement(visible_arr)
        y_polyset = self.build_polygon_set_from_arrangement(self.arran)

        local = y_polyset.difference(x_polyset)
        for pol in local.polygons:
            draw(pol, facecolor="lightblue")
        #gps = [GeneralPolygon([x]) for x in local.polygons]
        #for j in shadows.halfedges:
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