from skgeom import *
from skgeom.draw import *
from matplotlib import pyplot as plt

class LineBuilder:
    def __init__(self, line, arr):
        self.line = line
        self.xs = line.get_xdata()
        self.ys = line.get_ydata()
        self.arran = arr
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)

    def __call__(self, event):
        print('click', event)
        if event.inaxes!=self.line.axes: return
        self.xs = event.xdata
        self.ys = event.ydata
        self.line.set_data(self.xs, self.ys)
        #self.env_res()
        plt.cla()
        self.env_res()
        plt.plot(event.xdata, event.ydata, '.')
        vis_p = self.compute_visib_pursue()
        self.compute_shadows(vis_p)
        self.line.figure.canvas.draw()

    def env_res(self):
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

        self.arran = arr


    def compute_visib_pursue(self):
        vs = RotationalSweepVisibility(self.arran)

        p = Point2(self.xs, self.ys)
        face_p = arr.find(p)
        vs_p = vs.compute_visibility(p, face_p)

        for j in vs_p.halfedges:
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

        x_polyset = self.build_polygon_set_from_arrangement(visible_arr)
        y_polyset = self.build_polygon_set_from_arrangement(self.arran)

        local = y_polyset.difference(x_polyset)
        for pol in local.polygons:
            draw(pol, facecolor="lightblue")
        #gps = [GeneralPolygon([x]) for x in local.polygons]
        #for j in shadows.halfedges:
        #    draw(j.curve(), color="green", visible_arr = False)






fig, ax = plt.subplots()

# Set up enviorment
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



#ax.set_title('click to build line segments')
line, = ax.plot(0, 0)  # empty line
linebuilder = LineBuilder(line, arr)

plt.show()