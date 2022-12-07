import numpy as np
from skgeom import *
from skgeom.draw import *
from matplotlib import pyplot as plt
from general_polygon import GeneralPolygon
from scikit_utils import *
import time

class State:

    def __init__ (self, gp, position):
        self.gp = gp
        self.arran =  gp.arrangement
        self.full_polyset = self.remove_holes()
        visibility = self.compute_visib_pursue(position)
        self.shadows = self.compute_shadows(visibility)
        self.safezones = []
        
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

    def compute_visib_pursue(self, pos):
        vs = RotationalSweepVisibility(self.arran)
        #p = Point2(pos)
        face_p = self.arran.find(pos)
        vs_p = vs.compute_visibility(pos, face_p)

        return vs_p
    
    def compute_shadows(self, visible_arr):
        x_polyset = build_polygon_set_from_arrangement(visible_arr)

        local = self.full_polyset.difference(x_polyset)
        for pol in local.polygons:
            draw(pol, facecolor="lightblue")

        return local

    def new_state(self, escort_pos):
        v_poly = self.compute_visib_pursue(escort_pos)
        shadows = self.compute_shadows(v_poly)

        intersect = self.shadows.intersection(shadows)

        # Determine the contaminated shadows
        contain_shadows = np.array([])
        for pol in shadows.polygons:
            for p in intersect.polygons:
                if len(boolean_set.intersect(pol, p)):
                    print("Contaiminated!")
                    draw(pol, facecolor = "pink")
                    contain_shadows = np.append(contain_shadows, pol)
        
        
        edge_points = np.array([])
        for con in contain_shadows:
            con_outer = con.outer_boundary()
            for vertex in con_outer.vertices:
                edge_points = np.append(edge_points, Point2(float(vertex.x()),float(vertex.y())))

        vs = RotationalSweepVisibility(self.arran)

        vis_edges = np.array([])
        for edge in edge_points:
            edge_face = self.arran.find(edge)
            if (not (isinstance(edge, Vertex))):

                edge_vis = vs.compute_visibility(edge, edge_face)
                #


        contain_vip = np.array([])
        


        

    def env_res(self):
        for ha in self.arran.halfedges:
            draw(ha.curve())


if __name__ == '__main__':
    
    fig, ax = plt.subplots()

    gp = GeneralPolygon.load_from_json("Envs/rooms.json", verbose=True)
    gp.build_arrangement(verbose=True)

    pos = Point2(6,2)
    state = State(gp, pos)

    state.env_res()


    next_pos = Point2(6,5)
    state.new_state(next_pos)
    
    

    plt.show()