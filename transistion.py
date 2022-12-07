import numpy as np
from skgeom import *
from skgeom.draw import *
from matplotlib import pyplot as plt
from general_polygon import GeneralPolygon
from scikit_utils import *
import time

class State:

    def __init__ (self, gp, position, halves):
        self.gp = gp
        self.arran =  gp.arrangement
        self.full_polyset = self.remove_holes()
        visibility = self.compute_visib_pursue(position)
        self.shadows = self.compute_shadows(visibility,1)
        self.safezones = []
        self.halves = halves
        
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

        face_p = self.arran.find(pos)
        vs_p = vs.compute_visibility(pos, face_p)

        return vs_p
    
    def compute_shadows(self, visible_arr,num):
        x_polyset = build_polygon_set_from_arrangement(visible_arr)

        local = self.full_polyset.difference(x_polyset)

        for pol in local.polygons:
            if(num == 1):
                draw(pol, facecolor="lightblue")
 
            if(num ==2):
                draw(pol, facecolor="lightgreen")

        # Test purpose
        if (num == 1):
            plt.savefig('test_results/parent.png')
        if (num == 2):
            plt.savefig('test_results/child.png')
        return local

    def compute_occlusion(self,j):
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


        visibile_edges = np.array([])

        for point in points:

            current_point = Point2(point[0], point[1])
            draw(current_point, color = 'yellow')
            current_face = self.arran.find(current_point)
            vs_current = visibility.compute_visibility(current_point, current_face)

            visibile_edges = np.append(visibile_edges, vs_current)

        return visibile_edges

    def new_state(self, escort_pos):
        v_poly = self.compute_visib_pursue(escort_pos)
        shadows = self.compute_shadows(v_poly,2)

        intersect = self.shadows.intersection(shadows)

        # Determine the contaminated shadows
        contain_shadows = np.array([])
        for pol in shadows.polygons:
            for p in intersect.polygons:
                if len(boolean_set.intersect(pol, p)):
                    print("Contaiminated!")
                    draw(pol, facecolor = "pink")
                    contain_shadows = np.append(contain_shadows, pol)
        
        # Test purposes
        plt.savefig('test_results/contaminated_shadows.png')

        polygon_set = PolygonSet([py for py in contain_shadows])

        contain_poly_arr = arrangement.Arrangement()
        for v_s in v_poly.halfedges:
            if not (polygon_set.locate(v_s.source().point()) and polygon_set.locate(v_s.target().point())):
                continue
            else:
                contain_poly_arr.insert(v_s.curve())

        boundary_contaminated_shadows = np.array([])
        contaiminated_edges = np.array([])
        vs = RotationalSweepVisibility(self.arran)
        for v in contain_poly_arr.halfedges:
            if(self.compute_occlusion(v)):
                boundary_contaminated_shadows = np.append(boundary_contaminated_shadows, v)
                contaimin_edges = self.compute_unsafe_zone(v,vs)
                contaiminated_edges = np.append(contaiminated_edges, contaimin_edges)

        # Test Purposes
        plt.savefig('test_results/boundary_contaminated.png')

        union_boolean = PolygonSet([])
        for vs_c in contaiminated_edges:
            polyset = build_polygon_set_from_arrangement(vs_c)
            
            union_boolean = union_boolean.union(polyset)
        draw(union_boolean, facecolor="violet")
        plt.savefig('test_results/visibility_child.png')
                

        

    def env_res(self):
        for ha in self.arran.halfedges:
            draw(ha.curve())


if __name__ == '__main__':
    
    fig, ax = plt.subplots()

    gp = GeneralPolygon.load_from_json("Envs/rooms.json", verbose=True)
    gp.build_arrangement(verbose=True)

    np_half = np.array([])
    for ha in gp.arrangement.halfedges:
        draw(ha.curve())
        np_half = np.append(np_half, Segment2(ha.source().point(), ha.target().point()))


    pos = Point2(6,2)
    draw(pos, color = "black")
    state = State(gp, pos, np_half)

    state.env_res()



    next_pos = Point2(6,5)
    plt.cla()
    state.env_res()
    draw(next_pos, color = "red")
    state.new_state(next_pos)
    
    

    plt.show()