import numpy as np
from skgeom import *
from skgeom.draw import *
from matplotlib import pyplot as plt
from general_polygon import GeneralPolygon
from scikit_utils import *
import time

class State:

    def __init__ (self, gp, position, halves, num):
        self.gp = gp
        self.position = position
        self.arran =  gp.arrangement
        self.halves = halves
        self.full_polyset = self.remove_holes()
        visibility = self.compute_visib_pursue(position)
        self.shadows = self.compute_shadows(visibility, num)
        self.safezones = self.compute_safezones()
        self.allshadows = self.compute_shadows(visibility, num)
        self.allsafezones = []
        
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
                draw(pol, facecolor="lightblue")
        return local

    def compute_safezones(self):
        vs = RotationalSweepVisibility(self.arran)
        p = self.position
        face_p = self.arran.find(p)
        vs_p = vs.compute_visibility(p, face_p)


        unsafe_zones = np.array([])

        for j in vs_p.halfedges:
            if ( self.compute_occlusion(j)): 
                unsafe = self.compute_unsafe_zone(j, vs)

                unsafe_zones = np.append(unsafe_zones, unsafe)
        
        union_boolean = PolygonSet([])
        for vs_c in unsafe_zones:
            polyset = build_polygon_set_from_arrangement(vs_c)
            
            union_boolean = union_boolean.union(polyset)
        

        #polygon_set = build_polygon_set_from_arrangement(self.shadows)
        # Extract safe regions
        covered = union_boolean.union(self.shadows)
        #draw(covered, facecolor="violet")

        enviornment_arr = build_polygon_set_from_arrangement(self.arran)
        safezones = enviornment_arr.difference(covered)
        #draw(safezones, facecolor="green")

        return safezones


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
            #draw(current_point, color = 'yellow')
            current_face = self.arran.find(current_point)
            vs_current = visibility.compute_visibility(current_point, current_face)

            visibile_edges = np.append(visibile_edges, vs_current)

        return visibile_edges

    def new_state(self, escort_pos):

        newstate = State(self.gp, escort_pos, self.halves, num=2)

        v_poly = self.compute_visib_pursue(escort_pos)
        shadows = self.compute_shadows(v_poly,2)

        intersect = self.shadows.intersection(shadows)

        # Determine the contaminated shadows
        contain_shadows = np.array([])
        for pol in shadows.polygons:
            draw(pol, facecolor = "lightblue")
            for p in intersect.polygons:
                if len(boolean_set.intersect(pol, p)) > 0:
                    print("Contaiminated!")
                    #draw(pol, facecolor = "pink")
                    contain_shadows = np.append(contain_shadows, pol)

        polygon_set = PolygonSet([py for py in contain_shadows])
        newstate.shadows = polygon_set

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


        union_boolean = PolygonSet([])
        for vs_c in contaiminated_edges:
            polyset = build_polygon_set_from_arrangement(vs_c)
            
            union_boolean = union_boolean.union(polyset)
        #draw(union_boolean, facecolor="violet")

        # Extract safe regions
        covered = union_boolean.union(polygon_set)

        plt.cla()
        self.env_res()
        #draw(covered, facecolor="maroon")

        enviornment_arr = build_polygon_set_from_arrangement(self.arran)
        current_safezones = enviornment_arr.difference(covered)
        #draw(current_safezones, facecolor="green")

        vip_contain = np.array([])
        temp = np.array([])
        for safe in self.safezones.polygons:
            for poly_safe in current_safezones.polygons:
                temp = np.append(temp, poly_safe)
                if len(boolean_set.intersect(safe, poly_safe)) > 0:
                    vip_contain = np.append(vip_contain, poly_safe)
                    #draw(poly_safe, facecolor="white")    

        newstate.allsafezones = PolygonSet([pt for pt in temp])
        polygon_vip = PolygonSet([px for px in vip_contain])
        newstate.safezones = polygon_vip

        return newstate
        

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
    state = State(gp, pos, np_half, 1)

    plt.cla()
    state.env_res()
    for g in state.shadows.polygons:
        draw(g, facecolor = "pink")
        print("HERE")
    for h in state.safezones.polygons:
        draw(h, facecolor = "lightgreen")
    draw(pos, color = "black")

    # Test purpose
    plt.savefig("test_results/current_state.png")



    next_pos = Point2(6,5)
    plt.cla()
    state.env_res()
    draw(next_pos, color = "red")
    plt.cla()
    future_state = state.new_state(next_pos)

    
    future_state.env_res()
    
    #n_p = Point2(6,8)
    #plt.cla()
    #n_state = future_state.new_state(n_p)
    #uture_state.env_res()
    
    #for y in future_state.allshadows.polygons:
       # draw(y, facecolor = "lightblue")
    for g in future_state.shadows.polygons:
        draw(g, facecolor = "pink")
    
    for h in future_state.safezones.polygons:
        draw(h, facecolor = "lightgreen")
    draw(next_pos, color = "black")
    
    # Test Purpose
    plt.savefig("test_results/next_state.png")

    plt.cla()
    future_state.env_res()
    #
    for g in future_state.shadows.polygons:
        draw(g, facecolor = "pink")
    for g in future_state.allsafezones.polygons:
        draw(g, facecolor = "thistle")
    for t in future_state.safezones.polygons:
       draw(t, facecolor = "lightgreen")
    
    draw(next_pos, color = "black")
    
    plt.savefig("test_results/next_state_safe.png")
    
    plt.show()