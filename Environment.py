import numpy as np
from skgeom import *
from skgeom.draw import *
from matplotlib import pyplot as plt
from general_polygon import GeneralPolygon
from scikit_utils import *
from State import *
import time


class Environment:

    def __init__ (self, gp, halves, adj_list):
        self.gp = gp
        self.arran =  gp.arrangement
        self.halves = halves
        self.full_polyset = self.remove_holes()
        self.adj_list = adj_list
        # visibility = self.compute_visib_pursue(position)
        # self.shadows = self.compute_shadows(visibility, num)
        # self.safezones = self.compute_safezones()

    def remove_holes(self):
        # this function is used to calculate full polyset without holes
        # for use in shadow drawing function

        holeArrangement = sg.arrangement.Arrangement()
        for i, poly in enumerate(self.gp.polygons):
            for j, hole in enumerate(poly.holes):
                for e in hole.edges:
                    holeArrangement.insert(e)
        y_polyset = build_polygon_set_from_arrangement(self.arran)
        hole_polyset = build_polygon_set_from_arrangement(holeArrangement)
        return y_polyset.difference(hole_polyset)

    def compute_visib_pursue(self, pos):
        vs = sg.RotationalSweepVisibility(self.arran)

        face_p = self.arran.find(pos)
        vs_p = vs.compute_visibility(pos, face_p)

        return vs_p
    
    def compute_shadows(self, visible_arr):
        x_polyset = build_polygon_set_from_arrangement(visible_arr)

        local = self.full_polyset.difference(x_polyset)

        # for pol in local.polygons:
        #     if(num == 1):
        #         draw(pol, facecolor="lightblue")
 
        #     if(num ==2):
        #         draw(pol, facecolor="lightgreen")
        return local

    def compute_safezones(self, pos):
        vs = sg.RotationalSweepVisibility(self.arran)
        # pos = self.position
        face_p = self.arran.find(pos)
        vs_p = vs.compute_visibility(pos, face_p)


        unsafe_zones = np.array([])

        for j in vs_p.halfedges:
            if ( self.compute_occlusion(j)): 
                unsafe = self.compute_unsafe_zone(j, vs)
                unsafe_zones = np.append(unsafe_zones, unsafe)
        
        union_boolean = sg.PolygonSet([])
        for vs_c in unsafe_zones:
            polyset = build_polygon_set_from_arrangement(vs_c)
            union_boolean = union_boolean.union(polyset)
        
        
        shadows_set = self.compute_shadows(vs_p)
        # polygon_set = build_polygon_set_from_arrangement(self.shadows)
        # Extract safe regions
        covered = union_boolean.union(shadows_set)
        # vis_pur = build_polygon_set_from_arrangement(vs_p)
        # covered = union_boolean.union(vis_pur)
        # draw(covered, facecolor="violet")

        enviornment_arr = build_polygon_set_from_arrangement(self.arran)
        safezones = enviornment_arr.difference(covered)
        #draw(safezones, facecolor="green")

        return safezones


    def compute_occlusion(self, j):
        # Creates segment and sees if it overlaps
        seg = sg.Segment2(j.source().point(), j.target().point())

        for edge in self.halves:
            if isinstance(sg.intersection(seg, edge), sg.Segment2):
                return False
        return True

    def compute_unsafe_zone(self, j, visibility):
        # divide segment in to 10 points for now
        pt1 = np.array( (j.curve()[0].x(), j.curve()[0].y()) )
        pt2 = np.array( (j.curve()[1].x(), j.curve()[1].y()) )
        # replace 10 with number proportional to length of segment using np.linalg.norm
        points = np.linspace( pt1, pt2 , 10)[1:-1]


        visibile_edges = np.array([])

        for point in points:

            current_point = sg.Point2(point[0], point[1])
            # draw(current_point, color = 'yellow')
            current_face = self.arran.find(current_point)
            vs_current = visibility.compute_visibility(current_point, current_face)

            visibile_edges = np.append(visibile_edges, vs_current)

        return visibile_edges

    def transition_blackbox(self, state, new_position):
        '''
        Given a state, and a position for the escort to move to return new state corresponding to new escort position
        '''   
        pos = sg.Point2(new_position[0],new_position[1])
        # new_state.parent = state
        v_poly = self.compute_visib_pursue(pos)
        shadows = self.compute_shadows(v_poly)

        intersect = state.contaminated_shadows.intersection(shadows)

        # Determine the contaminated shadows
        contain_shadows = np.array([])
        for pol in shadows.polygons:
            for p in intersect.polygons:
                if len(sg.boolean_set.intersect(pol, p)) > 0:
                    # print("Contaiminated!")
                    # draw(pol, facecolor = "pink")
                    contain_shadows = np.append(contain_shadows, pol)

        polygon_set = sg.PolygonSet([py for py in contain_shadows])

        new_shadows = polygon_set

        contain_poly_arr = sg.arrangement.Arrangement()
        for v_s in v_poly.halfedges:
            if not (polygon_set.locate(v_s.source().point()) and polygon_set.locate(v_s.target().point())):
                continue
            else:
                contain_poly_arr.insert(v_s.curve())

        boundary_contaminated_shadows = np.array([])
        contaiminated_edges = np.array([])
        vs = sg.RotationalSweepVisibility(self.arran)
        for v in contain_poly_arr.halfedges:
            if(self.compute_occlusion(v)):
                boundary_contaminated_shadows = np.append(boundary_contaminated_shadows, v)
                contaimin_edges = self.compute_unsafe_zone(v,vs)
                contaiminated_edges = np.append(contaiminated_edges, contaimin_edges)


        union_boolean = sg.PolygonSet([])
        for vs_c in contaiminated_edges:
            polyset = build_polygon_set_from_arrangement(vs_c)
            
            union_boolean = union_boolean.union(polyset)
        # draw(union_boolean, facecolor="violet")

        # Extract safe regions
        covered = union_boolean.union(polygon_set)

        # plt.cla()
        # self.env_res()
        # draw(covered, facecolor="maroon")

        enviornment_arr = build_polygon_set_from_arrangement(self.arran)
        current_safezones = enviornment_arr.difference(covered)
        # draw(current_safezones, facecolor="green")

        vip_contain = np.array([])
        for safe in state.safezones.polygons:
            for poly_safe in current_safezones.polygons:
                if len(sg.boolean_set.intersect(safe, poly_safe)) > 0:
                    vip_contain = np.append(vip_contain, poly_safe)
                    # draw(poly_safe, facecolor="white")    

        polygon_vip = sg.PolygonSet([px for px in vip_contain])
        new_vip_safezones = polygon_vip
        new_state = State(position=new_position, parent=state, contaminated_shadows=new_shadows,\
                            safezones=new_vip_safezones, neighbors=self.adj_list[new_position])

        return new_state

    def get_starting_state(self, pos):
        '''Return starting state, every shadow is contaminated'''
        p = sg.Point2(pos[0], pos[1])
        v_poly = self.compute_visib_pursue(p)
        start_shadows = self.compute_shadows(v_poly)
        start_safezones = self.compute_safezones(p)
        return State(pos, parent=None, contaminated_shadows=start_shadows, safezones=start_safezones, neighbors=self.adj_list[pos])

    def draw_state(self, state):
        '''Draw the environment with the current state'''
        draw(self.gp)
        draw(state.safezones, facecolor="green")
        draw(state.contaminated_shadows, facecolor="red")
        draw(state.as_point(), color="blue")
        
    def env_res(self):
        for ha in self.arran.halfedges:
            draw(ha.curve())
    